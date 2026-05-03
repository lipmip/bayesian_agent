package bayesian_agent.tactics;

import adf.core.agent.action.Action;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.information.MessageCivilian;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.action.ActionExecutor;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.belief.BeliefManager;
import bayesian_agent.module.communication.CommunicationManager;
import bayesian_agent.module.observation.ObservationProcessor;
import bayesian_agent.module.policy.fire.FireBrigadePolicySelector;
import bayesian_agent.util.Logger;
import rescuecore2.standard.entities.Civilian;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;
import java.util.HashSet;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

public class TacticsFireBrigade extends adf.core.component.tactics.TacticsFireBrigade {

    private ObservationProcessor      observationProcessor;
    private BeliefManager             beliefManager;
    private FireBrigadePolicySelector policySelector;
    private ActionExecutor            actionExecutor;
    private PathPlanning              pathPlanning;

    private static final boolean USE_COMM = Boolean.parseBoolean(
            System.getProperty("bayesian.use_comm", "true"));

    private final CommunicationManager commManager       = new CommunicationManager();
    private final Set<EntityID>        broadcastedVictims = new HashSet<>();

    @Override
    public void initialize(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                           ModuleManager mm, MessageManager msg, DevelopData dd) {
        observationProcessor = new ObservationProcessor(ai, wi);
        beliefManager        = new BeliefManager(ai, wi);
        pathPlanning         = mm.getModule("TacticsFireBrigade.PathPlanning",
                                            "adf.impl.module.algorithm.DijkstraPathPlanning");
        policySelector       = new FireBrigadePolicySelector(ai, wi, pathPlanning);        actionExecutor       = new ActionExecutor(ai, wi, si);
        
        Logger.info(ai, "initialized");
    }

    @Override
    public void precompute(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                           ModuleManager mm, PrecomputeData pd, DevelopData dd) {
        Logger.info(ai, "precompute (stub)");
    }

    @Override
    public void resume(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                       ModuleManager mm, PrecomputeData pd, DevelopData dd) {
        Logger.info(ai, "resumed");
    }

    @Override
    public void preparate(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                          ModuleManager mm, DevelopData dd) {
    }

    @Override
    public Action think(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                        ModuleManager mm, MessageManager msg, DevelopData dd) {
        Logger.info(ai, "tick " + ai.getTime());

        ChangeSet cs = ai.getChanged();
        observationProcessor.process(cs);

        beliefManager.update(observationProcessor.getObservation());
        Belief belief = beliefManager.getBelief();

        // Per-victim entropy - debug-режим
        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            double entropy = 0;
            double[] probs = {vb.pHealthy, vb.pInjured, vb.pCritical, vb.pDead};
            for (double p : probs) if (p > 1e-9) entropy -= p * Math.log(p);
            Logger.debug(ai, "victim=" + e.getKey() + " " + vb
                + " H=" + String.format(Locale.US, "%.3f", entropy));
        }

        if (ai.getTime() % 50 == 0) {
            Logger.info(ai, "[FIRE_METRICS] t=" + ai.getTime()
                + " burning=" + belief.burningBuildings.size()
                + " fireIntensityKnown=" + belief.buildingFireIntensity.size()
                + " victims_tracked=" + belief.victims.size());
        }

        // Коммуникация: отправить засыпанных жертв, принять от других агентов
        if (USE_COMM) {
            // Отправить: один MessageCivilian за тик (голосовой канал = max 1 сообщение)
            // Приоритет: только что освобождённая жертва (buriedness=0) > новая засыпанная с макс. уроном
            Civilian bestToSend = null;
            boolean bestIsFreed = false;
            for (EntityID id : cs.getChangedEntities()) {
                StandardEntity e = wi.getEntity(id);
                if (!(e instanceof Civilian)) continue;
                Civilian civ = (Civilian) e;
                if (!civ.isBuriednessDefined() || !civ.isHPDefined()) continue;
                if (civ.getHP() <= 0) continue;
                boolean freed = civ.getBuriedness() == 0;
                boolean buried = civ.getBuriedness() > 0;
                if (buried && broadcastedVictims.contains(id)) continue;
                if (bestToSend == null) { bestToSend = civ; bestIsFreed = freed; continue; }
                // Освобождённые важнее засыпанных
                if (freed && !bestIsFreed) { bestToSend = civ; bestIsFreed = true; continue; }
                if (!freed && bestIsFreed) continue;
                // Среди равных - выбираем с наибольшим уроном
                if (civ.isDamageDefined() && bestToSend.isDamageDefined()
                        && civ.getDamage() > bestToSend.getDamage()) {
                    bestToSend = civ; bestIsFreed = freed;
                }
            }
            if (bestToSend != null) {
                commManager.sendVictimFound(msg, bestToSend);
                if (bestToSend.getBuriedness() > 0) broadcastedVictims.add(bestToSend.getID());
                Logger.info(ai, "[COMM] sent VictimFound victim=" + bestToSend.getID()
                    + " buried=" + bestToSend.getBuriedness() + " hp=" + bestToSend.getHP());
            }

            // Принять: обновить policySelector данными от санитаров
            for (MessageCivilian mc : commManager.receiveVictimInfo(msg)) {
                if (!mc.isPositionDefined()) continue;
                int buried = mc.isBuriednessDefined() ? mc.getBuriedness() : -1;
                if (buried < 0) continue;
                policySelector.addCommunicatedVictim(mc.getAgentID(), mc.getPosition(), buried);
                Logger.info(ai, "[COMM] received VictimInfo victim=" + mc.getAgentID()
                    + " buried=" + buried + " pos=" + mc.getPosition());
            }
        }

        policySelector.select(belief);
        Action action = actionExecutor.translate(policySelector.getSelectedAction());

        Logger.info(ai, "obs=" + observationProcessor.getObservation() + " belief=" + belief);
        Logger.info(ai, "action=" + policySelector.getSelectedAction());

        return action;
    }
}