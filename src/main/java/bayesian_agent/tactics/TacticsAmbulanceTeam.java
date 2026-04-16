package bayesian_agent.tactics;

import adf.core.agent.action.Action;
import adf.core.agent.communication.MessageManager;
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
import bayesian_agent.module.observation.ObservationProcessor;
import bayesian_agent.module.policy.ambulance.AmbulancePolicySelector;
import bayesian_agent.util.Logger;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;
import java.util.Locale;
import java.util.Map;

public class TacticsAmbulanceTeam
        extends adf.core.component.tactics.TacticsAmbulanceTeam {

    private ObservationProcessor    observationProcessor;
    private BeliefManager           beliefManager;
    private AmbulancePolicySelector policySelector;
    private ActionExecutor          actionExecutor;
    private PathPlanning            pathPlanning;

    // Метрики для сравнения эвристика vs POMCP
    private int victimsDelivered   = 0;
    private int victimsLost        = 0;
    private int totalDeliveryTicks = 0;
    private int deliveryCount      = 0;
    private int loadTick           = -1;

    @Override
    public void initialize(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                           ModuleManager mm, MessageManager msg, DevelopData dd) {
        observationProcessor = new ObservationProcessor(ai, wi);
        beliefManager        = new BeliefManager(ai, wi);
        actionExecutor       = new ActionExecutor(ai, wi, si);
        pathPlanning         = mm.getModule("TacticsAmbulanceTeam.PathPlanning",
                                            "adf.impl.module.algorithm.DijkstraPathPlanning");
        policySelector       = new AmbulancePolicySelector(ai, wi, pathPlanning);
        
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

        // Per-victim entropy - только в debug-режиме (verbose)
        Belief belief = beliefManager.getBelief();
        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            double entropy = 0;
            double[] probs = {vb.pHealthy, vb.pInjured, vb.pCritical, vb.pDead};
            for (double p : probs) {
                if (p > 1e-9) entropy -= p * Math.log(p);
            }
            Logger.debug(ai, "victim=" + e.getKey()
                + " " + vb
                + " H=" + String.format(Locale.US, "%.3f", entropy));
        }

        // Компактный снимок belief для post-analysis (POMCP, сравнение методов)
        if (ai.getTime() % 25 == 0) {
            int n = belief.victims.size();
            double sumPCrit = 0, sumH = 0;
            for (Belief.VictimBelief vb : belief.victims.values()) {
                sumPCrit += vb.pCritical;
                double[] ps = {vb.pHealthy, vb.pInjured, vb.pCritical, vb.pDead};
                for (double p : ps) if (p > 1e-9) sumH -= p * Math.log(p);
            }
            Logger.info(ai, "[BELIEF_SNAP] t=" + ai.getTime()
                + " victims=" + n
                + " avg_pCrit=" + String.format(Locale.US, "%.3f", n > 0 ? sumPCrit / n : 0)
                + " avg_H=" + String.format(Locale.US, "%.3f", n > 0 ? sumH / n : 0)
                + " blocked=" + belief.blockedRoads.size()
                + " burning=" + belief.burningBuildings.size());
        }

        policySelector.select(belief);
        bayesian_agent.module.policy.AgentAction selectedAction = policySelector.getSelectedAction();

        // Трекинг метрик
        if (selectedAction.type == bayesian_agent.module.policy.AgentAction.Type.LOAD) {
            loadTick = ai.getTime();
        }
        if (selectedAction.type == bayesian_agent.module.policy.AgentAction.Type.UNLOAD
                && ai.someoneOnBoard() != null && loadTick >= 0) {
            victimsDelivered++;
            totalDeliveryTicks += ai.getTime() - loadTick;
            deliveryCount++;
            loadTick = -1;
        }
        if (ai.getTime() % 50 == 0) {
            // victimsLost = удалены из belief как мёртвые, кроме доставленных
            victimsLost = Math.max(0, beliefManager.getVictimsRemovedAsDead() - victimsDelivered);
            double avg = deliveryCount > 0 ? (double) totalDeliveryTicks / deliveryCount : 0;
            Logger.info(ai, "[METRICS] t=" + ai.getTime()
                + " delivered=" + victimsDelivered
                + " lost=" + victimsLost
                + " avgTicks=" + String.format(Locale.US, "%.1f", avg)
                + " state=" + policySelector.getMacroState());
        }

        Logger.info(ai, "[STATE] macro=" + policySelector.getMacroState()
            + " target=" + policySelector.getTargetVictimId());

        Action action = actionExecutor.translate(selectedAction);

        Logger.info(ai, "obs=" + observationProcessor.getObservation() + " belief=" + beliefManager.getBelief());
        Logger.info(ai, "action=" + selectedAction);

        return action;
    }
}