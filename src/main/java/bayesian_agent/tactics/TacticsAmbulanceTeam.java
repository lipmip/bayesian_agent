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
import bayesian_agent.module.policy.AgentAction;
import bayesian_agent.module.policy.ambulance.AmbulancePolicySelector;
import bayesian_agent.module.pomcp.AgentMacroState;
import bayesian_agent.module.pomcp.POMCPPlanner;
import bayesian_agent.util.Logger;
import rescuecore2.standard.entities.Civilian;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;
import java.util.HashSet;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

public class TacticsAmbulanceTeam
        extends adf.core.component.tactics.TacticsAmbulanceTeam {

    private ObservationProcessor    observationProcessor;
    private BeliefManager           beliefManager;
    private AmbulancePolicySelector policySelector;
    private ActionExecutor          actionExecutor;
    private PathPlanning            pathPlanning;
    private POMCPPlanner            pomcpPlanner;

    private static final boolean USE_POMCP = Boolean.parseBoolean(
            System.getProperty("bayesian.use_pomcp", "true"));
    private static final boolean USE_COMM  = Boolean.parseBoolean(
            System.getProperty("bayesian.use_comm",  "true"));

    private final CommunicationManager commManager        = new CommunicationManager();
    private final Set<EntityID>        broadcastedVictims = new HashSet<>();
    private AgentMacroState prevMacroState = AgentMacroState.EXPLORE;

    // Метрики для сравнения эвристики vs POMCP
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
        pomcpPlanner         = new POMCPPlanner(ai, wi, pathPlanning);

        Logger.info(ai, "initialized mode=" + (USE_POMCP ? "POMCP" : "heuristic"));
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
        Logger.debug(ai, "tick " + ai.getTime());

        ChangeSet cs = ai.getChanged();
        observationProcessor.process(cs);
        beliefManager.update(observationProcessor.getObservation());

        Belief belief = beliefManager.getBelief();

        // Принять подтверждения расчистки от полиции
        if (USE_COMM) {
            for (EntityID road : commManager.receiveClearedRoads(msg)) {
                belief.roadBlockedProb.put(road, 0.0);
                belief.blockedRoads.remove(road);
                belief.clearedRoads.add(road);
                Logger.info(ai, "[COMM] road cleared confirmed: " + road);
            }

            // Принять данные о жертвах от других агентов
            for (MessageCivilian mc : commManager.receiveVictimInfo(msg)) {
                if (!mc.isBuriednessDefined()) continue;
                EntityID victimId = mc.getAgentID();
                int buried = mc.getBuriedness();
                Logger.info(ai, "[COMM] received VictimInfo victim=" + victimId
                    + " buried=" + buried + " pos=" + mc.getPosition());
                Belief.VictimBelief vb = belief.victims.get(victimId);
                if (buried == 0) {
                    // Пожарный освободил жертву: снять likelyBuried и убрать из skipVictims
                    if (vb != null) vb.likelyBuried = false;
                    if (USE_POMCP) pomcpPlanner.getStateMachine().notifyVictimFreed(victimId);
                } else if (vb != null) {
                    vb.likelyBuried = true;
                }
            }

            // Отправить: один MessageCivilian за тик (голосовой канал = max 1 сообщение)
            // Выбираем новую засыпанную жертву с наибольшим уроном (наиболее критичную)
            Civilian bestToSend = null;
            for (EntityID id : cs.getChangedEntities()) {
                StandardEntity e = wi.getEntity(id);
                if (!(e instanceof Civilian)) continue;
                Civilian civ = (Civilian) e;
                if (!civ.isBuriednessDefined() || civ.getBuriedness() <= 0) continue;
                if (!civ.isHPDefined() || civ.getHP() <= 0) continue;
                if (broadcastedVictims.contains(id)) continue;
                if (bestToSend == null
                        || (civ.isDamageDefined() && bestToSend.isDamageDefined()
                            && civ.getDamage() > bestToSend.getDamage())) {
                    bestToSend = civ;
                }
            }
            if (bestToSend != null) {
                commManager.sendVictimFound(msg, bestToSend);
                broadcastedVictims.add(bestToSend.getID());
                Logger.info(ai, "[COMM] sent VictimFound victim=" + bestToSend.getID()
                    + " buried=" + bestToSend.getBuriedness());
            }
        }

        // Per-victim entropy - только в debug-режиме
        if (Logger.DEBUG) {
            for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
                Belief.VictimBelief vb = e.getValue();
                double h = 0;
                for (double p : new double[]{vb.pHealthy, vb.pInjured, vb.pCritical, vb.pDead})
                    if (p > 1e-9) h -= p * Math.log(p);
                Logger.debug(ai, "victim=" + e.getKey() + " " + vb
                    + " H=" + String.format(Locale.US, "%.3f", h));
            }
            Logger.debug(ai, "obs=" + observationProcessor.getObservation()
                + " belief=" + belief);
        }

        // Компактный снимок belief каждые 25 тиков
        if (ai.getTime() % 25 == 0) {
            logBeliefSnap(ai, belief);
        }

        // Выбор действия
        AgentAction selectedAction;
        if (USE_POMCP) {
            selectedAction = pomcpPlanner.selectAction(
                belief, observationProcessor.getObservation());
        } else {
            policySelector.select(belief);
            selectedAction = policySelector.getSelectedAction();
            Logger.info(ai, "[STATE] t=" + ai.getTime()
                + " macro=" + policySelector.getMacroState()
                + " action=" + selectedAction.type
                + " target=" + policySelector.getTargetVictimId());
        }

        // T5: TELL(policeOffice, blockedEdge) при переходе NAVIGATE → WAIT_FOR_POLICE
        AgentMacroState curState = USE_POMCP
            ? pomcpPlanner.getCurrentState()
            : policySelector.getMacroState();
        if (USE_COMM
                && curState == AgentMacroState.WAIT_FOR_POLICE
                && prevMacroState != AgentMacroState.WAIT_FOR_POLICE) {
            EntityID blockedRoad = USE_POMCP
                ? pomcpPlanner.getLastBlockedRoad()
                : policySelector.getLastBlockedRoad();
            if (blockedRoad != null) {
                Road roadEntity = (Road) wi.getEntity(blockedRoad);
                if (roadEntity != null) {
                    commManager.sendRoadBlocked(msg, roadEntity, null);
                    Logger.info(ai, "[COMM] sent RoadBlocked road=" + blockedRoad
                        + " target=" + (USE_POMCP ? pomcpPlanner.getTargetVictimId() : policySelector.getTargetVictimId()));
                }
            }
        }
        prevMacroState = curState;

        // Метрики
        if (selectedAction.type == AgentAction.Type.LOAD) {
            loadTick = ai.getTime();
        }
        if (selectedAction.type == AgentAction.Type.UNLOAD && ai.someoneOnBoard() != null) {
            // Удаляем жертву из belief сразу - не ждём обновления worldInfo
            EntityID deliveredId = ai.someoneOnBoard().getID();
            beliefManager.removeDeliveredVictim(deliveredId);
            if (USE_POMCP) pomcpPlanner.getStateMachine().notifyDelivered(deliveredId);
            if (loadTick >= 0) {
                victimsDelivered++;
                int tripTicks = ai.getTime() - loadTick;
                totalDeliveryTicks += tripTicks;
                deliveryCount++;
                Logger.info(ai, "[DELIVERY] t=" + ai.getTime()
                    + " victim_delivered tripTicks=" + tripTicks
                    + " total=" + victimsDelivered);
                loadTick = -1;
            }
        }

        int t = ai.getTime();
        int maxT = -1;
        try { maxT = si.getKernelTimesteps(); } catch (Exception ignored) {}
        if (t % 50 == 0 || (maxT > 0 && t == maxT)) {
            logMetrics(ai, si, belief, t);
        }

        Logger.debug(ai, "action=" + selectedAction);
        return actionExecutor.translate(selectedAction);
    }

    private void logBeliefSnap(AgentInfo ai, Belief belief) {
        int n = belief.victims.size();
        double sumPCrit = 0, sumH = 0, topScore = 0;
        EntityID topId = null;
        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            sumPCrit += vb.pCritical;
            for (double p : new double[]{vb.pHealthy, vb.pInjured, vb.pCritical, vb.pDead})
                if (p > 1e-9) sumH -= p * Math.log(p);
            double sc = (vb.pCritical * 2.0 + vb.pInjured) * vb.pAlive();
            if (sc > topScore) { topScore = sc; topId = e.getKey(); }
        }
        Logger.info(ai, "[BELIEF_SNAP] t=" + ai.getTime()
            + " victims=" + n
            + " avg_pCrit=" + String.format(Locale.US, "%.3f", n > 0 ? sumPCrit / n : 0)
            + " avg_H=" + String.format(Locale.US, "%.3f", n > 0 ? sumH / n : 0)
            + " blocked=" + belief.blockedRoads.size()
            + " burning=" + belief.burningBuildings.size()
            + " top=" + topId
            + " topScore=" + String.format(Locale.US, "%.3f", topScore));
    }

    private void logMetrics(AgentInfo ai, ScenarioInfo si, Belief belief, int t) {
        victimsLost = Math.max(0, beliefManager.getVictimsRemovedAsDead() - victimsDelivered);
        double avg = deliveryCount > 0 ? (double) totalDeliveryTicks / deliveryCount : 0;
        int maxT2 = -1;
        try { maxT2 = si.getKernelTimesteps(); } catch (Exception ignored) {}
        boolean isFinal = maxT2 > 0 && t == maxT2;
        Logger.info(ai, "[METRICS" + (isFinal ? "_FINAL" : "") + "] t=" + t
            + " delivered=" + victimsDelivered
            + " lost=" + victimsLost
            + " inBelief=" + belief.victims.size()
            + " avgTripTicks=" + String.format(Locale.US, "%.1f", avg)
            + " mode=" + (USE_POMCP ? (USE_COMM ? "POMCP+COMM" : "POMCP") : "heuristic"));
    }
}
