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
import bayesian_agent.module.policy.police.PoliceForcePolicySelector;
import bayesian_agent.util.Logger;
import rescuecore2.worldmodel.ChangeSet;

public class TacticsPoliceForce extends adf.core.component.tactics.TacticsPoliceForce {

    private ObservationProcessor      observationProcessor;
    private BeliefManager             beliefManager;
    private PoliceForcePolicySelector policySelector;
    private ActionExecutor            actionExecutor;
    private PathPlanning              pathPlanning;

    @Override
    public void initialize(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                           ModuleManager mm, MessageManager msg, DevelopData dd) {
        observationProcessor = new ObservationProcessor(ai, wi);
        beliefManager        = new BeliefManager(ai, wi);
        pathPlanning         = mm.getModule("TacticsPoliceForce.PathPlanning",
                                            "adf.impl.module.algorithm.DijkstraPathPlanning");
        policySelector       = new PoliceForcePolicySelector(ai, wi, pathPlanning);
        actionExecutor       = new ActionExecutor(ai, wi, si);

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

        if (ai.getTime() % 50 == 0) {
            Logger.info(ai, "[POLICE_METRICS] t=" + ai.getTime()
                + " blocked=" + belief.blockedRoads.size()
                + " cleared=" + belief.clearedRoads.size()
                + " blockades=" + belief.knownBlockadeIds.size());
        }

        policySelector.select(belief);
        Action action = actionExecutor.translate(policySelector.getSelectedAction());

        Logger.info(ai, "obs=" + observationProcessor.getObservation() + " belief=" + belief);
        Logger.info(ai, "action=" + policySelector.getSelectedAction());

        return action;
    }
}