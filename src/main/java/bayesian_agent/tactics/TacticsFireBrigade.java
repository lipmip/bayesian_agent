package bayesian_agent.tactics;

import adf.core.agent.action.Action;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import bayesian_agent.action.ActionExecutor;
import bayesian_agent.module.belief.BeliefManager;
import bayesian_agent.module.observation.ObservationProcessor;
import bayesian_agent.module.policy.fire.FireBrigadePolicySelector;
import bayesian_agent.util.Logger;
import rescuecore2.worldmodel.ChangeSet;

public class TacticsFireBrigade extends adf.core.component.tactics.TacticsFireBrigade {

    private ObservationProcessor      observationProcessor;
    private BeliefManager             beliefManager;
    private FireBrigadePolicySelector policySelector;
    private ActionExecutor            actionExecutor;

    @Override
    public void initialize(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                           ModuleManager mm, MessageManager msg, DevelopData dd) {
        observationProcessor = new ObservationProcessor(ai, wi);
        beliefManager        = new BeliefManager(ai, wi);
        policySelector       = new FireBrigadePolicySelector(ai, wi);
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
        policySelector.select(beliefManager.getBelief());
        Action action = actionExecutor.translate(policySelector.getSelectedAction());
        Logger.info(ai, "action=" + policySelector.getSelectedAction());
        return action;
    }
}