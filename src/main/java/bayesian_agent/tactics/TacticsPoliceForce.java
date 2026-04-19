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
import bayesian_agent.module.policy.AgentAction;
import bayesian_agent.module.policy.police.PoliceForcePolicySelector;
import bayesian_agent.util.Logger;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;

public class TacticsPoliceForce extends adf.core.component.tactics.TacticsPoliceForce {

    private ObservationProcessor      observationProcessor;
    private BeliefManager             beliefManager;
    private PoliceForcePolicySelector policySelector;
    private ActionExecutor            actionExecutor;
    private PathPlanning              pathPlanning;

    private EntityID prevPos = null;
    private int      samePosTicks = 0;

    @Override
    public void initialize(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                           ModuleManager mm, MessageManager msg, DevelopData dd) {
        observationProcessor = new ObservationProcessor(ai, wi);
        beliefManager        = new BeliefManager(ai, wi);
        pathPlanning         = mm.getModule("TacticsPoliceForce.PathPlanning",
                                            "adf.impl.module.algorithm.DijkstraPathPlanning");
        policySelector       = new PoliceForcePolicySelector(ai, wi, si, pathPlanning);
        actionExecutor       = new ActionExecutor(ai, wi, si);
        Logger.info(ai, "initialized");
    }

    @Override
    public void precompute(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                           ModuleManager mm, PrecomputeData pd, DevelopData dd) {}

    @Override
    public void resume(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                       ModuleManager mm, PrecomputeData pd, DevelopData dd) {}

    @Override
    public void preparate(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                          ModuleManager mm, DevelopData dd) {}

    @Override
    public Action think(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                        ModuleManager mm, MessageManager msg, DevelopData dd) {
        ChangeSet cs = ai.getChanged();
        observationProcessor.process(cs);
        beliefManager.update(observationProcessor.getObservation());

        Belief belief = beliefManager.getBelief();

        // Stuck-детектор на уровне тактики: агент не двигался
        EntityID curPos = ai.getPosition();
        if (curPos != null && curPos.equals(prevPos)) {
            samePosTicks++;
        } else {
            samePosTicks = 0;
        }
        prevPos = curPos;

        policySelector.select(belief);
        AgentAction chosen = policySelector.getSelectedAction();

        Logger.info(ai, "[PF] t=" + ai.getTime()
            + " pos=" + curPos
            + " xy=(" + (int)ai.getX() + "," + (int)ai.getY() + ")"
            + " frozen=" + samePosTicks
            + " blocked=" + belief.blockedRoads.size()
            + " cleared=" + belief.clearedRoads.size()
            + " action=" + chosen.type
            + (chosen.targetId != null ? " tgt=" + chosen.targetId : "")
            + (chosen.path != null ? " pathLen=" + chosen.path.size() : ""));

        return actionExecutor.translate(chosen);
    }
}
