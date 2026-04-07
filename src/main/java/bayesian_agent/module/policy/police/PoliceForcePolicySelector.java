package bayesian_agent.module.policy.police;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

public class PoliceForcePolicySelector {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;
    private AgentAction selectedAction = AgentAction.rest();

    public PoliceForcePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo) {
        this.agentInfo = agentInfo;
        this.worldInfo = worldInfo;
    }

    public void select(Belief belief) {
        EntityID blockade = pickBestBlockade(belief);
        if (blockade != null) {
            selectedAction = AgentAction.clear(blockade);
            return;
        }
        EntityID road = pickBlockedRoad(belief);
        if (road != null) {
            selectedAction = AgentAction.move(Collections.singletonList(road));
            return;
        }
        selectedAction = AgentAction.rest();
    }

    private EntityID pickBestBlockade(Belief belief) {
        return belief.knownBlockadeIds.isEmpty() ? null
                : belief.knownBlockadeIds.iterator().next();
    }

    private EntityID pickBlockedRoad(Belief belief) {
        return belief.blockedRoads.isEmpty() ? null
                : belief.blockedRoads.iterator().next();
    }

    public AgentAction getSelectedAction() { return selectedAction; }
}