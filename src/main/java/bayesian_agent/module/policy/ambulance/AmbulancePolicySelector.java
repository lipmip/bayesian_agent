package bayesian_agent.module.policy.ambulance;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import bayesian_agent.module.pomcp.AgentMacroState;
import bayesian_agent.module.pomcp.StateMachineController;
import rescuecore2.worldmodel.EntityID;

// Тонкая обёртка над StateMachineController
// FSM логика перенесена в StateMachineController для использования в POMCP
public class AmbulancePolicySelector {

    private final StateMachineController sm;
    private AgentAction selectedAction = AgentAction.rest();

    public AmbulancePolicySelector(AgentInfo ai, WorldInfo wi, PathPlanning pp) {
        this.sm = new StateMachineController(ai, wi, pp);
    }

    public void select(Belief belief) {
        selectedAction = sm.tick(belief);
    }

    public AgentAction     getSelectedAction()  { return selectedAction; }
    public AgentMacroState getMacroState()      { return sm.getCurrentState(); }
    public EntityID        getTargetVictimId()  { return sm.getTargetVictimId(); }
    public EntityID        getLastBlockedRoad() { return sm.getLastBlockedRoad(); }
}
