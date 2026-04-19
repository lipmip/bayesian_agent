package bayesian_agent.module.pomcp;

// Макросостояния агента; перенесён из AmbulancePolicySelector для использования в POMCP
public enum AgentMacroState {
    EXPLORE, NAVIGATE, WAIT_FOR_POLICE, RESCUE, TRANSPORT, DELIVER
}
