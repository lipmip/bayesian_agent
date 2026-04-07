package bayesian_agent.module.belief;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import bayesian_agent.module.observation.Observation;
import rescuecore2.worldmodel.EntityID;
import java.util.Map;

public class BeliefManager {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;

    private Belief currentBelief = new Belief();

    public BeliefManager(AgentInfo agentInfo, WorldInfo worldInfo) {
        this.agentInfo = agentInfo;
        this.worldInfo = worldInfo;
    }

    public void update(Observation obs) {
        // TODO: predict через T
        for (Map.Entry<EntityID, Observation.VictimStatus> e : obs.victims.entrySet()) {
            currentBelief.victims.put(e.getKey(),
                    Belief.VictimBelief.fromObservation(e.getValue()));
        }
        for (Map.Entry<EntityID, Observation.BuriedStatus> e : obs.buriedStatus.entrySet()) {
            Belief.VictimBelief vb = currentBelief.victims.get(e.getKey());
            if (vb != null) {
                vb.likelyBuried =
                    (e.getValue() == Observation.BuriedStatus.BURIED);
            }
        }
        for (Map.Entry<EntityID, Observation.RefugeCapacity> e : obs.refugeCapacity.entrySet()) {
            Belief.RefugeState state = switch (e.getValue()) {
                case AVAILABLE -> Belief.RefugeState.AVAILABLE;
                case FULL      -> Belief.RefugeState.FULL;
                default        -> Belief.RefugeState.UNKNOWN;
            };
            currentBelief.knownRefuges.put(e.getKey(), state);
        }
        currentBelief.burningBuildings.addAll(obs.burningBuildings);
        currentBelief.burningBuildings.removeAll(obs.extinguishedBuildings);
        currentBelief.blockedRoads.addAll(obs.blockedRoads);
        currentBelief.blockedRoads.removeAll(obs.clearedRoads);
        currentBelief.knownBlockadeIds.addAll(obs.blockadeIds);
    }

    public Belief getBelief() {
        return currentBelief;
    }
}