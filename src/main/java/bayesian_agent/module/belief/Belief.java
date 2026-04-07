package bayesian_agent.module.belief;

import bayesian_agent.module.observation.Observation;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

/**
 * Убеждение агента bₜ.
 */
public class Belief {

    // AmbulanceTeam
    public final Map<EntityID, VictimBelief> victims = new LinkedHashMap<>();
    public enum RefugeState { AVAILABLE, FULL, UNKNOWN }
    public final Map<EntityID, RefugeState> knownRefuges = new LinkedHashMap<>();

    // FireBrigade
    public final Set<EntityID> burningBuildings      = new HashSet<>();

    // PoliceForce
    public final Set<EntityID> blockedRoads          = new HashSet<>();
    public final Set<EntityID> knownBlockadeIds      = new HashSet<>();

    public static class VictimBelief {
        public double pHealthy  = 0.0;
        public double pInjured  = 0.0;
        public double pCritical = 0.0;
        public double pDead     = 0.0;
        public boolean likelyBuried = false;

        public static VictimBelief fromObservation(Observation.VictimStatus s) {
            VictimBelief b = new VictimBelief();
            switch (s) {
                case HEALTHY:  b.pHealthy  = 1.0; break;
                case INJURED:  b.pInjured  = 1.0; break;
                case CRITICAL: b.pCritical = 1.0; break;
                case DEAD:     b.pDead     = 1.0; break;
            }
            return b;
        }

        public double pAlive() { return pHealthy + pInjured + pCritical; }

        @Override
        public String toString() {
            return String.format("VB{alive=%.2f,crit=%.2f,buried=%b}",
                    pAlive(), pCritical, likelyBuried);
        }
    }

    @Override
    public String toString() {
        return "Belief{victims=" + victims.size()
               + ", burning=" + burningBuildings.size()
               + ", blockedRoads=" + blockedRoads.size()
               + ", blockades=" + knownBlockadeIds.size() + "}";
    }
}