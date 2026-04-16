package bayesian_agent.module.belief;

import bayesian_agent.module.observation.Observation;
import rescuecore2.worldmodel.EntityID;
import java.util.*;
import java.util.Locale;

/**
 * Убеждение агента b_t.
 */
public class Belief {

    // AmbulanceTeam
    public final Map<EntityID, VictimBelief> victims     = new LinkedHashMap<>();
    public enum RefugeState { AVAILABLE, FULL, UNKNOWN }
    public final Map<EntityID, RefugeState> knownRefuges = new LinkedHashMap<>();

    // FireBrigade
    public final Set<EntityID> burningBuildings      = new HashSet<>();

    // PoliceForce
    public final Set<EntityID> blockedRoads          = new HashSet<>();
    public final Set<EntityID> knownBlockadeIds      = new HashSet<>();

    // PC: вероятность заблокированности дороги (1.0 = точно заблокирована из наблюдения)
    public Map<EntityID, Double> roadBlockedProb = new HashMap<>();

    // FI: интенсивность пожара по зданиям [0..8] (0 = нет пожара)
    public Map<EntityID, Integer> buildingFireIntensity = new HashMap<>();

    // Расчищенные дороги - для инициализации ParticleFilter в POMCP
    public Set<EntityID> clearedRoads = new HashSet<>();

    public static class VictimBelief {
        public double  pHealthy           = 0.0;
        public double  pInjured           = 0.0;
        public double  pCritical          = 0.0;
        public double  pDead              = 0.0;
        public int     lastKnownDamage      = 0;
        public int     ticksSinceObserved  = 0;
        public boolean likelyBuried        = false;
        // VDmg: оценка скорости убывания HP за тик
        public int     estimatedDamageRate = 50; // default: Low

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
            return String.format(Locale.US,
                    "VB{alive=%.3f,inj=%.3f,crit=%.3f,dmg=%d,dmgRate=%d,ticks=%d,buried=%b}",
                    pAlive(), pInjured, pCritical, lastKnownDamage, estimatedDamageRate, ticksSinceObserved, likelyBuried);
        }
    }

    @Override
    public String toString() {
        return "Belief{victims=" + victims.size()
               + ", burning=" + burningBuildings.size()
               + ", blockedRoads=" + blockedRoads.size()
               + ", roadBlockedProb=" + roadBlockedProb.size()
               + ", fireIntensity=" + buildingFireIntensity.size()
               + ", clearedRoads=" + clearedRoads.size()
               + ", blockades=" + knownBlockadeIds.size() + "}";
    }
}