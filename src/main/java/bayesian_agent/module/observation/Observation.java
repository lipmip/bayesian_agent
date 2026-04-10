package bayesian_agent.module.observation;

import rescuecore2.worldmodel.EntityID;
import java.util.*;

/**
 * Дискретизированное наблюдение o_t за один тик.
 */
public class Observation {

    public EntityID agentPosition = null;

    // AmbulanceTeam
    public final Map<EntityID, VictimStatus> victims            = new LinkedHashMap<>();
    public final Map<EntityID, BuriedStatus> buriedStatus       = new LinkedHashMap<>();
    public       boolean                     isCarrying         = false;
    public final Set<EntityID> visibleRefuges                   = new HashSet<>();
    public final Map<EntityID, RefugeCapacity> refugeCapacity   = new LinkedHashMap<>();

    // FireBrigade
    public final Set<EntityID> burningBuildings                 = new HashSet<>();
    public final Map<EntityID, FireIntensity> fireIntensity     = new LinkedHashMap<>();
    public final Set<EntityID> extinguishedBuildings            = new HashSet<>();

    // PoliceForce
    public final Set<EntityID> blockedRoads = new HashSet<>();
    public final Set<EntityID> clearedRoads = new HashSet<>();

    /**
     * EntityID самих сущностей Blockade.
     * ActionClear принимает именно Blockade ID, не Road ID.
     */
    public final Set<EntityID> blockadeIds           = new HashSet<>();

    public enum VictimStatus {
        HEALTHY, INJURED, CRITICAL, DEAD;

        public static VictimStatus fromHP(int hp) {
            if (hp <= 0)   return DEAD;
            if (hp < 3000) return CRITICAL;
            if (hp < 7500) return INJURED;
            return HEALTHY;
        }
    }
    public final Map<EntityID, Integer> victimDamage = new LinkedHashMap<>();

    public enum BuriedStatus {
        FREE,
        BURIED;

        public static BuriedStatus from(int buriedness) {
            return buriedness > 0 ? BURIED : FREE;
        }
    }

    public enum FireIntensity {
        NONE, LOW, MEDIUM, HIGH, BURNT;

        public static FireIntensity from(int fieryness) {
            if (fieryness <= 0) return NONE;
            if (fieryness <= 2) return LOW;
            if (fieryness <= 5) return MEDIUM;
            if (fieryness <= 7) return HIGH;
            return BURNT;
        }
    }    

    public enum RefugeCapacity {
        AVAILABLE,
        FULL,
        UNKNOWN;
    }    

    @Override
    public String toString() {
        return "Observation{victims=" + victims.size()
               + ", burning=" + burningBuildings.size()
               + ", blockedRoads=" + blockedRoads.size()
               + ", blockades=" + blockadeIds.size()
               + ", carrying=" + isCarrying + "}";
    }
}