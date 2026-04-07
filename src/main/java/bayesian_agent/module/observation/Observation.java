package bayesian_agent.module.observation;

import rescuecore2.worldmodel.EntityID;
import java.util.*;

/**
 * Дискретизированное наблюдение oₜ за один тик.
 */
public class Observation {

    public EntityID agentPosition = null;

    // AmbulanceTeam
    public final Map<EntityID, VictimStatus> victims = new LinkedHashMap<>();
    public boolean isCarrying                        = false;
    public final Set<EntityID> visibleRefuges        = new HashSet<>();

    // FireBrigade
    public final Set<EntityID> burningBuildings      = new HashSet<>();

    // PoliceForce
    /** EntityID дорог с завалами (для навигации). */
    public final Set<EntityID> blockedRoads          = new HashSet<>();

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

    @Override
    public String toString() {
        return "Observation{victims=" + victims.size()
               + ", burning=" + burningBuildings.size()
               + ", blockedRoads=" + blockedRoads.size()
               + ", blockades=" + blockadeIds.size()
               + ", carrying=" + isCarrying + "}";
    }
}