package bayesian_agent.module.pomcp;

import rescuecore2.worldmodel.EntityID;
import java.util.*;

// Скрытое состояние мира s ∈ S. Одна частица в ParticleFilter
// 7 переменных:
//   VHP  - Victim_HP:         {Healthy[7500-10000], Injured[3000-7499], Critical[1-2999], Dead[0]}
//   VDmg - Victim_Damage:     скорость убывания HP за тик (≈ estimatedDamageRate из Belief)
//   VB   - Victim_Buriedness: {Free[0], Buried[1+]} - LOAD возможен только при buriedness=0
//   PC   - Path_Clear:        {Clear=false, Blocked=true}
//   FI   - Fire_Intensity:    fieryness [0..8] (из buildingFireIntensity)
//   RB   - Refuge_Beds:       {Available=false, Full=true}
//   AG   - Agent_Carry:       null=Empty, EntityID=Carrying
public class SimState {

    public final Map<EntityID, Integer> victimHP;
    public final Map<EntityID, Integer> victimDamageRate;
    public final Map<EntityID, Integer> victimBuriedness;
    public final Map<EntityID, Boolean> roadBlocked;
    public final Map<EntityID, Integer> fireFieryness;
    public final Map<EntityID, Boolean> refugeFull;
    public EntityID carryingVictim;
    public EntityID agentPosition;

    public SimState(Map<EntityID, Integer> victimHP,
                    Map<EntityID, Integer> victimDamageRate,
                    Map<EntityID, Integer> victimBuriedness,
                    Map<EntityID, Boolean> roadBlocked,
                    Map<EntityID, Integer> fireFieryness,
                    Map<EntityID, Boolean> refugeFull,
                    EntityID agentPosition, EntityID carryingVictim) {
        this.victimHP         = new HashMap<>(victimHP);
        this.victimDamageRate = new HashMap<>(victimDamageRate);
        this.victimBuriedness = new HashMap<>(victimBuriedness);
        this.roadBlocked      = new HashMap<>(roadBlocked);
        this.fireFieryness    = new HashMap<>(fireFieryness);
        this.refugeFull       = new HashMap<>(refugeFull);
        this.agentPosition    = agentPosition;
        this.carryingVictim   = carryingVictim;
    }

    public SimState deepCopy() {
        return new SimState(victimHP, victimDamageRate, victimBuriedness,
                            roadBlocked, fireFieryness, refugeFull,
                            agentPosition, carryingVictim);
    }

    public enum HPCategory { HEALTHY, INJURED, CRITICAL, DEAD }

    public HPCategory getHPCategory(EntityID v) {
        int hp = victimHP.getOrDefault(v, 0);
        if (hp == 0)    return HPCategory.DEAD;
        if (hp <= 2999) return HPCategory.CRITICAL;
        if (hp <= 7499) return HPCategory.INJURED;
        return HPCategory.HEALTHY;
    }

    public boolean isAlive(EntityID v)  { return victimHP.getOrDefault(v, 0) > 0; }
    public boolean isBuried(EntityID v) { return victimBuriedness.getOrDefault(v, 0) > 0; }
}
