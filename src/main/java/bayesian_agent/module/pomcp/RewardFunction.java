package bayesian_agent.module.pomcp;

import bayesian_agent.module.policy.AgentAction;
import rescuecore2.worldmodel.EntityID;

// R: S × A → R_вещ (ориентация на xml-схему переходов):
//   +500  savedVictimReward    (T12: Transport→Deliver)
//   +50   loadReward           (T4, T10)
//   +30   rescueStartReward    (T3: Navigate→Rescue, первое освобождение)
//   +15   buriednessDelta      (T9: Rescue self-loop)
//   -5    waitPenalty          (T5, T7: WaitForPolice)
//   -200  lostVictimPenalty    (T6, T8, T11: жертва умерла)
//   -1    travelCost           (MOVE)
//   -8    criticalPenalty      (каждая critical жертва в тик)
public class RewardFunction {

    public double compute(SimState before, AgentAction action, SimState after) {
        double r = 0.0;

        switch (action.type) {
            case MOVE, MOVE_TO_POINT -> r -= 1.0;
            case REST                -> r -= 5.0;
            case LOAD -> {
                if (after.carryingVictim != null && before.carryingVictim == null)
                    r += 50.0;
            }
            case UNLOAD -> {
                if (before.carryingVictim != null && after.carryingVictim == null) {
                    EntityID v = before.carryingVictim;
                    r += after.isAlive(v) ? 500.0 : -200.0;
                }
            }
            case RESCUE -> {
                if (action.targetId != null) {
                    int bb = before.victimBuriedness.getOrDefault(action.targetId, 0);
                    int ba = after.victimBuriedness.getOrDefault(action.targetId, 0);
                    if (bb > 0 && ba == 0)  r += 30.0;
                    else if (bb > ba)       r += 15.0;
                }
            }
        }

        for (EntityID v : after.victimHP.keySet()) {
            if (after.getHPCategory(v) == SimState.HPCategory.CRITICAL) r -= 8.0;
            if (before.isAlive(v) && !after.isAlive(v))                 r -= 200.0;
        }

        return r;
    }
}
