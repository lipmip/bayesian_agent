package bayesian_agent.module.pomcp;

import bayesian_agent.module.policy.AgentAction;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

// DBN: P(S_{t+1} | S_t, a_t) = ∏ P(S^j_{t+1} | Parents(S^j_{t+1}))
// CPT заданы численно на основе механики RCRS:
//   P(VB=Free | VB=Buried, a=RESCUE)   = 0.70
//   P(PC=Clear | PC=Blocked, a=CLEAR)  = 0.25
//   P(FI+1 | FI, no EXTINGUISH)        = 0.10
//   P(FI-1 | FI, EXTINGUISH)           = 0.30
//   P(VDmg+50 | VB=Buried)             = 0.05/тик
//   P(RB=Full | UNLOAD)                 = 0.30
public class DynamicBayesianNetwork {

    private final Random rng = new Random();

    // T(s, a) → s' 
    public SimState transition(SimState s, AgentAction action) {
        SimState next = s.deepCopy();

        // VHP: убывает на damageRate, модифицированный FI и buriedness
        for (EntityID v : new HashSet<>(next.victimHP.keySet())) {
            int hp  = next.victimHP.get(v);
            if (hp <= 0) continue;
            int dmg = next.victimDamageRate.getOrDefault(v, 50);
            int fi  = next.fireFieryness.getOrDefault(v, 0);
            if (fi >= 6)      dmg = (int)(dmg * 2.0);
            else if (fi >= 3) dmg = (int)(dmg * 1.5);
            if (!next.isBuried(v)) dmg = Math.max(10, dmg / 3);
            next.victimHP.put(v, Math.max(0, hp - dmg));
        }

        // VDmg: P(VDmg+50 | VB=Buried) = 0.05
        for (EntityID v : new HashSet<>(next.victimDamageRate.keySet())) {
            if (next.isBuried(v) && rng.nextDouble() < 0.05)
                next.victimDamageRate.merge(v, 50, (oldVal, inc) -> Math.min(300, oldVal + inc));
        }

        // VB: P(VB=Free | VB=Buried, a=RESCUE) = 0.70
        if (action.type == AgentAction.Type.RESCUE && action.targetId != null) {
            int b = next.victimBuriedness.getOrDefault(action.targetId, 0);
            if (b > 0 && rng.nextDouble() < 0.70)
                next.victimBuriedness.put(action.targetId, b - 1);
        }

        // PC: P(PC=Clear | PC=Blocked, a=CLEAR) = 0.25
        if (action.type == AgentAction.Type.CLEAR && action.targetId != null
                && rng.nextDouble() < 0.25)
            next.roadBlocked.put(action.targetId, false);

        // FI: P(FI+1) = 0.10, P(FI-1 | EXTINGUISH) = 0.30
        for (EntityID bld : new HashSet<>(next.fireFieryness.keySet())) {
            int fi = next.fireFieryness.get(bld);
            if (fi <= 0 || fi >= 8) continue;
            boolean ext = action.type == AgentAction.Type.EXTINGUISH
                    && bld.equals(action.targetId);
            if (ext && rng.nextDouble() < 0.30)
                next.fireFieryness.put(bld, fi - 1);
            else if (!ext && rng.nextDouble() < 0.10)
                next.fireFieryness.put(bld, fi + 1);
        }

        // RB: P(Full | UNLOAD) = 0.30
        if (action.type == AgentAction.Type.UNLOAD && next.agentPosition != null
                && rng.nextDouble() < 0.30)
            next.refugeFull.put(next.agentPosition, true);

        // AG: детерминировано
        if (action.type == AgentAction.Type.LOAD && action.targetId != null)
            next.carryingVictim = action.targetId;
        else if (action.type == AgentAction.Type.UNLOAD)
            next.carryingVictim = null;

        return next;
    }

    // Z(s', o) = P(o | s') 
    public double observationLikelihood(SimState s, ObservationSummary obs) {
        long alive   = s.victimHP.values().stream().filter(hp -> hp > 0).count();
        long blocked = s.roadBlocked.values().stream().filter(b -> b).count();
        double lk = Math.exp(-Math.abs((int)alive   - obs.visibleVictimCount)  * 0.5);
        lk        *= Math.exp(-Math.abs((int)blocked - obs.visibleBlockedRoads) * 0.3);
        return lk;
    }
}
