package bayesian_agent.module.pomcp;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import rescuecore2.standard.entities.StandardEntity;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.observation.Observation;
import bayesian_agent.module.policy.AgentAction;
import bayesian_agent.util.Logger;
import rescuecore2.standard.entities.Human;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

// Фильтр частиц: b_t ≈ {(s^i, w^i)}, N=500
// Систематический ресэмплинг при ESS/N < 0.5
public class ParticleFilter {

    private static final int    N            = 500;
    private static final double ESS_THRESH   = 0.5;
    private static final double INJECT_THRESH = 0.3;  // inject when ESS/N < 0.3
    private static final double ALIVE_THRESH  = 0.3;  // inject when alive-particle fraction < 0.3
    private static final double INJECT_RATIO  = 0.1;  // replace 10% of particles

    private final WorldInfo              worldInfo;
    private final AgentInfo              agentInfo;
    private final DynamicBayesianNetwork dbn;
    private final Random                 rng;

    private List<SimState> particles = new ArrayList<>(N);
    private double[]       weights   = new double[N];

    public ParticleFilter(WorldInfo wi, AgentInfo ai) {
        this.worldInfo = wi;
        this.agentInfo = ai;
        this.dbn       = new DynamicBayesianNetwork();
        this.rng       = new Random();
        Arrays.fill(weights, 1.0 / N);
    }

    // Инициализировать из Belief
    public void initFromBelief(Belief belief) {
        particles.clear();
        EntityID pos      = agentInfo.getPosition();
        Human    onBoard  = agentInfo.someoneOnBoard();
        EntityID carrying = onBoard != null ? onBoard.getID() : null;
        for (int i = 0; i < N; i++)
            particles.add(createParticle(belief, pos, carrying));
        Arrays.fill(weights, 1.0 / N);
    }

    private SimState createParticle(Belief belief, EntityID pos, EntityID carrying) {
        Map<EntityID, Integer> hp      = new HashMap<>();
        Map<EntityID, Integer> dmg     = new HashMap<>();
        Map<EntityID, Integer> buried  = new HashMap<>();
        Map<EntityID, Boolean> blocked = new HashMap<>();
        Map<EntityID, Integer> fire    = new HashMap<>();
        Map<EntityID, Boolean> refuge  = new HashMap<>();
        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            // Жертвы с pAlive < 0.1 почти наверняка мертвы. Включение их в частицы даёт
            // фантомные штрафы -200 за смерть на каждом шаге симуляции и делает Q(LOAD/RESCUE)
            // отрицательным даже когда рядом есть живая цель.
            if (vb.pAlive() < 0.1) continue;
            hp.put(e.getKey(),     sampleHP(vb));
            dmg.put(e.getKey(),    vb.estimatedDamageRate);
            buried.put(e.getKey(), vb.likelyBuried ? 1 : 0);
        }
        for (Map.Entry<EntityID, Double> e : belief.roadBlockedProb.entrySet())
            blocked.put(e.getKey(), rng.nextDouble() < e.getValue());
        for (EntityID road : belief.clearedRoads)
            blocked.put(road, false);
        fire.putAll(belief.buildingFireIntensity);
        for (Map.Entry<EntityID, Belief.RefugeState> e : belief.knownRefuges.entrySet())
            refuge.put(e.getKey(), e.getValue() == Belief.RefugeState.FULL);
        SimState state = new SimState(hp, dmg, buried, blocked, fire, refuge, pos, carrying);
        for (EntityID victimId : hp.keySet()) {
            StandardEntity e = worldInfo.getEntity(victimId);
            if (e instanceof Human) {
                EntityID victimPos = ((Human) e).getPosition();
                if (victimPos != null) state.victimPosition.put(victimId, victimPos);
            }
        }
        return state;
    }

    // Predict + Update после выполненного действия
    public void update(AgentAction lastAction, Observation obs, Belief belief) {
        ObservationSummary os = new ObservationSummary(
            obs.victims.size(),
            obs.blockedRoads.size(),
            agentInfo.someoneOnBoard() != null
        );

        double total = 0.0;
        for (int i = 0; i < N; i++) {
            particles.set(i, dbn.transition(particles.get(i), lastAction));
            weights[i] *= dbn.observationLikelihood(particles.get(i), os);
            total += weights[i];
        }
        if (total > 1e-10)
            for (int i = 0; i < N; i++) weights[i] /= total;
        else
            Arrays.fill(weights, 1.0 / N);

        double essRatio = ess() / N;
        if (essRatio < ESS_THRESH) resample();

        // Инъекция частиц из belief против двух форм вырождения:
        //   1. Низкий ESS (< INJECT_THRESH): стандартная дивергенция весов
        //   2. Низкая доля живых частиц (< ALIVE_THRESH): жертвы умирают в DBN-переходах,
        //      наблюдение "нет жертв рядом" даёт им likelihood=1.0 → фильтр сходится к мёртвым
        //      (проверяем только если belief считает кого-то живым - иначе alive=0 норма)
        long aliveParticles = 0;
        for (SimState p : particles)
            if (p.victimHP.values().stream().anyMatch(hp -> hp > 0)) aliveParticles++;

        boolean beliefHasAlive = belief.victims.values().stream().anyMatch(v -> v.pAlive() > 0.01);
        boolean aliveDegenerate = beliefHasAlive && (double) aliveParticles / N < ALIVE_THRESH;

        if (essRatio < INJECT_THRESH || aliveDegenerate) {
            int k = (int)(N * INJECT_RATIO);
            inject(belief, k);
            Logger.info(agentInfo, "[PF] inject ess=" + String.format("%.2f", essRatio)
                + " alive=" + aliveParticles + "/" + N + " k=" + k);
        }
    }

    private void inject(Belief belief, int count) {
        EntityID pos      = agentInfo.getPosition();
        Human    onBoard  = agentInfo.someoneOnBoard();
        EntityID carrying = onBoard != null ? onBoard.getID() : null;

        List<Integer> idx = new ArrayList<>(N);
        for (int i = 0; i < N; i++) idx.add(i);
        Collections.shuffle(idx, rng);

        for (int k = 0; k < count; k++) {
            particles.set(idx.get(k), createParticle(belief, pos, carrying));
            weights[idx.get(k)] = 1.0 / N;
        }
        double sum = 0;
        for (double w : weights) sum += w;
        if (sum > 1e-10) for (int i = 0; i < N; i++) weights[i] /= sum;
    }

    public SimState sample() {
        double r = rng.nextDouble(), cum = 0.0;
        for (int i = 0; i < N; i++) {
            cum += weights[i];
            if (r <= cum) return particles.get(i).deepCopy();
        }
        return particles.get(N - 1).deepCopy();
    }

    private int sampleHP(Belief.VictimBelief vb) {
        double r = rng.nextDouble();
        if (r < vb.pDead)     return 0;                     r -= vb.pDead;
        if (r < vb.pCritical) return rng.nextInt(2999) + 1; r -= vb.pCritical;
        if (r < vb.pInjured)  return rng.nextInt(4500) + 3000;
        return rng.nextInt(2500) + 7500;
    }

    private double ess() {
        double s = 0; for (double w : weights) s += w * w;
        return s > 1e-15 ? 1.0 / s : 0;
    }

    private void resample() {
        List<SimState> r = new ArrayList<>(N);
        double step = 1.0 / N, u = rng.nextDouble() * step, cum = 0; int j = 0;
        for (int i = 0; i < N; i++) {
            double tgt = u + i * step;
            while (cum + weights[j] < tgt && j < N - 1) cum += weights[j++];
            r.add(particles.get(j).deepCopy());
        }
        particles = r;
        Arrays.fill(weights, 1.0 / N);
    }
}
