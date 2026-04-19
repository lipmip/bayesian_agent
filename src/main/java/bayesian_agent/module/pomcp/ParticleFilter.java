package bayesian_agent.module.pomcp;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.observation.Observation;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.Human;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

// Фильтр частиц: b_t ≈ {(s^i, w^i)}, N=500
// Систематический ресэмплинг при ESS/N < 0.5
public class ParticleFilter {

    private static final int    N          = 500;
    private static final double ESS_THRESH = 0.5;

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
    // Использует реальные данные из полей Belief (все уже существуют):
    //   vb.estimatedDamageRate  → VDmg
    //   belief.roadBlockedProb  → PC (вероятностная выборка)
    //   belief.clearedRoads     → PC = Clear (детерминировано)
    //   belief.buildingFireIntensity → FI
    //   belief.knownRefuges     → RB
    public void initFromBelief(Belief belief) {
        particles.clear();
        EntityID pos      = agentInfo.getPosition();
        Human onBoard     = agentInfo.someoneOnBoard();
        EntityID carrying = onBoard != null ? onBoard.getID() : null;

        for (int i = 0; i < N; i++) {
            Map<EntityID, Integer> hp      = new HashMap<>();
            Map<EntityID, Integer> dmg     = new HashMap<>();
            Map<EntityID, Integer> buried  = new HashMap<>();
            Map<EntityID, Boolean> blocked = new HashMap<>();
            Map<EntityID, Integer> fire    = new HashMap<>();
            Map<EntityID, Boolean> refuge  = new HashMap<>();

            // VHP, VDmg, VB из VictimBelief
            for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
                Belief.VictimBelief vb = e.getValue();
                hp.put(e.getKey(),    sampleHP(vb));
                dmg.put(e.getKey(),   vb.estimatedDamageRate);
                buried.put(e.getKey(), vb.likelyBuried ? 1 : 0);
            }

            // PC из roadBlockedProb (вероятностная выборка)
            for (Map.Entry<EntityID, Double> e : belief.roadBlockedProb.entrySet()) {
                blocked.put(e.getKey(), rng.nextDouble() < e.getValue());
            }
            // Расчищенные дороги - точно Clear
            for (EntityID road : belief.clearedRoads) {
                blocked.put(road, false);
            }

            // FI из buildingFireIntensity
            fire.putAll(belief.buildingFireIntensity);

            // RB из knownRefuges
            for (Map.Entry<EntityID, Belief.RefugeState> e : belief.knownRefuges.entrySet())
                refuge.put(e.getKey(), e.getValue() == Belief.RefugeState.FULL);

            particles.add(new SimState(hp, dmg, buried, blocked, fire, refuge, pos, carrying));
        }
        Arrays.fill(weights, 1.0 / N);
    }

    // Predict + Update после выполненного действия 
    public void update(AgentAction lastAction, Observation obs) {
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

        if (ess() / N < ESS_THRESH) resample();
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
