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
        // Шаг 1: PREDICT - применяем decay к невидимым жертвам 
        for (Map.Entry<EntityID, Belief.VictimBelief> e
                : currentBelief.victims.entrySet()) {
            if (!obs.victims.containsKey(e.getKey())) {
                Belief.VictimBelief vb = e.getValue();
                vb.ticksSinceObserved++;
                applyDamageDecay(vb);
            }
        }

        // Шаг 2: UPDATE - обновляем из наблюдений 
        for (Map.Entry<EntityID, Observation.VictimStatus> e
                : obs.victims.entrySet()) {
            Belief.VictimBelief vb = Belief.VictimBelief.fromObservation(e.getValue());
            // Сохраняем damage если доступен
            if (obs.victimDamage.containsKey(e.getKey())) {
                vb.lastKnownDamage = obs.victimDamage.get(e.getKey());
            }
            vb.ticksSinceObserved = 0;
            currentBelief.victims.put(e.getKey(), vb);
        }

        // Buriedness 
        for (Map.Entry<EntityID, Observation.BuriedStatus> e
                : obs.buriedStatus.entrySet()) {
            Belief.VictimBelief vb = currentBelief.victims.get(e.getKey());
            if (vb != null) {
                vb.likelyBuried = (e.getValue() == Observation.BuriedStatus.BURIED);
            }
        }

        // Убежища 
        for (Map.Entry<EntityID, Observation.RefugeCapacity> e
                : obs.refugeCapacity.entrySet()) {
            Belief.RefugeState state = switch (e.getValue()) {
                case AVAILABLE -> Belief.RefugeState.AVAILABLE;
                case FULL      -> Belief.RefugeState.FULL;
                default        -> Belief.RefugeState.UNKNOWN;
            };
            currentBelief.knownRefuges.put(e.getKey(), state);
        }

        // Пожары
        currentBelief.burningBuildings.addAll(obs.burningBuildings);
        currentBelief.burningBuildings.removeAll(obs.extinguishedBuildings);

        // Дороги 
        currentBelief.blockedRoads.addAll(obs.blockedRoads);
        currentBelief.blockedRoads.removeAll(obs.clearedRoads);
        currentBelief.knownBlockadeIds.addAll(obs.blockadeIds);
    }

    /**
     * Predict-шаг: HP жертвы убывает на lastKnownDamage каждый тик.
     * Сдвигаем вероятности вверх по шкале тяжести.
     */
    private void applyDamageDecay(Belief.VictimBelief vb) {
        double decayRate;
        if (vb.lastKnownDamage > 0) {
            decayRate = Math.min(vb.lastKnownDamage / 10000.0, 0.25);
        } else if (vb.likelyBuried) {
            decayRate = 0.02;
        } else {
            return;
        }

        // HEALTHY → INJURED → CRITICAL → DEAD
        double healthyToInjured  = vb.pHealthy  * decayRate;
        double injuredToCritical = vb.pInjured  * decayRate;
        double criticalToDead    = vb.pCritical * decayRate;

        vb.pHealthy  -= healthyToInjured;
        vb.pInjured  += healthyToInjured - injuredToCritical;
        vb.pCritical += injuredToCritical - criticalToDead;
        vb.pDead     += criticalToDead;
    }

    public Belief getBelief() {
        return currentBelief;
    }
}