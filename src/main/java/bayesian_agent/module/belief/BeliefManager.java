package bayesian_agent.module.belief;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import bayesian_agent.module.observation.Observation;
import bayesian_agent.util.Logger;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class BeliefManager {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;

    private Belief currentBelief = new Belief();

    // Счётчик жертв, удалённых из belief как мёртвые (pDead > 0.99)
    private int victimsRemovedAsDead = 0;

    // ID жертв, уже доставленных в убежище - никогда не добавляем обратно в belief
    private final java.util.Set<EntityID> deliveredVictims = new java.util.HashSet<>();

    public BeliefManager(AgentInfo agentInfo, WorldInfo worldInfo) {
        this.agentInfo = agentInfo;
        this.worldInfo = worldInfo;
    }

    public void update(Observation obs) {
        // PREDICT - применяем decay к невидимым жертвам
        for (Map.Entry<EntityID, Belief.VictimBelief> e
                : currentBelief.victims.entrySet()) {
            if (!obs.victims.containsKey(e.getKey())) {
                Belief.VictimBelief vb = e.getValue();
                vb.ticksSinceObserved++;
                // Жертвы в убежище уже в безопасности - decay не применяем
                if (isAtRefuge(e.getKey())) continue;

                double pDeadBefore = vb.pDead;
                applyDamageDecay(vb);
                if (vb.pDead - pDeadBefore > 0.05) {
                    Logger.debug(agentInfo, "[DECAY] id=" + e.getKey()
                        + " pDead " + String.format(Locale.US, "%.3f→%.3f", pDeadBefore, vb.pDead)
                        + " dmg=" + vb.lastKnownDamage);
                }
            }
        }

        // Удаляем фактически мёртвых жертв и логируем события
        // Также удаляем жертв, которых worldInfo видит в убежище - они доставлены
        List<EntityID> toRemove = new ArrayList<>();
        for (Map.Entry<EntityID, Belief.VictimBelief> e : currentBelief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            if (vb.pDead > 0.99 && vb.ticksSinceObserved > 5) {
                toRemove.add(e.getKey());
            } else if (isAtRefuge(e.getKey())) {
                Logger.info(agentInfo, "[BELIEF] -delivered id=" + e.getKey()
                    + " (victim now at refuge)");
                toRemove.add(e.getKey());
            }
        }

        for (EntityID id : toRemove) {
            Belief.VictimBelief vb = currentBelief.victims.get(id);
            if (vb != null && vb.pDead > 0.99) {
                Logger.info(agentInfo, "[BELIEF] -dead id=" + id
                    + " ticks=" + vb.ticksSinceObserved
                    + " pDead=" + String.format(Locale.US, "%.3f", vb.pDead));
                victimsRemovedAsDead++;
            }
            currentBelief.victims.remove(id);
        }

        // PREDICT - вероятность заблокированности убывает если дорогу давно не наблюдали
        for (EntityID road : new HashSet<>(currentBelief.roadBlockedProb.keySet())) {
            if (!obs.blockedRoads.contains(road)) {
                double prob = currentBelief.roadBlockedProb.get(road);
                double decayed = prob * 0.98;

                if (decayed < 0.1) {
                    currentBelief.roadBlockedProb.remove(road);
                    currentBelief.blockedRoads.remove(road);
                } else {
                    currentBelief.roadBlockedProb.put(road, decayed);
                }
            }
        }

        // UPDATE - обновляем из наблюдений
        for (Map.Entry<EntityID, Observation.VictimStatus> e
                : obs.victims.entrySet()) {
            // Доставленная или находящаяся в убежище жертва - не добавляем обратно
            if (deliveredVictims.contains(e.getKey())) continue;
            if (isAtRefuge(e.getKey())) continue;

            boolean isNew = !currentBelief.victims.containsKey(e.getKey());
            Belief.VictimBelief vb = Belief.VictimBelief.fromObservation(e.getValue());
            // Сохраняем damage и estimatedDamageRate если доступен
            if (obs.victimDamage.containsKey(e.getKey())) {
                int dmg = obs.victimDamage.get(e.getKey());
                vb.lastKnownDamage = dmg;
                // VDmg категории: Low<100, Medium<200, High>=200
                vb.estimatedDamageRate = dmg > 0 ? dmg : (vb.likelyBuried ? 100 : 30);
            }
            vb.ticksSinceObserved = 0;
            currentBelief.victims.put(e.getKey(), vb);
            
            if (isNew) {
                Logger.info(agentInfo, "[BELIEF] +victim id=" + e.getKey()
                    + " status=" + e.getValue());
            }
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
        // FI: интенсивность пожара
        currentBelief.buildingFireIntensity.putAll(obs.buildingFireIntensityObs);

        // Дороги - обновляем blockedRoads и roadBlockedProb
        for (EntityID road : obs.blockedRoads) {
            currentBelief.roadBlockedProb.put(road, 1.0);
            currentBelief.blockedRoads.add(road);
        }
        for (EntityID road : obs.clearedRoads) {
            currentBelief.roadBlockedProb.remove(road);
            currentBelief.blockedRoads.remove(road);
            currentBelief.clearedRoads.add(road);
        }
        currentBelief.knownBlockadeIds.addAll(obs.blockadeIds);
    }

    // Predict-шаг: HP жертвы убывает на lastKnownDamage каждый тик
    // Сдвигаем вероятности вверх по шкале тяжести
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

    // Число жертв, удалённых из belief по критерию pDead > 0.99
    public int getVictimsRemovedAsDead() {
        return victimsRemovedAsDead;
    }

    //  Явно удалить жертву из belief после успешной доставки в убежище (UNLOAD)
    //  Добавляет в deliveredVictims - жертва никогда не будет добавлена обратно
    public void removeDeliveredVictim(EntityID id) {
        deliveredVictims.add(id);
        if (currentBelief.victims.remove(id) != null) {
            Logger.info(agentInfo, "[BELIEF] -delivered id=" + id + " (explicit after UNLOAD)");
        }
    }

    // Возвращает true если жертва находится в убежище (доставлена, в безопасности)
    private boolean isAtRefuge(EntityID victimId) {
        StandardEntity entity = worldInfo.getEntity(victimId);
        if (!(entity instanceof Human)) return false;
        EntityID pos = ((Human) entity).getPosition();
        if (pos == null) return false;
        return worldInfo.getEntity(pos) instanceof Refuge;
    }
}