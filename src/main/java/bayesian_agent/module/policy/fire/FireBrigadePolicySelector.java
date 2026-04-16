package bayesian_agent.module.policy.fire;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

/**
 * Выбор действия для агента-пожарного: b_t → a_t.
 * [ЗАМЕНИТЬ: POMCP]
 */
public class FireBrigadePolicySelector {

    private final AgentInfo    agentInfo;
    private final WorldInfo    worldInfo;
    private final PathPlanning pathPlanning;
    private AgentAction selectedAction = AgentAction.rest();

    public FireBrigadePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo,
                                   PathPlanning pathPlanning) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.pathPlanning = pathPlanning;
    }

    private EntityID findBurningInRange(Belief belief) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return null;

        StandardEntity posEntity = worldInfo.getEntity(pos);
        if (posEntity instanceof Area) {
            Area area = (Area) posEntity;
            for (EntityID neighbourId : area.getNeighbours()) {
                if (belief.burningBuildings.contains(neighbourId)) {
                    return neighbourId;
                }
            }
        }

        return null;
    }

    public void select(Belief belief) {
        // FireBrigade - единственный тип агента, который может выполнять AKRescue.
        // Приоритет: сначала спасаем погребённых жертв, потом тушим.

        // Opportunistic rescue: если рядом погребённая живая жертва - спасаем немедленно,
        // не дожидаясь пока injCrit поднимется. Не давать HEALTHY жертвам умереть под
        // завалом пока FireBrigade ждёт порога.
        EntityID nearbyBuried = findBuriedVictimAtCurrentLocation(belief);
        if (nearbyBuried != null) {
            selectedAction = AgentAction.rescue(nearbyBuried);
            return;
        }

        // Navigation: идти к наиболее критической погребённой жертве.
        // injCrit>0.3 используется только для выбора цели навигации - не блокирует
        // rescue когда уже рядом (см. opportunistic выше).
        EntityID buriedVictim = pickBestBuriedVictim(belief);
        if (buriedVictim != null) {
            AgentAction rescueAction = buildRescueOrMoveToVictim(buriedVictim);
            if (rescueAction != null) {
                selectedAction = rescueAction;
                return;
            }
        }

        EntityID nearby = findBurningInRange(belief);
        if (nearby != null) {
            selectedAction = AgentAction.extinguish(nearby);
            return;
        }

        EntityID known = pickBestBurningBuilding(belief);
        if (known != null) {
            selectedAction = buildMoveToFire(known);
            return;
        }

        selectedAction = buildSearchMove();
    }

    /**
     * Возвращает EntityID погребённой живой жертвы на текущей позиции FireBrigade.
     * Порог injCrit не применяется - если мы уже рядом, спасаем немедленно.
     */
    private EntityID findBuriedVictimAtCurrentLocation(Belief belief) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return null;

        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            if (!vb.likelyBuried || vb.pAlive() < 0.01) continue;

            StandardEntity ve = worldInfo.getEntity(e.getKey());
            if (!(ve instanceof Human)) continue;
            EntityID victimPos = ((Human) ve).getPosition();
            if (pos.equals(victimPos)) return e.getKey();
        }
        return null;
    }

    private EntityID pickBestBuriedVictim(Belief belief) {
        EntityID best = null;
        double bestScore = -1.0;

        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            // Пороги для навигации: b(VB=Buried)>0.5 AND injCrit>0.3
            // (rescue при нахождении рядом - см. findBuriedVictimAtCurrentLocation)
            if (!vb.likelyBuried) continue;
            if (vb.pAlive() < 0.01) continue;
            double injCrit = vb.pInjured + vb.pCritical;
            if (injCrit < 0.3) continue;

            double score = (vb.pCritical * 2.0 + vb.pInjured) * vb.pAlive();
            if (score > bestScore) {
                bestScore = score;
                best = e.getKey();
            }
        }

        return best;
    }

    private AgentAction buildRescueOrMoveToVictim(EntityID victimId) {
        EntityID agentPos = agentInfo.getPosition();
        if (agentPos == null) return null;

        StandardEntity ve = worldInfo.getEntity(victimId);
        EntityID victimPos = null;
        if (ve instanceof Human) {
            EntityID pos = ((Human) ve).getPosition();
            if (pos != null && worldInfo.getEntity(pos) instanceof Area) {
                victimPos = pos;
            }
        }
        if (victimPos == null) return null;

        if (agentPos.equals(victimPos)) {
            return AgentAction.rescue(victimId);
        }

        List<EntityID> path = pathPlanning
            .setFrom(agentPos)
            .setDestination(victimPos)
            .calc()
            .getResult();
        if (path != null && !path.isEmpty()) return AgentAction.move(path);

        return null;
    }

    // Ищет горящее здание рядом с позицией агента
    private AgentAction buildMoveToFire(EntityID target) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return AgentAction.rest();

        List<EntityID> path = pathPlanning
            .setFrom(pos)
            .setDestination(target)
            .calc()
            .getResult();
        if (path != null && !path.isEmpty()) return AgentAction.move(path);

        return AgentAction.rest();
    }

    private EntityID pickBestBurningBuilding(Belief belief) {
        return belief.burningBuildings.isEmpty() ? null
                : belief.burningBuildings.iterator().next();
    }

    private AgentAction buildSearchMove() {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return AgentAction.rest();

        rescuecore2.standard.entities.StandardEntity e = worldInfo.getEntity(pos);
        if (e instanceof rescuecore2.standard.entities.Area) {
            List<EntityID> neighbours = 
                ((rescuecore2.standard.entities.Area) e).getNeighbours();

            if (!neighbours.isEmpty()) {
                EntityID next = neighbours.get(
                    (int)(Math.random() * neighbours.size()));
                List<EntityID> path = pathPlanning
                    .setFrom(pos).setDestination(next).calc().getResult();
                    
                if (path != null && !path.isEmpty()) 
                    return AgentAction.move(path);
            }
        }
        
        return AgentAction.rest();
    }

    public AgentAction getSelectedAction() { return selectedAction; }
}