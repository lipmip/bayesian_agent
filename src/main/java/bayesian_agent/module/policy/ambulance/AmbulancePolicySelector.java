package bayesian_agent.module.policy.ambulance;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

/**
 * Выбор действия для агента-санитара: b_t → a_t.
 * [ЗАМЕНИТЬ: POMCP]
 */
public class AmbulancePolicySelector {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;
    private final PathPlanning pathPlanning;
    private AgentAction selectedAction = AgentAction.rest();

    public AmbulancePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo,
                                   PathPlanning pathPlanning) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.pathPlanning = pathPlanning;
    }

    public void select(Belief belief) {
        // Везём жертву - едем в убежище или выгружаем
        if (agentInfo.someoneOnBoard() != null) {
            selectedAction = buildUnloadAction(belief);
            return;
        }

        // Есть известные живые жертвы
        EntityID best = pickBestVictim(belief);
        if (best != null) {
            selectedAction = buildRescueOrMoveAction(best, belief);
            return;
        }

        // Нечего делать
        // TODO (Этап 2): Search - обход непосещённых зданий
        selectedAction = buildSearchMove();
    }

    private EntityID pickBestVictim(Belief belief) {
        EntityID critical = null, injured = null;

        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            if (vb.pAlive() < 0.01) continue;
            if (vb.pCritical > 0.5 && critical == null) critical = e.getKey();
            else if (vb.pInjured > 0.5 && injured == null) injured = e.getKey();
        }

        return critical != null ? critical : injured;
    }

    private AgentAction buildRescueOrMoveAction(EntityID victimId, Belief belief) {
        EntityID agentPos  = agentInfo.getPosition();
        EntityID victimPos = getEntityPosition(victimId);

        if (agentPos != null && agentPos.equals(victimPos)) {
            Belief.VictimBelief vb = belief.victims.get(victimId);
            if (vb != null && vb.likelyBuried) return AgentAction.rescue(victimId);
            return AgentAction.load(victimId);
        }

        // Реальный маршрут через Dijkstra
        if (victimPos != null && agentPos != null) {
            List<EntityID> path = pathPlanning
                .setFrom(agentPos)
                .setDestination(victimPos)
                .calc()
                .getResult();
            if (path != null && !path.isEmpty()) return AgentAction.move(path);
        }

        return AgentAction.rest();
    }

    /**
     * Строит действие для доставки жертвы в убежище.
     * Учитывает ёмкость убежища - не едет в полное.
     */
    private AgentAction buildUnloadAction(Belief belief) {
        EntityID pos = agentInfo.getPosition();

        if (pos != null && belief.knownRefuges.containsKey(pos)
                && belief.knownRefuges.get(pos) != Belief.RefugeState.FULL) {
            return AgentAction.unload();
        }

        for (Map.Entry<EntityID, Belief.RefugeState> e
                : belief.knownRefuges.entrySet()) {
            if (e.getValue() != Belief.RefugeState.FULL) {
                EntityID refuge = e.getKey();
                List<EntityID> path = pathPlanning
                    .setFrom(pos)
                    .setDestination(refuge)
                    .calc()
                    .getResult();
                if (path != null && !path.isEmpty()) return AgentAction.move(path);
            }
        }
        return AgentAction.rest();
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

    //Возвращает EntityID позиции сущности
    private EntityID getEntityPosition(EntityID entityId) {
        StandardEntity entity = worldInfo.getEntity(entityId);

        if (entity instanceof Human) {
            return ((Human) entity).getPosition();
        }

        return null;
    }

    public AgentAction getSelectedAction() { return selectedAction; }
}