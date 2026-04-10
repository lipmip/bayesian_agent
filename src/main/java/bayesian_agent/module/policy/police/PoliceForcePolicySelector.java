package bayesian_agent.module.policy.police;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

public class PoliceForcePolicySelector {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;
    private final PathPlanning pathPlanning;
    private AgentAction selectedAction = AgentAction.rest();
    private EntityID lastBlockade = null;
    private int lastRepairCost = Integer.MAX_VALUE;
    private int stuckCount = 0;
    private static final int STUCK_THRESHOLD = 5;

    public PoliceForcePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo,
                                     PathPlanning pathPlanning) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.pathPlanning = pathPlanning;
    }

    public void select(Belief belief) {
        EntityID agentPos = agentInfo.getPosition();

        if (agentPos != null && belief.blockedRoads.contains(agentPos)) {
            EntityID blockade = pickBlockadeOnRoad(agentPos);
            if (blockade != null) {
                rescuecore2.standard.entities.StandardEntity b =
                    worldInfo.getEntity(blockade);
                if (b instanceof rescuecore2.standard.entities.Blockade) {
                    rescuecore2.standard.entities.Blockade bl =
                        (rescuecore2.standard.entities.Blockade) b;
                    // Проверяем зависание - тот же завал и стоимость не падает
                    int currentCost = bl.getRepairCost();

                    if (blockade.equals(lastBlockade) && currentCost >= lastRepairCost) {
                        stuckCount++;
                    } else {
                        stuckCount = 0;
                    }
                    lastBlockade = blockade;
                    lastRepairCost = currentCost;

                    // Застряли - переходим к другой заблокированной дороге
                    if (stuckCount >= STUCK_THRESHOLD) {
                        stuckCount = 0;
                        lastBlockade = null;
                        lastRepairCost = Integer.MAX_VALUE;

                        // Ищем другую дорогу кроме текущей
                        EntityID nextRoad = pickOtherBlockedRoad(belief, agentPos);
                        if (nextRoad != null) {
                            selectedAction = buildMoveToBlockade(nextRoad);
                            return;
                        }
                        selectedAction = AgentAction.rest();
                        return;
                    }

                    double dist = Math.hypot(
                        bl.getX() - agentInfo.getX(),
                        bl.getY() - agentInfo.getY());
                    if (dist < 7500) {
                        selectedAction = AgentAction.clear(blockade);
                        return;
                    } else {
                        selectedAction = AgentAction.moveToPoint(bl.getX(), bl.getY());
                        return;
                    }
                }
            }
        }

        // Едем к ближайшей заблокированной дороге
        EntityID road = pickBlockedRoad(belief);
        if (road != null) {
            selectedAction = buildMoveToBlockade(road);
            return;
        }

        selectedAction = AgentAction.rest();
    }

    private EntityID pickOtherBlockedRoad(Belief belief, EntityID currentRoad) {
        for (EntityID road : belief.blockedRoads) {
            if (!road.equals(currentRoad)) return road;
        }
        
        return null;
    }

    private EntityID pickBlockadeOnRoad(EntityID roadId) {
        rescuecore2.standard.entities.StandardEntity entity = worldInfo.getEntity(roadId);
        if (entity instanceof rescuecore2.standard.entities.Road) {
            rescuecore2.standard.entities.Road road =
                (rescuecore2.standard.entities.Road) entity;
            if (road.isBlockadesDefined() && !road.getBlockades().isEmpty()) {
                EntityID bid = road.getBlockades().iterator().next();

                // Диагностика
                rescuecore2.standard.entities.StandardEntity b = worldInfo.getEntity(bid);
                if (b instanceof rescuecore2.standard.entities.Blockade) {
                    rescuecore2.standard.entities.Blockade bl =
                        (rescuecore2.standard.entities.Blockade) b;

                    System.err.println("[DIAG] blockade x=" + bl.getX()
                        + " y=" + bl.getY()
                        + " repairCost=" + bl.getRepairCost());
                }

                return bid;
            }
        }
        return null;
    }

    private EntityID pickBestBlockade(Belief belief) {
        return belief.knownBlockadeIds.isEmpty() ? null
                : belief.knownBlockadeIds.iterator().next();
    }

    private EntityID pickBlockedRoad(Belief belief) {
        return belief.blockedRoads.isEmpty() ? null
                : belief.blockedRoads.iterator().next();
    }

    private AgentAction buildMoveToBlockade(EntityID target) {
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

    public AgentAction getSelectedAction() { return selectedAction; }
}