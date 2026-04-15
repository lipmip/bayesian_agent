package bayesian_agent.module.policy.police;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
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
    private EntityID committedRoad = null;
    private EntityID lastMoveTarget = null;
    private double lastX = Double.NaN;
    private double lastY = Double.NaN;
    private int moveStuckCount = 0;
    private static final int MOVE_STUCK_THRESHOLD = 3;

    public PoliceForcePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo,
                                     PathPlanning pathPlanning) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.pathPlanning = pathPlanning;
    }

    public void select(Belief belief) {
        EntityID agentPos = agentInfo.getPosition();

        // Стоим на заблокированной дороге - расчищаем
        if (agentPos != null && belief.blockedRoads.contains(agentPos)) {
            EntityID blockade = pickBlockadeOnRoad(agentPos);
            if (blockade != null) {
                StandardEntity b = worldInfo.getEntity(blockade);
                if (b instanceof Blockade) {
                    Blockade bl = (Blockade) b;
                    int currentCost = bl.getRepairCost();

                    if (blockade.equals(lastBlockade) && currentCost >= lastRepairCost) {
                        stuckCount++;
                    } else {
                        stuckCount = 0;
                    }
                    lastBlockade = blockade;
                    lastRepairCost = currentCost;

                    if (stuckCount >= STUCK_THRESHOLD) {
                        stuckCount = 0;
                        lastBlockade = null;
                        lastRepairCost = Integer.MAX_VALUE;
                        committedRoad = null; 
                        EntityID nextRoad = pickOtherBlockedRoad(belief, agentPos);
                        if (nextRoad != null) {
                            committedRoad = nextRoad;
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

        // Сбрасываем committedRoad если дорога уже расчищена
        if (committedRoad != null && !belief.blockedRoads.contains(committedRoad)) {
            System.err.println("[POLICE] committedRoad=" + committedRoad + " cleared, resetting");
            committedRoad = null;
        }

        // Есть жертвы - приоритет дорогам на пути к ним
        // Если уже зафиксировали цель - продолжаем двигаться к ней (не меняем при смене позиции)
        EntityID priorityRoad = committedRoad;
        if (priorityRoad == null) {
            priorityRoad = pickRoadTowardVictim(belief, agentPos);
            if (priorityRoad != null) {
                committedRoad = priorityRoad;

                System.err.println("[POLICE] committed to road=" + priorityRoad);
            }
        }
        if (priorityRoad != null) {
            System.err.println("[POLICE] victim-priority road=" + priorityRoad);

            // Детектор застревания: агент не двигается несколько тиков подряд
            double curX = agentInfo.getX();
            double curY = agentInfo.getY();
            if (priorityRoad.equals(lastMoveTarget)
                    && !Double.isNaN(lastX)
                    && Math.hypot(curX - lastX, curY - lastY) < 500) {
                moveStuckCount++;
            } else {
                moveStuckCount = 0;
            }
            lastMoveTarget = priorityRoad;
            lastX = curX;
            lastY = curY;

            if (moveStuckCount >= MOVE_STUCK_THRESHOLD) {
                System.err.println("[POLICE] MOVE stuck on road=" + priorityRoad
                    + ", abandoning");
                moveStuckCount = 0;
                committedRoad = null;
                // Пробуем другую заблокированную дорогу
                EntityID other = pickOtherBlockedRoad(belief, priorityRoad);
                if (other != null) {
                    committedRoad = other;
                    selectedAction = buildMoveToBlockade(other);
                    return;
                }
            }

            selectedAction = buildMoveToBlockade(priorityRoad);
            return;
        }

        // Обычная ближайшая заблокированная дорога
        EntityID road = pickBlockedRoad(belief);
        if (road != null) {
            System.err.println("[POLICE] random road=" + road);
            
            selectedAction = buildMoveToBlockade(road);
            return;
        }

        selectedAction = AgentAction.rest();
    }

    /**
     * Ищет первую заблокированную дорогу на пути к ближайшей живой жертве
     * Это направляет полицейского расчищать проходы к пострадавшим
     */
    private EntityID pickRoadTowardVictim(Belief belief, EntityID agentPos) {
        if (belief.victims.isEmpty() || agentPos == null) return null;

        // Берём жертву с максимальным pAlive
        EntityID victimId = null;
        double bestAlive = 0.0;
        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            if (e.getValue().pAlive() > bestAlive) {
                bestAlive = e.getValue().pAlive();
                victimId = e.getKey();
            }
        }
        if (victimId == null || bestAlive < 0.01) return null;

        // Получаем позицию жертвы
        StandardEntity ve = worldInfo.getEntity(victimId);
        EntityID victimPos = null;
        if (ve instanceof Human) {
            victimPos = ((Human) ve).getPosition();
        }
        if (victimPos == null) return null;

        // Строим путь к жертве
        List<EntityID> path = pathPlanning
            .setFrom(agentPos)
            .setDestination(victimPos)
            .calc()
            .getResult();
        if (path == null) return null;

        // Возвращаем первую заблокированную дорогу на пути
        for (EntityID step : path) {
            if (belief.blockedRoads.contains(step)) {
                return step;
            }
        }

        // Жертва в здании: path заканчивается на Building, которое не является дорогой
        // Проверяем соседей здания - ищем заблокированную дорогу у его входа
        StandardEntity dest = worldInfo.getEntity(victimPos);
        if (dest instanceof Building) {
            for (EntityID neighbour : ((Building) dest).getNeighbours()) {
                if (belief.blockedRoads.contains(neighbour)) {
                    return neighbour;
                }
            }
        }
        
        return null;
    }

    private EntityID pickOtherBlockedRoad(Belief belief, EntityID currentRoad) {
        for (EntityID road : belief.blockedRoads) {
            if (!road.equals(currentRoad)) return road;
        }
        return null;
    }

    private EntityID pickBlockadeOnRoad(EntityID roadId) {
        StandardEntity entity = worldInfo.getEntity(roadId);
        if (entity instanceof Road) {
            Road road = (Road) entity;

            if (road.isBlockadesDefined() && !road.getBlockades().isEmpty()) {
                EntityID bid = road.getBlockades().iterator().next();
                StandardEntity b = worldInfo.getEntity(bid);
                if (b instanceof Blockade) {
                    Blockade bl = (Blockade) b;
                    System.err.println("[DIAG] blockade x=" + bl.getX()
                        + " y=" + bl.getY()
                        + " repairCost=" + bl.getRepairCost());
                }

                return bid;
            }
        }

        return null;
    }

    private EntityID pickBlockedRoad(Belief belief) {
        return belief.blockedRoads.isEmpty() ? null
                : belief.blockedRoads.iterator().next();
    }

    private AgentAction buildMoveToBlockade(EntityID target) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return AgentAction.rest();

        // Если на целевой дороге уже есть завал - расчищаем (или подходим к нему)
        EntityID blockadeId = pickBlockadeOnRoad(target);
        if (blockadeId != null) {
            StandardEntity b = worldInfo.getEntity(blockadeId);
            if (b instanceof Blockade) {
                Blockade bl = (Blockade) b;
                double dist = Math.hypot(
                    bl.getX() - agentInfo.getX(),
                    bl.getY() - agentInfo.getY());
                if (dist < 7500) {
                    return AgentAction.clear(blockadeId);
                } else {
                    return AgentAction.moveToPoint(bl.getX(), bl.getY());
                }
            }
        }

        // Завала нет - просто двигаемся к целевой дороге
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