package bayesian_agent.module.policy.police;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import bayesian_agent.util.Logger;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

public class PoliceForcePolicySelector {

    private final AgentInfo    agentInfo;
    private final WorldInfo    worldInfo;
    private final ScenarioInfo scenarioInfo;
    private final PathPlanning pathPlanning;

    private AgentAction selectedAction = AgentAction.rest();

    // Приоритетная цель от PoliceOffice 
    private EntityID overrideTarget = null;

    // Текущая целевая дорога (зафиксирована до расчистки)
    private EntityID committedRoad = null;

    // Блэклист дорог на которые агент физически не может попасть
    private final Set<EntityID> unreachableRoads = new HashSet<>();
    // Сколько тиков дорога считается недостижимой перед повторной попыткой
    private static final int UNREACHABLE_EXPIRY = 30;
    private final Map<EntityID, Integer> unreachableUntil = new HashMap<>();

    // Детектор stuck при движении к целевой дороге
    private double  lastX = Double.NaN, lastY = Double.NaN;
    private int     moveStuckCount   = 0;
    private boolean prevActionWasMove = false;   // не считать stuck во время CLEAR/REST
    private static final int MOVE_STUCK_THRESHOLD = 4;

    // Детектор stuck при расчистке (repairCost не меняется)
    private int    lastRepairCost  = Integer.MAX_VALUE;
    private int    clearStuckCount = 0;
    private static final int CLEAR_STUCK_THRESHOLD = 4;

    public PoliceForcePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo,
                                     ScenarioInfo scenarioInfo, PathPlanning pathPlanning) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.scenarioInfo = scenarioInfo;
        this.pathPlanning = pathPlanning;
    }

    public void setOverrideTarget(EntityID road) { this.overrideTarget = road; }
    public EntityID getOverrideTarget()          { return overrideTarget; }
    public void clearOverrideTarget()            { this.overrideTarget = null; }

    public void select(Belief belief) {
        int t = agentInfo.getTime();
        EntityID pos = agentInfo.getPosition();

        // Истёк срок блэклиста - убираем
        unreachableUntil.entrySet().removeIf(e -> e.getValue() <= t);
        unreachableRoads.removeIf(r -> !unreachableUntil.containsKey(r));

        // Приоритетная цель от PoliceOffice: форсируем committedRoad
        if (overrideTarget != null && belief.blockedRoads.contains(overrideTarget)) {
            committedRoad = overrideTarget;
        }

        // В ЗДАНИИ: выйти 
        if (pos != null && worldInfo.getEntity(pos) instanceof Building) {
            selectedAction = buildExitBuilding(pos);
            prevActionWasMove = selectedAction.type == AgentAction.Type.MOVE;
            Logger.info(agentInfo, "[PF] in_building=" + pos + " → " + selectedAction.type);
            return;
        }

        // НА ЗАБЛОКИРОВАННОЙ ДОРОГЕ: расчищать 
        if (pos != null && belief.blockedRoads.contains(pos)) {
            EntityID blockade = pickBlockadeOnRoad(pos);
            if (blockade != null) {
                StandardEntity be = worldInfo.getEntity(blockade);
                if (be instanceof Blockade) {
                    Blockade bl = (Blockade) be;
                    int agentX = (int) agentInfo.getX();
                    int agentY = (int) agentInfo.getY();

                    // Stuck-детектор: repairCost не меняется
                    if (bl.getRepairCost() >= lastRepairCost) {
                        clearStuckCount++;
                    } else {
                        clearStuckCount = 0;
                    }
                    lastRepairCost = bl.getRepairCost();

                    Logger.info(agentInfo, "[PF_CLEAR] road=" + pos
                        + " blockade=" + blockade
                        + " centroid=(" + bl.getX() + "," + bl.getY() + ")"
                        + " repairCost=" + bl.getRepairCost()
                        + " distToCent=" + (int) Math.hypot(bl.getX()-agentX, bl.getY()-agentY)
                        + " clearStuck=" + clearStuckCount);

                    if (clearStuckCount >= CLEAR_STUCK_THRESHOLD) {
                        // Уже N тиков нет прогресса - двигаемся к ближайшему апексу
                        // (агент мог застрять далеко от оставшихся тайлов)
                        Logger.warn(agentInfo, "[PF_CLEAR_STUCK] repairCost=" + bl.getRepairCost()
                            + " frozen → moveToApex");
                        clearStuckCount = 0;
                        lastRepairCost  = Integer.MAX_VALUE;
                        int[] ap = nearestApex(bl, agentX, agentY);
                        prevActionWasMove = true;
                        selectedAction = AgentAction.moveToPoint(ap[0], ap[1]);
                        return;
                    }

                    // CLEAR: ActionExecutor сам масштабирует таргет к центроиду
                    // в пределах clearRepairDistance. Никаких проверок дистанции здесь.
                    prevActionWasMove = false;
                    selectedAction = AgentAction.clear(blockade);
                    return;
                }
            }
            // Нет видимого завала - stale belief, идём на другую дорогу
            Logger.info(agentInfo, "[PF] on blocked road=" + pos + " no blockade → find other");
            clearStuckCount = 0;
            lastRepairCost  = Integer.MAX_VALUE;
            committedRoad   = null;
        }

        // CROSS-ROAD: завал на соседней дороге в радиусе расчистки - чистим не входя на неё
        if (pos != null) {
            EntityID nearbyBlockade = findNearbyBlockade(belief, pos);
            if (nearbyBlockade != null) {
                Blockade nbl = (Blockade) worldInfo.getEntity(nearbyBlockade);
                int agentX = (int) agentInfo.getX(), agentY = (int) agentInfo.getY();
                Logger.info(agentInfo, "[PF_CROSS_CLEAR] blockade=" + nearbyBlockade
                    + " centroid=(" + nbl.getX() + "," + nbl.getY() + ")"
                    + " dist=" + (int) Math.hypot(nbl.getX()-agentX, nbl.getY()-agentY)
                    + " repairCost=" + nbl.getRepairCost());
                moveStuckCount = 0; lastX = Double.NaN; lastY = Double.NaN;
                prevActionWasMove = false;
                selectedAction = AgentAction.clear(nearbyBlockade);
                return;
            }
        }

        // НЕ НА ЗАБЛОКИРОВАННОЙ ДОРОГЕ: двигаться к ближайшей

        // Сбросить committedRoad если уже расчищена
        if (committedRoad != null && !belief.blockedRoads.contains(committedRoad)) {
            Logger.info(agentInfo, "[PF] committedRoad=" + committedRoad + " cleared → reset");
            committedRoad = null;
        }

        if (committedRoad == null) {
            committedRoad = pickBestBlockedRoad(belief, pos);
            if (committedRoad != null)
                Logger.info(agentInfo, "[PF] new committedRoad=" + committedRoad);
        }

        if (committedRoad != null) {
            // Stuck-детектор движения - только когда прошлое действие было MOVE
            double cx = agentInfo.getX(), cy = agentInfo.getY();
            if (prevActionWasMove && !Double.isNaN(lastX) && Math.hypot(cx - lastX, cy - lastY) < 500) {
                moveStuckCount++;
            } else if (prevActionWasMove) {
                moveStuckCount = 0;
            }
            lastX = cx; lastY = cy;

            if (moveStuckCount >= MOVE_STUCK_THRESHOLD) {
                Logger.warn(agentInfo, "[PF_MOVE_STUCK] road=" + committedRoad
                    + " stuck " + moveStuckCount + " ticks → marking unreachable for " + UNREACHABLE_EXPIRY + " ticks");
                unreachableRoads.add(committedRoad);
                unreachableUntil.put(committedRoad, t + UNREACHABLE_EXPIRY);
                committedRoad  = null;
                moveStuckCount = 0;
                lastX = Double.NaN; lastY = Double.NaN;

                committedRoad = pickBestBlockedRoad(belief, pos);
                Logger.info(agentInfo, "[PF] after stuck, new committedRoad=" + committedRoad);
            }

            if (committedRoad != null) {
                List<EntityID> path = pathPlanning
                    .setFrom(pos).setDestination(committedRoad).calc().getResult();
                if (path != null && !path.isEmpty()) {
                    Logger.info(agentInfo, "[PF] MOVE → road=" + committedRoad + " pathLen=" + path.size());
                    prevActionWasMove = true;
                    selectedAction = AgentAction.move(path);
                    return;
                }
                // PathPlanning не нашёл путь → сразу в unreachable
                Logger.warn(agentInfo, "[PF] no path to road=" + committedRoad + " → unreachable");
                unreachableRoads.add(committedRoad);
                unreachableUntil.put(committedRoad, t + UNREACHABLE_EXPIRY);
                committedRoad = null;
            }
        }

        Logger.info(agentInfo, "[PF] nothing to do → REST");
        prevActionWasMove = false;
        selectedAction = AgentAction.rest();
    }

    // вспомогательные 

    private AgentAction buildExitBuilding(EntityID buildingId) {
        StandardEntity e = worldInfo.getEntity(buildingId);
        if (!(e instanceof Building)) return AgentAction.rest();
        for (EntityID nb : ((Building) e).getNeighbours()) {
            if (worldInfo.getEntity(nb) instanceof Building) continue;
            List<EntityID> path = pathPlanning.setFrom(buildingId).setDestination(nb).calc().getResult();
            if (path != null && !path.isEmpty()) return AgentAction.move(path);
        }
        return AgentAction.rest();
    }

    // Выбрать лучшую заблокированную дорогу: сначала на пути к жертвам, иначе ближайшая 
    private EntityID pickBestBlockedRoad(Belief belief, EntityID from) {
        if (from == null || belief.blockedRoads.isEmpty()) return null;

        // Приоритет: дороги на пути к живым жертвам
        EntityID victimRoad = pickRoadTowardVictim(belief, from);
        if (victimRoad != null) return victimRoad;

        // Иначе: ближайшая заблокированная дорога по длине пути
        EntityID best = null;
        int bestLen = Integer.MAX_VALUE;
        List<EntityID> candidates = new ArrayList<>(belief.blockedRoads);
        candidates.removeIf(r -> unreachableRoads.contains(r));
        
        // детерминировано распределяем агентов по разным дорогам
        candidates.sort(Comparator.comparingInt(EntityID::getValue));
        int offset = Math.abs(agentInfo.getID().getValue()) % Math.max(1, candidates.size());
        
        // пробуем от смещения по кругу
        for (int i = 0; i < candidates.size(); i++) {
            EntityID road = candidates.get((offset + i) % candidates.size());
            List<EntityID> path = pathPlanning.setFrom(from).setDestination(road).calc().getResult();
            if (path != null && path.size() < bestLen) {
                bestLen = path.size();
                best = road;
            }
        }
        return best;
    }

    private EntityID pickRoadTowardVictim(Belief belief, EntityID from) {
        if (belief.victims.isEmpty()) return null;
        EntityID bestVictim = null;
        double bestAlive = 0;
        for (var e : belief.victims.entrySet()) {
            if (e.getValue().pAlive() > bestAlive) {
                bestAlive  = e.getValue().pAlive();
                bestVictim = e.getKey();
            }
        }
        if (bestVictim == null || bestAlive < 0.01) return null;
        StandardEntity ve = worldInfo.getEntity(bestVictim);
        EntityID victimPos = null;
        if (ve instanceof rescuecore2.standard.entities.Human)
            victimPos = ((rescuecore2.standard.entities.Human) ve).getPosition();
        if (victimPos == null) return null;

        List<EntityID> path = pathPlanning.setFrom(from).setDestination(victimPos).calc().getResult();
        if (path == null) return null;

        List<EntityID> blocked = new ArrayList<>();
        for (EntityID step : path) {
            if (belief.blockedRoads.contains(step) && !unreachableRoads.contains(step))
                blocked.add(step);
        }
        if (blocked.isEmpty()) return null;
        int idx = Math.abs(agentInfo.getID().getValue()) % blocked.size();
        return blocked.get(idx);
    }

    private EntityID pickBlockadeOnRoad(EntityID roadId) {
        StandardEntity e = worldInfo.getEntity(roadId);
        if (!(e instanceof Road)) return null;
        Road road = (Road) e;
        if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) return null;
        List<EntityID> blockades = new ArrayList<>(road.getBlockades());
        blockades.sort(Comparator.comparingInt(EntityID::getValue));
        int idx = Math.abs(agentInfo.getID().getValue()) % blockades.size();
        return blockades.get(idx);
    }

    // Ближайший апекс завала к агенту (физически доступный край полигона) 
    private int[] nearestApex(Blockade bl, int agentX, int agentY) {
        int nearX = bl.getX(), nearY = bl.getY();
        double nearDist = Math.hypot(nearX - agentX, nearY - agentY);
        int[] apexes = bl.getApexes();
        if (apexes != null) {
            for (int i = 0; i < apexes.length / 2; i++) {
                int ax = apexes[i * 2], ay = apexes[i * 2 + 1];
                double d = Math.hypot(ax - agentX, ay - agentY);
                if (d < nearDist) { nearDist = d; nearX = ax; nearY = ay; }
            }
        }
        return new int[]{nearX, nearY};
    }

    //  Ищет завал на соседней заблокированной дороге в радиусе clearRepairDistance
    //  Возвращает EntityID завала, который можно расчистить с текущей позиции без входа на ту дорогу. 
    private EntityID findNearbyBlockade(Belief belief, EntityID currentPos) {
        int agentX  = (int) agentInfo.getX();
        int agentY  = (int) agentInfo.getY();
        int clearDist = scenarioInfo.getClearRepairDistance();

        EntityID best     = null;
        int      bestCost = 0;
        for (EntityID roadId : belief.blockedRoads) {
            if (roadId.equals(currentPos)) continue;
            StandardEntity re = worldInfo.getEntity(roadId);
            
            if (!(re instanceof Road road)) continue;
            if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) continue;
            
            for (EntityID blockadeId : road.getBlockades()) {
                StandardEntity be = worldInfo.getEntity(blockadeId);
                if (!(be instanceof Blockade bl)) continue;
                double dist = Math.hypot(bl.getX() - agentX, bl.getY() - agentY);
                if (dist <= clearDist && bl.getRepairCost() > bestCost) {
                    bestCost = bl.getRepairCost();
                    best = blockadeId;
                }
            }
        }
        return best;
    }

    public AgentAction getSelectedAction() { return selectedAction; }
}
