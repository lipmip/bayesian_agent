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

    // Блэклист конкретных блокад: завал, по которому не идёт прогресс,
    // помечается чтобы pickBlockadeOnRoad/findNearbyBlockade его не выбирали
    // Ключ = blockadeId, значение = тик, до которого блокада в чёрном списке
    private final Map<EntityID, Integer> blockadeBlacklistUntil = new HashMap<>();
    private static final int BLOCKADE_BLACKLIST_EXPIRY = 30;

    // Детектор stuck при движении к целевой дороге
    private double  lastX = Double.NaN, lastY = Double.NaN;
    private int     moveStuckCount   = 0;
    private boolean prevActionWasMove = false;   // не считать stuck во время CLEAR/REST
    private static final int MOVE_STUCK_THRESHOLD = 4;

    // Per-blockade tracking «cost не падает». Ключ = blockadeId,
    // значение = последний наблюдённый repairCost для этого завала
    private final Map<EntityID, Integer> lastRepairCostByBlockade = new HashMap<>();
    // Per-blockade счётчик тиков без прогресса.
    private final Map<EntityID, Integer> noProgressTicksByBlockade = new HashMap<>();
    // Жёсткий watchdog: после N тиков без падения cost - заносим завал в блэклист
    // Это срабатывает раньше moveToApex (CLEAR_STUCK_THRESHOLD=2) только когда
    // мы уже были у этого завала достаточно долго: моток BLOCKADE_NO_PROGRESS_LIMIT > 2
    private static final int BLOCKADE_NO_PROGRESS_LIMIT = 5;
    private static final int CLEAR_STUCK_THRESHOLD = 2;

    // Минимальная дистанция от агента до центроида: если меньше, считаем
    // что агент стоит ВНУТРИ полигона завала. CLEAR через себя бесполезен,
    // так что такие блокады отфильтровываем и/или принудительно отступаем
    private static final double DEGENERATE_TARGET_EPS = 500.0; // мм

    private final Map<EntityID, Integer> lastVisitedTick = new HashMap<>();
    private EntityID patrolTarget = null;

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

        // Истёк срок блэклиста дорог - убираем
        unreachableUntil.entrySet().removeIf(e -> e.getValue() <= t);
        unreachableRoads.removeIf(r -> !unreachableUntil.containsKey(r));

        // Истёк срок блэклиста конкретных блокад - убираем + лог
        var bIter = blockadeBlacklistUntil.entrySet().iterator();
        while (bIter.hasNext()) {
            var entry = bIter.next();
            if (entry.getValue() <= t) {
                Logger.info(agentInfo, "[PF_BLOCKADE_UNBLACKLIST] blockade=" + entry.getKey()
                    + " (expiry passed at t=" + t + ")");
                lastRepairCostByBlockade.remove(entry.getKey());
                noProgressTicksByBlockade.remove(entry.getKey());
                bIter.remove();
            }
        }

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
            EntityID blockade = pickBlockadeOnRoad(pos, t);
            if (blockade != null) {
                StandardEntity be = worldInfo.getEntity(blockade);
                if (be instanceof Blockade) {
                    Blockade bl = (Blockade) be;
                    int agentX = (int) agentInfo.getX();
                    int agentY = (int) agentInfo.getY();
                    int curCost = bl.getRepairCost();
                    double distToCent = Math.hypot(bl.getX()-agentX, bl.getY()-agentY);

                    // Per-blockade tracking: сравниваем cost ИМЕННО этого завала
                    Integer prevCost = lastRepairCostByBlockade.get(blockade);
                    int noProgress = noProgressTicksByBlockade.getOrDefault(blockade, 0);
                    if (prevCost != null && curCost >= prevCost) {
                        noProgress++;
                    } else {
                        noProgress = 0;
                    }
                    lastRepairCostByBlockade.put(blockade, curCost);
                    noProgressTicksByBlockade.put(blockade, noProgress);

                    Logger.info(agentInfo, "[PF_CLEAR] road=" + pos
                        + " blockade=" + blockade
                        + " centroid=(" + bl.getX() + "," + bl.getY() + ")"
                        + " repairCost=" + curCost
                        + " distToCent=" + (int) distToCent
                        + " noProgress=" + noProgress);

                    // Watchdog #1: degenerate target - агент стоит в/у центроида
                    // CLEAR через себя бесполезен, симулятор примет команду но
                    // ни один тайл не уберётся. Заносим завал в блэклист и
                    // отступаем к ближайшему апексу
                    if (distToCent < DEGENERATE_TARGET_EPS) {
                        Logger.warn(agentInfo, "[PF_DEGENERATE] xy≈centroid (dist="
                            + (int) distToCent + " < " + (int) DEGENERATE_TARGET_EPS + ")"
                            + " → blacklist blockade=" + blockade
                            + " for " + BLOCKADE_BLACKLIST_EXPIRY + " ticks, step away");
                        blacklistBlockade(blockade, t);
                        int[] ap = nearestApex(bl, agentX, agentY);
                        prevActionWasMove = true;
                        selectedAction = AgentAction.moveToPoint(ap[0], ap[1]);
                        return;
                    }

                    // Watchdog #2: жёсткий лимит - N тиков подряд cost не падает
                    // Срабатывает раньше, чем агент успеет застрять надолго,
                    // и навсегда выводит проблемный завал из выбора (на M тиков)
                    if (noProgress >= BLOCKADE_NO_PROGRESS_LIMIT) {
                        Logger.warn(agentInfo, "[PF_NO_PROGRESS] blockade=" + blockade
                            + " repairCost=" + curCost
                            + " stuck " + noProgress + " ticks → blacklist for "
                            + BLOCKADE_BLACKLIST_EXPIRY + " ticks");
                        blacklistBlockade(blockade, t);
                        // Сразу попробуем другую блокаду на этой же дороге в след. тик
                        prevActionWasMove = false;
                        selectedAction = AgentAction.rest();
                        return;
                    }

                    // Watchdog #3: мягкий - moveToApex после CLEAR_STUCK_THRESHOLD
                    // Сохранён, но теперь nearestApex возвращает РЕАЛЬНЫЙ апекс,
                    // а не центроид, поэтому телепорт-в-центроид невозможен
                    if (noProgress >= CLEAR_STUCK_THRESHOLD) {
                        Logger.warn(agentInfo, "[PF_CLEAR_STUCK] repairCost=" + curCost
                            + " noProgress=" + noProgress + " → moveToApex");
                        int[] ap = nearestApex(bl, agentX, agentY);
                        prevActionWasMove = true;
                        selectedAction = AgentAction.moveToPoint(ap[0], ap[1]);
                        return;
                    }

                    // CLEAR: ActionExecutor сам отодвигает target если он совпал
                    // с позицией агента (защита от degenerate-сектора)
                    prevActionWasMove = false;
                    selectedAction = AgentAction.clear(blockade);
                    return;
                }
            }
            // Нет видимого завала / все в блэклисте - stale belief, идём на другую дорогу
            Logger.info(agentInfo, "[PF] on blocked road=" + pos + " no usable blockade → find other");
            committedRoad = null;
        }

        // CROSS-ROAD: завал на соседней дороге в радиусе расчистки - чистим не входя на неё
        if (pos != null) {
            EntityID nearbyBlockade = findNearbyBlockade(belief, pos, t);
            if (nearbyBlockade != null) {
                Blockade nbl = (Blockade) worldInfo.getEntity(nearbyBlockade);
                int agentX = (int) agentInfo.getX(), agentY = (int) agentInfo.getY();
                int curCost = nbl.getRepairCost();
                double dist = Math.hypot(nbl.getX()-agentX, nbl.getY()-agentY);

                // Per-blockade tracking — те же поля, что и для on-road CLEAR
                Integer prevCost = lastRepairCostByBlockade.get(nearbyBlockade);
                int noProgress = noProgressTicksByBlockade.getOrDefault(nearbyBlockade, 0);
                if (prevCost != null && curCost >= prevCost) {
                    noProgress++;
                } else {
                    noProgress = 0;
                }
                lastRepairCostByBlockade.put(nearbyBlockade, curCost);
                noProgressTicksByBlockade.put(nearbyBlockade, noProgress);

                Logger.info(agentInfo, "[PF_CROSS_CLEAR] blockade=" + nearbyBlockade
                    + " centroid=(" + nbl.getX() + "," + nbl.getY() + ")"
                    + " dist=" + (int) dist
                    + " repairCost=" + curCost
                    + " noProgress=" + noProgress);

                // Watchdog: degenerate target в cross-режиме маловероятен (мы НЕ
                // на дороге блокады), но всё равно защищаемся
                if (dist < DEGENERATE_TARGET_EPS) {
                    Logger.warn(agentInfo, "[PF_CROSS_DEGENERATE] xy≈centroid → blacklist blockade="
                        + nearbyBlockade);
                    blacklistBlockade(nearbyBlockade, t);
                    prevActionWasMove = false;
                    selectedAction = AgentAction.rest();
                    return;
                }

                // Watchdog: cost не падает N тиков → блэклист
                // Раньше для CROSS_CLEAR защиты вообще не было: агент мог
                // вечно слать CLEAR на завал, до которого не достаёт сектор
                if (noProgress >= BLOCKADE_NO_PROGRESS_LIMIT) {
                    Logger.warn(agentInfo, "[PF_CROSS_NO_PROGRESS] blockade=" + nearbyBlockade
                        + " repairCost=" + curCost
                        + " stuck " + noProgress + " ticks → blacklist for "
                        + BLOCKADE_BLACKLIST_EXPIRY + " ticks");
                    blacklistBlockade(nearbyBlockade, t);
                    prevActionWasMove = false;
                    selectedAction = AgentAction.rest();
                    return;
                }

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

        // Нет заблокированных дорог в belief → патрулировать, чтобы обнаружить новые завалы
        AgentAction patrol = buildPatrolMove(pos);
        if (patrol.type != AgentAction.Type.REST) {
            Logger.info(agentInfo, "[PF] nothing to do → PATROL");
            prevActionWasMove = true;
            selectedAction = patrol;
            return;
        }

        Logger.info(agentInfo, "[PF] nothing to do → REST");
        prevActionWasMove = false;
        selectedAction = AgentAction.rest();
    }

    // вспомогательные

    // Патрульное движение: идём к наименее посещённой дороге карты (глобально, не только соседи)
    private AgentAction buildPatrolMove(EntityID pos) {
        if (pos == null) return AgentAction.rest();

        int t = agentInfo.getTime();
        lastVisitedTick.put(pos, t);

        if (pos.equals(patrolTarget))
            patrolTarget = null;

        if (patrolTarget == null)
            patrolTarget = pickPatrolTarget(pos);

        if (patrolTarget == null) return AgentAction.rest();

        List<EntityID> path = pathPlanning
            .setFrom(pos).setDestination(patrolTarget).calc().getResult();
        if (path != null && !path.isEmpty()) return AgentAction.move(path);

        // Цель недостижима - пометить и сбросить
        lastVisitedTick.put(patrolTarget, t);
        patrolTarget = null;
        return AgentAction.rest();
    }

    // Выбор цели патруля: сначала ни разу не посещённые дороги, затем самая давняя
    private EntityID pickPatrolTarget(EntityID from) {
        List<EntityID> unvisited = new ArrayList<>();
        List<EntityID> visited   = new ArrayList<>();
        for (StandardEntity e : worldInfo) {
            if (!(e instanceof Road)) continue;
            EntityID id = e.getID();
            if (id.equals(from)) continue;
            (lastVisitedTick.containsKey(id) ? visited : unvisited).add(id);
        }

        if (!unvisited.isEmpty()) {
            unvisited.sort(Comparator.comparingInt(EntityID::getValue));
            return unvisited.get(Math.abs(agentInfo.getID().getValue()) % unvisited.size());
        }
        if (!visited.isEmpty()) {
            visited.sort((a, b) -> Integer.compare(lastVisitedTick.get(a), lastVisitedTick.get(b)));
            return visited.get(0);
        }
        return null;
    }

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

        // Иначе: распределяем агентов по разным дорогам
        // Агент выбирает дорогу по смещённому рейтингу (pathLen + agent-hash * tieBreaker)
        // чтобы соседние агенты расходились по разным направлениям
        List<EntityID> candidates = new ArrayList<>(belief.blockedRoads);
        candidates.removeIf(r -> unreachableRoads.contains(r));
        if (candidates.isEmpty()) return null;
        candidates.sort(Comparator.comparingInt(EntityID::getValue));

        EntityID best = null;
        double bestScore = Double.MAX_VALUE;
        int agentHash = Math.abs(agentInfo.getID().getValue());
        for (int i = 0; i < candidates.size(); i++) {
            EntityID road = candidates.get(i);
            List<EntityID> path = pathPlanning.setFrom(from).setDestination(road).calc().getResult();
            if (path == null) continue;
            // Добавляем agent-specific перестановку чтобы разные агенты предпочитали разные дороги
            double score = path.size() + ((agentHash + i) % candidates.size()) * 0.1;
            if (score < bestScore) { bestScore = score; best = road; }
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

    // Помечает блокаду как «не выбирать на N тиков» и сбрасывает её tracker
    private void blacklistBlockade(EntityID blockadeId, int t) {
        blockadeBlacklistUntil.put(blockadeId, t + BLOCKADE_BLACKLIST_EXPIRY);
        lastRepairCostByBlockade.remove(blockadeId);
        noProgressTicksByBlockade.remove(blockadeId);
    }

    private boolean isBlackBlockade(EntityID blockadeId, int t) {
        Integer until = blockadeBlacklistUntil.get(blockadeId);
        return until != null && until > t;
    }

    private EntityID pickBlockadeOnRoad(EntityID roadId, int t) {
        StandardEntity e = worldInfo.getEntity(roadId);
        if (!(e instanceof Road)) return null;
        Road road = (Road) e;
        if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) return null;

        int agentX = (int) agentInfo.getX();
        int agentY = (int) agentInfo.getY();

        // Фильтруем (а) блокады в блэклисте; (б) с центроидом в xy агента
        List<EntityID> usable = new ArrayList<>();
        int skippedBlack = 0, skippedDegen = 0;
        for (EntityID bid : road.getBlockades()) {
            if (isBlackBlockade(bid, t)) { skippedBlack++; continue; }
            StandardEntity be = worldInfo.getEntity(bid);
            if (!(be instanceof Blockade)) continue;
            Blockade bl = (Blockade) be;
            double dist = Math.hypot(bl.getX() - agentX, bl.getY() - agentY);
            if (dist < DEGENERATE_TARGET_EPS) { skippedDegen++; continue; }
            usable.add(bid);
        }
        if (skippedBlack > 0 || skippedDegen > 0) {
            Logger.info(agentInfo, "[PF_PICK_ON_ROAD] road=" + roadId
                + " total=" + road.getBlockades().size()
                + " usable=" + usable.size()
                + " black=" + skippedBlack
                + " degen=" + skippedDegen);
        }
        if (usable.isEmpty()) return null;

        // Среди оставшихся выбираем ближайший к агенту по апексу
        // (или центроиду как fallback). Tie-breaker - детерминированный
        // hash от agentID, чтобы разные агенты разводились по целям
        usable.sort(Comparator.comparingInt(EntityID::getValue));
        EntityID best = null;
        double bestDist = Double.MAX_VALUE;
        for (EntityID bid : usable) {
            Blockade bl = (Blockade) worldInfo.getEntity(bid);
            double d = nearestApexDist(bl, agentX, agentY);
            if (d < bestDist) { bestDist = d; best = bid; }
        }
        return best;
    }

    // Расстояние до ближайшего апекса (или до центроида если апексов нет)
    private double nearestApexDist(Blockade bl, int agentX, int agentY) {
        int[] apexes = bl.getApexes();
        if (apexes == null || apexes.length < 2) {
            return Math.hypot(bl.getX() - agentX, bl.getY() - agentY);
        }
        double best = Double.MAX_VALUE;
        for (int i = 0; i < apexes.length / 2; i++) {
            double d = Math.hypot(apexes[i*2] - agentX, apexes[i*2+1] - agentY);
            if (d < best) best = d;
        }
        return best;
    }

    // Ближайший апекс завала к агенту (реальный край полигона, НЕ центроид)
    // Раньше функция инициализировала nearDist расстоянием до центроида и
    // обновляла его только если апекс был ближе - для агента, стоящего у
    // центроида, это всегда возвращало центроид, что приводило к телепорту
    // ВНУТРЬ блокады и зависанию навсегда. Теперь итерируем строго по апексам
    // Если апексов нет (пустой полигон) - fallback в точку, отступающую от
    // центроида в сторону агента, чтобы CLEAR-сектор был ненулевым
    private int[] nearestApex(Blockade bl, int agentX, int agentY) {
        int[] apexes = bl.getApexes();
        if (apexes != null && apexes.length >= 2) {
            int nearX = apexes[0], nearY = apexes[1];
            double nearDist = Math.hypot(nearX - agentX, nearY - agentY);
            for (int i = 1; i < apexes.length / 2; i++) {
                int ax = apexes[i * 2], ay = apexes[i * 2 + 1];
                double d = Math.hypot(ax - agentX, ay - agentY);
                if (d < nearDist) { nearDist = d; nearX = ax; nearY = ay; }
            }
            Logger.info(agentInfo, "[PF_APEX] real apex=(" + nearX + "," + nearY + ")"
                + " dist=" + (int) nearDist
                + " centroid=(" + bl.getX() + "," + bl.getY() + ")");
            return new int[]{nearX, nearY};
        }
        // Fallback: отступить от центроида в сторону агента на ~1м
        int cx = bl.getX(), cy = bl.getY();
        double dx = agentX - cx, dy = agentY - cy;
        double dist = Math.hypot(dx, dy);
        if (dist < 1.0) {
            // Агент в центроиде, апексов нет - двигаемся произвольно вверх на 1м
            Logger.warn(agentInfo, "[PF_APEX] no apexes & agent==centroid, fallback +1000Y");
            return new int[]{cx, cy + 1000};
        }
        double scale = Math.min(1000.0, dist) / dist;
        int fx = cx + (int)(dx * scale);
        int fy = cy + (int)(dy * scale);
        Logger.info(agentInfo, "[PF_APEX] fallback (no apexes) → (" + fx + "," + fy + ")");
        return new int[]{fx, fy};
    }

    // Залочиваемся на одной cross-блокаде до её исчезновения / попадания в блэклист
    // Раньше каждый тик брался max-cost в радиусе → агент пинг-понг между несколькими
    // блокадами, ни одну не доводя до конца. Теперь - sticky target
    private EntityID activeCrossTarget = null;

    //  Ищет завал на соседней заблокированной дороге в радиусе clearRepairDistance
    //  Возвращает EntityID завала, который можно расчистить с текущей позиции
    //  без входа на ту дорогу. Дистанция считается по ближайшему апексу,
    //  блокады в блэклисте и «через себя» отсекаются
    private EntityID findNearbyBlockade(Belief belief, EntityID currentPos, int t) {
        int agentX  = (int) agentInfo.getX();
        int agentY  = (int) agentInfo.getY();
        int clearDist = scenarioInfo.getClearRepairDistance();

        // Sticky: если активная цель ещё валидна - продолжаем её
        if (activeCrossTarget != null) {
            StandardEntity be = worldInfo.getEntity(activeCrossTarget);
            if (be instanceof Blockade bl
                && !isBlackBlockade(activeCrossTarget, t)
                && nearestApexDist(bl, agentX, agentY) <= clearDist
                && Math.hypot(bl.getX() - agentX, bl.getY() - agentY) >= DEGENERATE_TARGET_EPS) {
                return activeCrossTarget;
            }
            Logger.info(agentInfo, "[PF_CROSS_LOCK_RELEASE] activeCrossTarget="
                + activeCrossTarget + " no longer valid → reselect");
            activeCrossTarget = null;
        }

        EntityID best        = null;
        double   bestApexDst = Double.MAX_VALUE;
        int      skippedBlack = 0, skippedDegen = 0, skippedFar = 0;
        for (EntityID roadId : belief.blockedRoads) {
            if (roadId.equals(currentPos)) continue;
            StandardEntity re = worldInfo.getEntity(roadId);

            if (!(re instanceof Road road)) continue;
            if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) continue;

            for (EntityID blockadeId : road.getBlockades()) {
                if (isBlackBlockade(blockadeId, t)) { skippedBlack++; continue; }
                StandardEntity be = worldInfo.getEntity(blockadeId);
                if (!(be instanceof Blockade bl)) continue;
                double centDist = Math.hypot(bl.getX() - agentX, bl.getY() - agentY);
                if (centDist < DEGENERATE_TARGET_EPS) { skippedDegen++; continue; }
                double apexDist = nearestApexDist(bl, agentX, agentY);
                if (apexDist > clearDist) { skippedFar++; continue; }
                if (apexDist < bestApexDst) {
                    bestApexDst = apexDist;
                    best = blockadeId;
                }
            }
        }
        if (best != null) {
            activeCrossTarget = best;
            Logger.info(agentInfo, "[PF_CROSS_LOCK] new activeCrossTarget=" + best
                + " apexDist=" + (int) bestApexDst
                + " (skipped: black=" + skippedBlack
                + " degen=" + skippedDegen
                + " far=" + skippedFar + ")");
        }
        return best;
    }

    public AgentAction getSelectedAction() { return selectedAction; }
}
