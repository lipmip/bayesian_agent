package bayesian_agent.module.pomcp;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import bayesian_agent.util.Logger;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Civilian;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

// FSM-контроллер агента-санитара
// Перенесён из AmbulancePolicySelector для использования в POMCP как rollout-политика
// Публичный интерфейс: tick(Belief) → AgentAction
public class StateMachineController {

    private final AgentInfo    agentInfo;
    private final WorldInfo    worldInfo;
    private final PathPlanning pathPlanning;

    // FSM
    private AgentMacroState macroState     = AgentMacroState.EXPLORE;
    private AgentMacroState prevMacroState = AgentMacroState.EXPLORE;
    private EntityID        targetVictimId = null;
    private int             ticksInState   = 0;

    // Последняя дорога, заблокировавшая путь к жертве при переходе в WAIT_FOR_POLICE
    private EntityID lastBlockedRoad = null;

    // Stuck detection при транспортировке жертвы
    private final Set<EntityID> failedRefuges    = new HashSet<>();
    private EntityID currentRefugeTarget  = null;
    private double   lastRefugeDist       = Double.MAX_VALUE;
    private int      carryStuckCount      = 0;
    private int      allRefugesFailedWait = 0;
    // Порог: < 500мм прогресса за тик считается отсутствием движения к убежищу.
    // Агент может физически двигаться, но не приближаться к цели из-за завалов.
    private static final int    CARRY_STUCK_THRESHOLD   = 8;
    private static final double CARRY_PROGRESS_MIN      = 500.0;
    // После исчерпания всех убежищ ждём перед повторной попыткой — даём полиции время расчистить путь.
    private static final int    ALL_REFUGES_FAILED_WAIT = 10;

    // Bug 2: повторные таймауты навигации к одной жертве → пропустить её
    private final Map<EntityID, Integer> navigateTimeouts = new HashMap<>();
    private final Set<EntityID>          skipVictims      = new HashSet<>();
    private static final int MAX_NAVIGATE_TIMEOUTS = 2;

    // Bug 3: застрял в TRANSPORT (atLocation, но LOAD не срабатывает) → пропустить жертву
    private int      transportLoadTicks  = 0;
    private EntityID transportLoadTarget = null;
    private static final int TRANSPORT_LOAD_TIMEOUT = 5;

    // Bug 4: T5b-осциляция - жертва каждый тик недостижима (injCrit≤0.4), агент не входит в NAVIGATE
    // Считаем consecutive тики с одной и той же недостижимой жертвой → пропускаем после порога
    private EntityID lastT5bVictim  = null;
    private int      t5bVictimTicks = 0;
    private static final int T5B_VICTIM_TIMEOUT = 8;

    private final Map<EntityID, Integer> lastVisitedTick = new HashMap<>();

    public StateMachineController(AgentInfo agentInfo, WorldInfo worldInfo,
                                  PathPlanning pathPlanning) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.pathPlanning = pathPlanning;
    }

    // Один вызов в тик. Объединяет stuck detection + FSM + построение действия
    public AgentAction tick(Belief belief) {
        EntityID pos = agentInfo.getPosition();

        // Stuck detection для режима доставки
        // Проверяем прогресс по расстоянию до текущего убежища, а не по равенству EntityID позиции
        // Агент может физически двигаться между соседними дорогами, но не приближаться к убежищу
        // из-за заблокированных маршрутов - в этом случае pos.equals(lastCarryPos) никогда не сработает
        if (agentInfo.someoneOnBoard() != null) {
            if (currentRefugeTarget != null) {
                StandardEntity re = worldInfo.getEntity(currentRefugeTarget);
                if (re instanceof Area) {
                    double dist = Math.hypot(((Area) re).getX() - agentInfo.getX(),
                                             ((Area) re).getY() - agentInfo.getY());
                    if (dist >= lastRefugeDist - CARRY_PROGRESS_MIN) {
                        carryStuckCount++;
                        if (carryStuckCount >= CARRY_STUCK_THRESHOLD) {
                            Logger.info(agentInfo, "[CARRY_STUCK] refuge=" + currentRefugeTarget
                                + " dist=" + (int)dist + " no progress for " + carryStuckCount + " ticks");
                            failedRefuges.add(currentRefugeTarget);
                            currentRefugeTarget = null;
                            lastRefugeDist = Double.MAX_VALUE;
                            carryStuckCount = 0;
                        }
                    } else {
                        carryStuckCount = 0;
                    }
                    lastRefugeDist = dist;
                }
            }
        } else {
            carryStuckCount = 0;
            lastRefugeDist = Double.MAX_VALUE;
            allRefugesFailedWait = 0;
            failedRefuges.clear();
            currentRefugeTarget = null;
        }

        // Переходы FSM
        macroState = evaluateTransition(belief, pos);

        // Лог только при смене состояния; при зависании - напоминание каждые 20 тиков
        if (macroState != prevMacroState) {
            Logger.info(agentInfo, "[FSM] " + prevMacroState + "→" + macroState
                + (targetVictimId != null ? " target=" + targetVictimId : "")
                + " (was " + ticksInState + " ticks in " + prevMacroState + ")");
            prevMacroState = macroState;
            ticksInState   = 0;
        } else {
            ticksInState++;
            if (ticksInState % 20 == 0 && macroState == AgentMacroState.WAIT_FOR_POLICE) {
                Logger.warn(agentInfo, "[FSM] stuck in WAIT_FOR_POLICE for "
                    + ticksInState + " ticks, target=" + targetVictimId);
            }
        }

        // Построение действия по макросостоянию
        return switch (macroState) {
            case EXPLORE         -> buildSearchMove();
            case NAVIGATE        -> buildRescueOrMoveAction(targetVictimId, belief);
            case WAIT_FOR_POLICE -> AgentAction.rest();
            case RESCUE          -> AgentAction.rest();  
            case TRANSPORT       -> {
                if (agentInfo.someoneOnBoard() != null) {
                    yield buildUnloadAction(belief);
                } else {
                    yield AgentAction.load(targetVictimId);
                }
            }
            case DELIVER         -> AgentAction.unload();
        };
    }

    private AgentMacroState evaluateTransition(Belief belief, EntityID pos) {
        // T2: AG=Carrying → TRANSPORT (любое состояние, включая EXPLORE)
        if (agentInfo.someoneOnBoard() != null) {
            return AgentMacroState.TRANSPORT;
        }

        // TRANSPORT stuck: LOAD не срабатывает - проверяем до pickBestVictim,
        // чтобы жертва не бросалась при временном выпадении из belief
        if (macroState == AgentMacroState.TRANSPORT && targetVictimId != null) {
            if (!targetVictimId.equals(transportLoadTarget)) {
                transportLoadTarget = targetVictimId;
                transportLoadTicks  = 0;
            }
            transportLoadTicks++;
            if (transportLoadTicks >= TRANSPORT_LOAD_TIMEOUT) {
                EntityID stuck = transportLoadTarget;
                transportLoadTarget = null;
                transportLoadTicks  = 0;
                Logger.warn(agentInfo, "[FSM] TRANSPORT load stuck, skip victim=" + stuck);
                skipVictims.add(stuck);
                targetVictimId = null;
                // fall through to pickBestVictim for new target
            } else {
                return AgentMacroState.TRANSPORT;
            }
        } else if (macroState != AgentMacroState.TRANSPORT) {
            transportLoadTicks  = 0;
            transportLoadTarget = null;
        }

        // Выбрать лучшую жертву
        // В состоянии NAVIGATE не менять цель (предотвращает осцилляцию при равных скорах)
        EntityID bestVictim = pickBestVictim(belief);
        if (bestVictim == null) return AgentMacroState.EXPLORE;
        if (macroState != AgentMacroState.NAVIGATE
                || targetVictimId == null
                || !belief.victims.containsKey(targetVictimId)) {
            targetVictimId = bestVictim;
        }

        Belief.VictimBelief tvb = belief.victims.get(targetVictimId);
        if (tvb == null) return AgentMacroState.EXPLORE;

        // Критичность: b(VHP=Critical) + b(VHP=Injured) > 0.4
        double injCrit = tvb.pInjured + tvb.pCritical;

        // T6: b(VHP=Dead) > 0.8 → Explore
        if (tvb.pDead > 0.8) return AgentMacroState.EXPLORE;

        // Сбросить T5b-счётчик если жертва изменилась или путь теперь свободен
        if (!targetVictimId.equals(lastT5bVictim)) {
            lastT5bVictim  = null;
            t5bVictimTicks = 0;
        }

        // T3/T4: уже рядом с жертвой - пробуем грузить (AKLoad не требует buriedness=0)
        if (atVictimLocation(pos, targetVictimId)) {
            // Синхронизируем likelyBuried с worldInfo: TrafficSimulator отвергает AKLoad для засыпанных
            int realBuried = getRealBuriedness(targetVictimId);
            if (realBuried > 0) tvb.likelyBuried = true;
            else if (realBuried == 0) tvb.likelyBuried = false;

            if (tvb.likelyBuried) {
                // Жертва ещё засыпана - не выдавать AKLoad. Идём к другой жертве или исследуем
                EntityID alt = pickAccessibleVictim(belief, pos, targetVictimId);
                if (alt != null) { targetVictimId = alt; return AgentMacroState.NAVIGATE; }
                return AgentMacroState.EXPLORE;
            }
            if (tvb.pAlive() > 0.1) return AgentMacroState.TRANSPORT;
        }

        // Проверить доступность пути
        boolean pathBlocked = isPathBlockedToVictim(belief, pos, targetVictimId);

        // T5b: путь заблокирован, жертва не критична → разведка вместо ожидания
        if (pathBlocked && injCrit <= 0.4) {
            if (targetVictimId.equals(lastT5bVictim)) {
                t5bVictimTicks++;
            } else {
                lastT5bVictim  = targetVictimId;
                t5bVictimTicks = 1;
            }
            if (t5bVictimTicks >= T5B_VICTIM_TIMEOUT) {
                int timeouts = navigateTimeouts.merge(targetVictimId, 1, Integer::sum);
                Logger.warn(agentInfo, "[FSM] T5b stuck " + t5bVictimTicks
                    + " ticks, skip victim=" + targetVictimId + " timeouts=" + timeouts);
                if (timeouts >= MAX_NAVIGATE_TIMEOUTS) {
                    skipVictims.add(targetVictimId);
                    Logger.warn(agentInfo, "[FSM] T5b skip victim=" + targetVictimId);
                }
                lastT5bVictim  = null;
                t5bVictimTicks = 0;
            }
            return AgentMacroState.EXPLORE;
        }

        // T5: путь заблокирован, жертва критична → ждать полицию (или идти к другой)
        if (pathBlocked) {
            EntityID alt = pickAccessibleVictim(belief, pos, targetVictimId);
            if (alt != null) {
                targetVictimId = alt;
                return AgentMacroState.NAVIGATE;
            }
            lastBlockedRoad = findBlockedRoadOnPath(belief, pos, targetVictimId);
            return AgentMacroState.WAIT_FOR_POLICE;
        }

        // Таймаут NAVIGATE: если застряли без прогресса - сбросить цель и исследовать
        if (macroState == AgentMacroState.NAVIGATE && ticksInState > 15) {
            int timeouts = navigateTimeouts.merge(targetVictimId, 1, Integer::sum);
            Logger.warn(agentInfo, "[FSM] NAVIGATE timeout (" + ticksInState
                + " ticks), dropping target=" + targetVictimId + " timeouts=" + timeouts);
            if (timeouts >= MAX_NAVIGATE_TIMEOUTS) {
                Logger.warn(agentInfo, "[FSM] skip victim=" + targetVictimId + " after " + timeouts + " timeouts");
                skipVictims.add(targetVictimId);
            }
            targetVictimId = null;
            return AgentMacroState.EXPLORE;
        }

        return AgentMacroState.NAVIGATE;
    }

    private boolean atVictimLocation(EntityID agentPos, EntityID victimId) {
        EntityID vPos = getEntityPosition(victimId);
        return agentPos != null && agentPos.equals(vPos);
    }

    // Возвращает лучшую жертву (кроме excludeId) с незаблокированным путём, или null
    private EntityID pickAccessibleVictim(Belief belief, EntityID pos, EntityID excludeId) {
        EntityID best = null;
        double bestScore = -1.0;
        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            if (e.getKey().equals(excludeId)) continue;
            Belief.VictimBelief vb = e.getValue();
            if (vb.pAlive() < 0.01 || vb.pDead > 0.8) continue;

            EntityID victimPos = getEntityPosition(e.getKey());
            if (victimPos == null) continue;
            if (worldInfo.getEntity(victimPos) instanceof Refuge) continue;
            if (isPathBlockedToVictim(belief, pos, e.getKey())) continue;

            double score = (vb.pCritical * 2.0 + vb.pInjured) * vb.pAlive();
            if (score > bestScore) { bestScore = score; best = e.getKey(); }
        }
        return best;
    }

    private boolean isPathBlockedToVictim(Belief belief, EntityID from, EntityID victimId) {
        if (from == null || victimId == null) return false;
        EntityID dest = getEntityPosition(victimId);
        if (dest == null) return true;  // позиция неизвестна → недостижима
        List<EntityID> path = pathPlanning
            .setFrom(from).setDestination(dest).calc().getResult();
        if (path == null) return true;
        for (EntityID step : path) {
            Double prob = belief.roadBlockedProb.get(step);
            if (prob != null && prob > 0.75) return true;
        }
        return false;
    }

    private EntityID findBlockedRoadOnPath(Belief belief, EntityID from, EntityID victimId) {
        if (from == null || victimId == null) return null;
        EntityID dest = getEntityPosition(victimId);
        if (dest == null) return null;
        List<EntityID> path = pathPlanning
            .setFrom(from).setDestination(dest).calc().getResult();
        if (path == null) return null;
        for (EntityID step : path) {
            Double prob = belief.roadBlockedProb.get(step);
            if (prob != null && prob > 0.75) return step;
        }
        return null;
    }

    private EntityID pickBestVictim(Belief belief) {
        // Периодически даём второй шанс скипнутым жертвам (могли стать доступными)
        // navigateTimeouts НЕ сбрасываются: уже-проваленные жертвы будут скипнуты
        // быстрее (после 1 таймаута вместо 2), снижая потери на повторные попытки
        int t = agentInfo.getTime();
        if (t % 100 == 0 && !skipVictims.isEmpty()) {
            Logger.info(agentInfo, "[FSM] clearing skipVictims=" + skipVictims.size());
            skipVictims.clear();
        }
        EntityID best = null;
        double bestScore = -1.0;
        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            if (skipVictims.contains(e.getKey())) continue;
            Belief.VictimBelief vb = e.getValue();
            if (vb.pAlive() < 0.01) continue;

            EntityID victimPos = getEntityPosition(e.getKey());
            if (victimPos == null) continue;  // позиция неизвестна - нет смысла навигировать
            if (worldInfo.getEntity(victimPos) instanceof Refuge) continue;
            double score = (vb.pCritical * 2.0 + vb.pInjured) * vb.pAlive();
            if (score > bestScore) {
                bestScore = score;
                best = e.getKey();
            }
        }
        return best;
    }

    private AgentAction buildRescueOrMoveAction(EntityID victimId, Belief belief) {
        EntityID agentPos  = agentInfo.getPosition();
        EntityID victimPos = getEntityPosition(victimId);

        if (agentPos != null && agentPos.equals(victimPos)) {
            return AgentAction.load(victimId);
        }

        if (victimPos != null && agentPos != null) {
            List<EntityID> path = pathPlanning
                .setFrom(agentPos).setDestination(victimPos).calc().getResult();
            if (path != null && !path.isEmpty()) return AgentAction.move(path);
            Logger.info(agentInfo, "[NAVIGATE] no path to victim=" + victimId + " pos=" + victimPos);
        }

        return AgentAction.rest();
    }

    // Строит действие для доставки жертвы в убежище
    // Выбирает ближайшее доступное убежище. При застревании помечает как failed
    private AgentAction buildUnloadAction(Belief belief) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return AgentAction.rest();

        StandardEntity posEntity = worldInfo.getEntity(pos);
        if (!(posEntity instanceof Area)) {
            return AgentAction.unload();
        }
        if (posEntity instanceof Refuge) {
            return AgentAction.unload();
        }

        EntityID bestRefuge = null;
        List<EntityID> bestPath = null;
        int bestLen = Integer.MAX_VALUE;

        for (EntityID refugeId : worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE)) {
            if (pos.equals(refugeId)) return AgentAction.unload();
            if (failedRefuges.contains(refugeId)) continue;

            Belief.RefugeState state = belief.knownRefuges.get(refugeId);
            if (state == Belief.RefugeState.FULL) continue;  // T13: RB=Full → выбрать другое убежище
            
            List<EntityID> path = pathPlanning
                .setFrom(pos).setDestination(refugeId).calc().getResult();
            if (path != null && !path.isEmpty() && path.size() < bestLen) {
                bestLen   = path.size();
                bestRefuge = refugeId;
                bestPath  = path;
            }
        }

        if (bestRefuge != null) {
            currentRefugeTarget = bestRefuge;
            return AgentAction.move(bestPath);
        }

        if (!failedRefuges.isEmpty()) {
            allRefugesFailedWait++;
            if (allRefugesFailedWait >= ALL_REFUGES_FAILED_WAIT) {
                Logger.info(agentInfo, "[CARRY] all refuges failed, retrying after " + allRefugesFailedWait + " ticks");
                failedRefuges.clear();
                currentRefugeTarget = null;
                allRefugesFailedWait = 0;
                lastRefugeDist = Double.MAX_VALUE;
            }
        }
        return AgentAction.rest();
    }

    private AgentAction buildSearchMove() {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return AgentAction.rest();

        int t = agentInfo.getTime();
        lastVisitedTick.put(pos, t);

        StandardEntity e = worldInfo.getEntity(pos);
        if (!(e instanceof Area)) return AgentAction.rest();

        List<EntityID> neighbours = new ArrayList<>(((Area) e).getNeighbours());
        if (neighbours.isEmpty()) return AgentAction.rest();

        List<EntityID> unvisited = new ArrayList<>();
        List<EntityID> visited   = new ArrayList<>();
        for (EntityID nb : neighbours) {
            (lastVisitedTick.containsKey(nb) ? visited : unvisited).add(nb);
        }

        List<EntityID> candidates;
        if (!unvisited.isEmpty()) {
            unvisited.sort(Comparator.comparingInt(EntityID::getValue));
            int start = Math.abs(agentInfo.getID().getValue()) % unvisited.size();
            candidates = new ArrayList<>();
            for (int i = 0; i < unvisited.size(); i++)
                candidates.add(unvisited.get((start + i) % unvisited.size()));
        } else {
            visited.sort((a, b) -> Integer.compare(lastVisitedTick.get(a), lastVisitedTick.get(b)));
            candidates = visited;
        }

        for (EntityID next : candidates) {
            List<EntityID> path = pathPlanning
                .setFrom(pos).setDestination(next).calc().getResult();
            if (path != null && !path.isEmpty()) return AgentAction.move(path);
            lastVisitedTick.put(next, t);
        }
        return AgentAction.rest();
    }

    private EntityID getEntityPosition(EntityID entityId) {
        StandardEntity entity = worldInfo.getEntity(entityId);
        if (entity instanceof Human) {
            EntityID pos = ((Human) entity).getPosition();
            if (pos == null) return null;
            StandardEntity posEntity = worldInfo.getEntity(pos);
            if (posEntity instanceof Area) return pos;
            return null;
        }
        return null;
    }

    // Вызывается из POMCP, когда при выборе MOVE наш FSM возвращает REST (e.g. WAIT_FOR_POLICE)
    public AgentAction buildForceNavigate(Belief belief) {
        if (targetVictimId == null) return buildSearchMove();
        return buildRescueOrMoveAction(targetVictimId, belief);
    }

    // Возвращает buriedness из worldInfo: 0 = свободна, >0 = засыпана, -1 = нет данных
    private int getRealBuriedness(EntityID victimId) {
        StandardEntity e = worldInfo.getEntity(victimId);
        if (!(e instanceof Civilian)) return -1;
        Civilian civ = (Civilian) e;
        if (!civ.isBuriednessDefined()) return -1;
        return civ.getBuriedness();
    }

    private void logRescueProgress(EntityID victimId) {
        StandardEntity e = worldInfo.getEntity(victimId);
        if (!(e instanceof Civilian)) return;
        Civilian civ = (Civilian) e;
        int buried = civ.isBuriednessDefined() ? civ.getBuriedness() : -1;
        int hp     = civ.isHPDefined()         ? civ.getHP()         : -1;
        int dmg    = civ.isDamageDefined()      ? civ.getDamage()     : -1;
        EntityID agentPos  = agentInfo.getPosition();
        EntityID victimPos = civ.isPositionDefined() ? civ.getPosition() : null;
        boolean sameArea   = agentPos != null && agentPos.equals(victimPos);
        Logger.info(agentInfo, "[RESCUE] tick=" + agentInfo.getTime()
            + " victim=" + victimId
            + " buriedness=" + buried
            + " hp=" + hp
            + " dmg=" + dmg
            + " rescueTick=" + ticksInState
            + " agentPos=" + agentPos
            + " victimPos=" + victimPos
            + " sameArea=" + sameArea);
    }

    // Вызывается из TacticsAmbulanceTeam при получении COMM-сигнала об освобождённой жертве
    public void notifyVictimFreed(EntityID victimId) {
        skipVictims.remove(victimId);
        navigateTimeouts.remove(victimId);
        Belief.VictimBelief vb = null;  // belief недоступен здесь - очистим только skip/timeout
        Logger.info(agentInfo, "[FSM] victim freed (COMM): removed from skipVictims id=" + victimId);
    }

    public AgentMacroState getCurrentState()    { return macroState; }
    public EntityID         getTargetVictimId() { return targetVictimId; }
    public EntityID         getLastBlockedRoad() { return lastBlockedRoad; }
}
