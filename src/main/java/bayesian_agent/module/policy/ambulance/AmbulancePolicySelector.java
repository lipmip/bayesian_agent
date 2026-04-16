package bayesian_agent.module.policy.ambulance;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import bayesian_agent.util.Logger;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

enum AgentMacroState {
    EXPLORE, NAVIGATE, WAIT_FOR_POLICE, RESCUE, TRANSPORT, DELIVER
}

/**
 * Выбор действия для агента-санитара: b_t → a_t.
 * [ЗАМЕНИТЬ: POMCP]
 */
public class AmbulancePolicySelector {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;
    private final PathPlanning pathPlanning;
    private AgentAction selectedAction = AgentAction.rest();

    // FSM
    private AgentMacroState macroState     = AgentMacroState.EXPLORE;
    private AgentMacroState prevMacroState = AgentMacroState.EXPLORE;
    private EntityID targetVictimId        = null;

    // Застревание при доставке жертвы
    private final Set<EntityID> failedRefuges = new HashSet<>();
    private EntityID currentRefugeTarget = null;
    private EntityID lastCarryPos = null;
    private int carryStuckCount = 0;
    private static final int CARRY_STUCK_THRESHOLD = 5;

    public AmbulancePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo,
                                   PathPlanning pathPlanning) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.pathPlanning = pathPlanning;
    }

    public void select(Belief belief) {
        EntityID pos = agentInfo.getPosition();

        // Stuck detection для режима доставки
        if (agentInfo.someoneOnBoard() != null) {
            if (pos != null && pos.equals(lastCarryPos)) {
                carryStuckCount++;
                if (carryStuckCount >= CARRY_STUCK_THRESHOLD && currentRefugeTarget != null) {
                    failedRefuges.add(currentRefugeTarget);
                    currentRefugeTarget = null;
                    carryStuckCount = 0;
                }
            } else {
                carryStuckCount = 0;
            }
            lastCarryPos = pos;
        } else {
            lastCarryPos = null;
            carryStuckCount = 0;
            failedRefuges.clear();
            currentRefugeTarget = null;
        }

        // Переходы FSM по порогам belief из ВКР
        macroState = evaluateTransition(belief, pos);

        // Лог только при смене состояния
        if (macroState != prevMacroState) {
            Logger.info(agentInfo, "[FSM] " + prevMacroState + "→" + macroState
                + (targetVictimId != null ? " target=" + targetVictimId : ""));
            prevMacroState = macroState;
        }

        switch (macroState) {
            case EXPLORE         -> selectedAction = buildSearchMove();
            case NAVIGATE        -> selectedAction = buildRescueOrMoveAction(targetVictimId, belief);
            case WAIT_FOR_POLICE -> selectedAction = AgentAction.rest(); // этап 5: SAY/TELL
            // Пробуем LOAD каждый тик: если FireBrigade только что освободил жертву
            // (buriedness стало 0), LOAD сработает немедленно без потери тика.
            // Если жертва ещё погребена - LOAD отклоняется симулятором тихо (= REST).
            case RESCUE          -> selectedAction = AgentAction.load(targetVictimId);
            case TRANSPORT       -> {
                if (agentInfo.someoneOnBoard() != null) {
                    // Уже везём жертву → едем в убежище
                    selectedAction = buildUnloadAction(belief);
                } else {
                    // T4: находимся у жертвы, !buried, injCrit > 0.3 → LOAD
                    selectedAction = AgentAction.load(targetVictimId);
                }
            }
            case DELIVER         -> selectedAction = AgentAction.unload();
        }
    }

    private AgentMacroState evaluateTransition(Belief belief, EntityID pos) {
        // Если несём жертву - сразу TRANSPORT
        if (agentInfo.someoneOnBoard() != null) {
            return AgentMacroState.TRANSPORT;
        }

        // Выбрать лучшую жертву
        EntityID bestVictim = pickBestVictim(belief);
        if (bestVictim == null) return AgentMacroState.EXPLORE;
        targetVictimId = bestVictim;

        Belief.VictimBelief tvb = belief.victims.get(targetVictimId);
        if (tvb == null) return AgentMacroState.EXPLORE;

        double injCrit = tvb.pInjured + tvb.pCritical;

        // T6: b(VHP=Dead) > 0.8 → Explore
        if (tvb.pDead > 0.8) return AgentMacroState.EXPLORE;

        // Проверить доступность пути
        boolean pathBlocked = isPathBlockedToVictim(belief, pos, targetVictimId);

        // T5: b(PC=Blocked) > 0.75 AND injCrit > 0.4 → WaitForPolice
        if (pathBlocked && injCrit > 0.4) return AgentMacroState.WAIT_FOR_POLICE;

        // T3: atLocation AND b(VB=Buried)>0.5 AND injCrit>0.3 → Rescue
        if (atVictimLocation(pos, targetVictimId) && tvb.likelyBuried && injCrit > 0.3)
            return AgentMacroState.RESCUE;

        // T4: atLocation AND b(VB=Free)>0.7 AND injCrit>0.3 → Transport (LOAD)
        if (atVictimLocation(pos, targetVictimId) && !tvb.likelyBuried && injCrit > 0.3)
            return AgentMacroState.TRANSPORT;

        // Иначе: Navigate к жертве
        return AgentMacroState.NAVIGATE;
    }

    private boolean atVictimLocation(EntityID agentPos, EntityID victimId) {
        EntityID vPos = getEntityPosition(victimId);
        return agentPos != null && agentPos.equals(vPos);
    }

    private boolean isPathBlockedToVictim(Belief belief, EntityID from, EntityID victimId) {
        if (from == null || victimId == null) return false;

        EntityID dest = getEntityPosition(victimId);
        if (dest == null) return false;
        List<EntityID> path = pathPlanning
            .setFrom(from).setDestination(dest).calc().getResult();
        if (path == null) return true;

        for (EntityID step : path) {
            Double prob = belief.roadBlockedProb.get(step);
            if (prob != null && prob > 0.75) return true;
        }
        return false;
    }

    private EntityID pickBestVictim(Belief belief) {
        EntityID best = null;
        double bestScore = -1.0;

        for (Map.Entry<EntityID, Belief.VictimBelief> e
                : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            if (vb.pAlive() < 0.01) continue;

            // Пропускаем жертв, уже находящихся в убежище - они в безопасности
            // Иначе санитар войдёт в цикл LOAD→UNLOAD при встрече жертвы в убежище
            EntityID victimPos = getEntityPosition(e.getKey());
            if (victimPos != null && worldInfo.getEntity(victimPos) instanceof Refuge) continue;

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
            Belief.VictimBelief vb = belief.victims.get(victimId);
            // В данной версии RCRS-сервера AKRescue разрешён только FireBrigade
            // AmbulanceTeam может только LOAD жертву с buriedness=0
            // Если жертва ещё погребена - ждём рядом, пока FireBrigade не освободит
            if (vb != null && vb.likelyBuried) return AgentAction.rest();
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
     * Строит действие для доставки жертвы в убежище
     * Выбирает ближайшее (по длине пути) доступное убежище
     * Если агент застрял (позиция не меняется N тиков), текущее убежище
     * помечается как недоступное и выбирается следующее ближайшее
     */
    private AgentAction buildUnloadAction(Belief belief) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return AgentAction.rest();

        // После LOAD TrafficSimulator может временно выставить позицию агента
        // в его собственный EntityID. Pathfinding в таком случае выдаёт невалидный путь
        StandardEntity posEntity = worldInfo.getEntity(pos);
        if (!(posEntity instanceof Area)) {
            return AgentAction.unload();
        }

        // Уже в убежище
        if (posEntity instanceof Refuge) {
            return AgentAction.unload();
        }

        // Перебираем все убежища из worldInfo, выбираем ближайшее доступное
        EntityID bestRefuge = null;
        List<EntityID> bestPath = null;
        int bestLen = Integer.MAX_VALUE;

        for (EntityID refugeId : worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE)) {
            // Уже здесь - выгружаем немедленно
            if (pos.equals(refugeId)) return AgentAction.unload();

            // Пропускаем убежища, к которым уже пытались добраться и застряли
            if (failedRefuges.contains(refugeId)) continue;

            // Пропускаем заполненные убежища (если знаем из belief)
            Belief.RefugeState state = belief.knownRefuges.get(refugeId);
            if (state == Belief.RefugeState.FULL) continue;

            List<EntityID> path = pathPlanning
                .setFrom(pos).setDestination(refugeId).calc().getResult();
            if (path != null && !path.isEmpty() && path.size() < bestLen) {
                bestLen = path.size();
                bestRefuge = refugeId;
                bestPath = path;
            }
        }

        if (bestRefuge != null) {
            currentRefugeTarget = bestRefuge;
            return AgentAction.move(bestPath);
        }

        // Все убежища недоступны или помечены как failed - снимаем ограничения и пробуем снова
        if (!failedRefuges.isEmpty()) {
            failedRefuges.clear();
            currentRefugeTarget = null;
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

    // Возвращает EntityID позиции сущности
    private EntityID getEntityPosition(EntityID entityId) {
        StandardEntity entity = worldInfo.getEntity(entityId);
        if (entity instanceof Human) {
            EntityID pos = ((Human) entity).getPosition();
            if (pos == null) return null;
            
            // Проверяем что позиция - это зона карты, а не другой агент
            StandardEntity posEntity = worldInfo.getEntity(pos);
            if (posEntity instanceof rescuecore2.standard.entities.Area) {
                return pos;
            }
            return null; // жертва внутри транспортного средства
        }
        return null;
    }

    public AgentAction getSelectedAction()  { return selectedAction; }

    public AgentMacroState getMacroState()  { return macroState; }
    
    public EntityID getTargetVictimId()     { return targetVictimId; }
}