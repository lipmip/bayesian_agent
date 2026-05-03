package bayesian_agent.module.pomcp;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.observation.Observation;
import bayesian_agent.module.policy.AgentAction;
import bayesian_agent.util.Logger;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

// POMCP: M = <S, A, T, Z, O, R, γ, H, b₀, BN>
//
// H=20, N=500, C=100, γ=0.95
// Rollout-политика: StateMachineController (FSM)
// Переиспользование поддерева между тиками
public class POMCPPlanner {

    private static final int    H     = 20;
    private static final int    N     = 500;
    private static final double C     = 1.5;   // для нормализованных Q ∈ [0,1]
    private static final double GAMMA = 0.95;

    private final AgentInfo              agentInfo;
    private final DynamicBayesianNetwork dbn;
    private final ParticleFilter         pf;
    private final RewardFunction         rewardFn;
    private final StateMachineController sm;

    private POMCPNode        root           = new POMCPNode();
    private AgentAction      lastAction     = null;
    private AgentAction.Type lastBestType   = null;
    private boolean          lastWasForced  = false;
    private int              emptyBeliefTicks = 0;
    private static final int EMPTY_BELIEF_RESET = 3;  // сбросить дерево после N тиков без жертв

    public POMCPPlanner(AgentInfo ai, WorldInfo wi, PathPlanning pp) {
        this.agentInfo = ai;
        this.dbn       = new DynamicBayesianNetwork();
        this.pf        = new ParticleFilter(wi, ai);
        this.rewardFn  = new RewardFunction();
        this.sm        = new StateMachineController(ai, wi, pp);
    }

    // Вызывается каждый тик из TacticsAmbulanceTeam
    public AgentMacroState        getCurrentState()    { return sm.getCurrentState(); }
    public EntityID               getLastBlockedRoad() { return sm.getLastBlockedRoad(); }
    public EntityID               getTargetVictimId()  { return sm.getTargetVictimId(); }
    public StateMachineController getStateMachine()    { return sm; }

    public AgentAction selectAction(Belief belief, Observation obs) {
        long t0 = System.currentTimeMillis();

        boolean treeReused;
        if (lastAction != null) {
            pf.update(lastAction, obs, belief);

            // Переиспользуем поддерево по типу, который POMCP выбрал (не обязательно тому, что FSM исполнил)
            // Частичный фильтр обновляется реальным действием (lastAction), дерево - намеренным (lastBestType)
            AgentAction.Type advanceType = lastBestType != null ? lastBestType : lastAction.type;
            // REST не продвигает агента - не переносить его отравленное поддерево
            if (advanceType == AgentAction.Type.REST) {
                root = new POMCPNode();
                treeReused = false;
            } else {
                POMCPNode advanced = advanceTree(advanceType, obs);
                treeReused = advanced.getTotalVisits() > 0;
                root = advanced;
            }
        } else {
            pf.initFromBelief(belief);
            root = new POMCPNode();
            treeReused = false;
        }

        // Сброс дерева при пустом belief - предотвращает Q-freeze на REST
        if (belief.victims.isEmpty()) {
            emptyBeliefTicks++;
            if (emptyBeliefTicks >= EMPTY_BELIEF_RESET) {
                root = new POMCPNode();
                Logger.info(agentInfo, "[POMCP] tree reset: belief empty for " + emptyBeliefTicks + " ticks");
                emptyBeliefTicks = 0;
            }
        } else {
            emptyBeliefTicks = 0;
        }

        List<AgentAction.Type> available = availableActions(belief);

        for (int i = 0; i < N; i++) {
            SimState s = pf.sample();
            simulate(root, s, H, available);
        }

        AgentAction.Type bestType  = root.bestAction(available);
        String           qSummary  = root.getQSummary(available);
        AgentAction      action     = buildAction(bestType, belief);
        long             dt         = System.currentTimeMillis() - t0;

        Logger.info(agentInfo, "[POMCP] t=" + agentInfo.getTime()
            + " Q=" + qSummary
            + " best=" + bestType
            + " state=" + sm.getCurrentState()
            + " action=" + action.type
            + (lastWasForced ? " OVERRIDE" : "")
            + " reused=" + treeReused
            + " target=" + sm.getTargetVictimId()
            + " dt=" + dt + "ms");

        lastAction   = action;
        lastBestType = bestType;
        
        return action;
    }

    private double simulate(POMCPNode node, SimState s, int depth,
                            List<AgentAction.Type> available) {
        if (depth == 0 || terminal(s)) return 0.0;

        AgentAction.Type at = node.selectUCB(C, available);
        double total;

        if (node.isLeaf()) {
            // Первый визит: rollout без рекурсии в дерево, но обновляем узел
            total = rollout(s, depth - 1);
        } else {
            AgentAction        a      = simAction(at, s);
            SimState           next   = dbn.transition(s, a);
            ObservationSummary obsKey = synthObs(next);
            double             reward = rewardFn.compute(s, a, next);
            POMCPNode          child  = node.getOrCreate(at, obsKey);
            total = reward + GAMMA * simulate(child, next, depth - 1, available);
        }

        node.update(at, total);  // обновляем всегда: и при rollout, и при раскрытии
        return total;
    }

    // Rollout жадной политикой FSM 
    private double rollout(SimState s, int depth) {
        double total = 0, disc = 1; SimState cur = s.deepCopy();
        for (int d = 0; d < depth; d++) {
            AgentAction.Type at   = rolloutPolicy(cur);
            AgentAction      a    = simAction(at, cur);
            SimState         next = dbn.transition(cur, a);
            total += disc * rewardFn.compute(cur, a, next);
            disc  *= GAMMA; cur = next;
            if (terminal(cur)) break;
        }
        return total;
    }

    private AgentAction.Type rolloutPolicy(SimState s) {
        if (s.carryingVictim != null) return AgentAction.Type.UNLOAD;
        // LOAD только если агент уже рядом с незасыпанной живой жертвой
        for (Map.Entry<EntityID, Integer> e : s.victimHP.entrySet()) {
            if (e.getValue() <= 0 || s.isBuried(e.getKey())) continue;
            EntityID victimArea = s.victimPosition.get(e.getKey());
            if (victimArea == null || victimArea.equals(s.agentPosition))
                return AgentAction.Type.LOAD;
        }
        // Иначе двигаться к живой жертве
        for (Map.Entry<EntityID, Integer> e : s.victimHP.entrySet()) {
            if (e.getValue() > 0) return AgentAction.Type.MOVE;
        }
        return AgentAction.Type.REST;
    }

    private boolean terminal(SimState s) {
        // Пустой поток → allMatch = true (vacuously), но в режиме EXPLORE жертв нет →
        // это не терминальное состояние, продолжаем разведку
        return !s.victimHP.isEmpty() && s.victimHP.values().stream().allMatch(hp -> hp <= 0);
    }

    private POMCPNode advanceTree(AgentAction.Type actionType, Observation obs) {
        if (actionType == null) return new POMCPNode();
        // Грубый ключ: bool(жертвы видны) + bool(несём) - стабилен между тиками
        ObservationSummary os = new ObservationSummary(
            obs.victims.isEmpty() ? 0 : 1,
            0,
            agentInfo.someoneOnBoard() != null);
        return root.getOrCreate(actionType, os);
    }

    private List<AgentAction.Type> availableActions(Belief belief) {
        List<AgentAction.Type> a = new ArrayList<>(
            List.of(AgentAction.Type.MOVE, AgentAction.Type.REST));
        if (agentInfo.someoneOnBoard() != null)
            a.add(AgentAction.Type.UNLOAD);
        else if (!belief.victims.isEmpty())
            a.add(AgentAction.Type.LOAD);
        return a;
    }

    // Построить реальное действие
    // POMCP выбирает тип - FSM строит конкретный путь/цель
    // Переопределения: MOVE vs REST → форсируем навигацию; LOAD vs RESCUE → пробуем LOAD
    private AgentAction buildAction(AgentAction.Type type, Belief belief) {
        lastWasForced = false;
        if (type == AgentAction.Type.UNLOAD) return AgentAction.unload();

        AgentAction fsmAction = sm.tick(belief);

        if (type == AgentAction.Type.REST) {
            // REST принимается только когда FSM тоже заблокирован - иначе FSM знает лучше.
            // Без этого детерминированный PF замораживает Q на REST и агент стоит вечно.
            if (sm.getCurrentState() != AgentMacroState.WAIT_FOR_POLICE) {
                lastWasForced = true;
                return fsmAction;
            }
            return AgentAction.rest();
        }

        // POMCP → MOVE, FSM → REST (застрял в WAIT_FOR_POLICE): форсируем навигацию
        if (type == AgentAction.Type.MOVE && fsmAction.type == AgentAction.Type.REST) {
            lastWasForced = true;
            return sm.buildForceNavigate(belief);
        }

        return fsmAction;
    }

    // Действие для симуляции внутри дерева (без PathPlanning)
    // MOVE телепортирует агента к цели (path[0] = victimId) - 1-шаговая модель
    // LOAD ограничен жертвами на текущей позиции агента
    private AgentAction simAction(AgentAction.Type type, SimState s) {
        return switch (type) {
            case UNLOAD -> AgentAction.unload();
            case MOVE -> {
                // Целевая жертва: живая незасыпанная с известной позицией, не там где агент сейчас
                EntityID target = s.victimHP.entrySet().stream()
                    .filter(e -> e.getValue() > 0 && !s.isBuried(e.getKey()))
                    .filter(e -> {
                        EntityID vArea = s.victimPosition.get(e.getKey());
                        return vArea != null && !vArea.equals(s.agentPosition);
                    })
                    .min(Comparator.comparingInt(Map.Entry::getValue))
                    .map(Map.Entry::getKey).orElse(null);
                if (target == null)  // позиции не известны или все рядом - любая живая
                    target = s.victimHP.entrySet().stream()
                        .filter(e -> e.getValue() > 0)
                        .min(Comparator.comparingInt(Map.Entry::getValue))
                        .map(Map.Entry::getKey).orElse(null);
                yield target != null
                    ? AgentAction.move(Collections.singletonList(target))
                    : AgentAction.move(Collections.emptyList());
            }
            case LOAD -> {
                // Только жертвы на текущей позиции агента; если позиций нет - разрешаем всё
                EntityID v = s.victimHP.entrySet().stream()
                    .filter(e -> e.getValue() > 0 && !s.isBuried(e.getKey()))
                    .filter(e -> {
                        if (s.victimPosition.isEmpty()) return true;
                        EntityID vArea = s.victimPosition.get(e.getKey());
                        return vArea != null && vArea.equals(s.agentPosition);
                    })
                    .min(Comparator.comparingInt(Map.Entry::getValue))
                    .map(Map.Entry::getKey).orElse(null);
                yield v != null ? AgentAction.load(v) : AgentAction.rest();
            }
            case RESCUE -> {
                // Погребённая жертва → rescue; иначе любая живая (нейтральный шаг, не -5 как REST)
                EntityID b = s.victimBuriedness.entrySet().stream()
                    .filter(e -> e.getValue() > 0 && s.victimHP.getOrDefault(e.getKey(), 0) > 0)
                    .map(Map.Entry::getKey).findFirst().orElse(null);
                if (b != null) yield AgentAction.rescue(b);
                EntityID v = s.victimHP.entrySet().stream()
                    .filter(e -> e.getValue() > 0)
                    .map(Map.Entry::getKey).findFirst().orElse(null);
                yield v != null ? AgentAction.rescue(v) : AgentAction.rest();
            }
            default -> AgentAction.rest();
        };
    }

    private ObservationSummary synthObs(SimState s) {
        boolean anyAlive = s.victimHP.values().stream().anyMatch(hp -> hp > 0);
        return new ObservationSummary(anyAlive ? 1 : 0, 0, s.carryingVictim != null);
    }
}
