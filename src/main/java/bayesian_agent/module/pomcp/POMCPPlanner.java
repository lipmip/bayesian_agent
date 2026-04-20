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

    private POMCPNode        root          = new POMCPNode();
    private AgentAction      lastAction    = null;
    private AgentAction.Type lastBestType  = null;  // тип, выбранный POMCP (для advancement дерева)
    private boolean          lastWasForced = false;

    public POMCPPlanner(AgentInfo ai, WorldInfo wi, PathPlanning pp) {
        this.agentInfo = ai;
        this.dbn       = new DynamicBayesianNetwork();
        this.pf        = new ParticleFilter(wi, ai);
        this.rewardFn  = new RewardFunction();
        this.sm        = new StateMachineController(ai, wi, pp);
    }

    // Вызывается каждый тик из TacticsAmbulanceTeam
    public AgentAction selectAction(Belief belief, Observation obs) {
        long t0 = System.currentTimeMillis();

        boolean treeReused;
        if (lastAction != null) {
            pf.update(lastAction, obs);

            // Переиспользуем поддерево по типу, который POMCP выбрал (не обязательно тому, что FSM исполнил)
            // Частичный фильтр обновляется реальным действием (lastAction), дерево - намеренным (lastBestType)
            AgentAction.Type advanceType = lastBestType != null ? lastBestType : lastAction.type;
            POMCPNode advanced = advanceTree(advanceType, obs);
            treeReused = advanced.getTotalVisits() > 0;
            root = advanced;
        } else {
            pf.initFromBelief(belief);
            root = new POMCPNode();
            treeReused = false;
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
        for (Map.Entry<EntityID, Integer> e : s.victimHP.entrySet()) {
            if (e.getValue() > 0 && s.isBuried(e.getKey())) return AgentAction.Type.RESCUE;
            if (e.getValue() > 0 && e.getValue() <= 2999)   return AgentAction.Type.LOAD;
        }
        return AgentAction.Type.MOVE;
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
        else if (!belief.victims.isEmpty()) {
            a.add(AgentAction.Type.LOAD);
            a.add(AgentAction.Type.RESCUE);
        }
        return a;
    }

    // Построить реальное действие
    // POMCP выбирает тип - FSM строит конкретный путь/цель
    // Если POMCP решил MOVE, но FSM застрял в WAIT_FOR_POLICE → форсируем навигацию
    private AgentAction buildAction(AgentAction.Type type, Belief belief) {
        lastWasForced = false;
        if (type == AgentAction.Type.UNLOAD) return AgentAction.unload();
        if (type == AgentAction.Type.REST)   return AgentAction.rest();
        AgentAction fsmAction = sm.tick(belief);
        if (type == AgentAction.Type.MOVE && fsmAction.type == AgentAction.Type.REST) {
            lastWasForced = true;
            return sm.buildForceNavigate(belief);
        }
        return fsmAction;
    }

    // Действие для симуляции внутри дерева (без PathPlanning)
    // MOVE и REST различаются по наградe: MOVE=-1, REST=-5
    private AgentAction simAction(AgentAction.Type type, SimState s) {
        return switch (type) {
            case UNLOAD -> AgentAction.unload();
            case MOVE   -> AgentAction.move(Collections.emptyList());   // нет пути, но тип MOVE
            case LOAD   -> {
                EntityID v = s.victimHP.entrySet().stream()
                    .filter(e -> e.getValue() > 0)
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
