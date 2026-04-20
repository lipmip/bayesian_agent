package bayesian_agent.module.pomcp;

import bayesian_agent.module.policy.AgentAction;
import java.util.*;
import java.util.Locale;

// Узел дерева POMCP
// UCB1: argmax Q(a) + C * sqrt(ln N / N(a))
// Инкрементальное обновление Q(a)
public class POMCPNode {

    private final Map<AgentAction.Type, Integer> visits   = new HashMap<>();
    private final Map<AgentAction.Type, Double>  qValue   = new HashMap<>();
    private final Map<String, POMCPNode>          children = new HashMap<>();
    private int totalVisits = 0;

    public AgentAction.Type selectUCB(double C, List<AgentAction.Type> actions) {
        // Нормализуем Q по диапазону посещённых действий перед UCB
        // Без нормализации: Q(LOAD)=418 >> C=100, и LOAD занимает 497/500 симуляций,
        // лишая RESCUE каких-либо визитов
        double qMin = Double.MAX_VALUE, qMax = -Double.MAX_VALUE;
        for (AgentAction.Type a : actions) {
            if (visits.getOrDefault(a, 0) > 0) {
                double q = qValue.getOrDefault(a, 0.0);
                if (q < qMin) qMin = q;
                if (q > qMax) qMax = q;
            }
        }
        if (qMin > qMax) { qMin = 0.0; qMax = 1.0; }
        double range = Math.max(qMax - qMin, 1.0);

        AgentAction.Type best = null; double bestVal = Double.NEGATIVE_INFINITY;
        for (AgentAction.Type a : actions) {
            int n = visits.getOrDefault(a, 0);
            double qNorm = (qValue.getOrDefault(a, 0.0) - qMin) / range;
            double ucb = (n == 0) ? Double.POSITIVE_INFINITY
                : qNorm + C * Math.sqrt(Math.log(totalVisits + 1.0) / n);
            if (ucb > bestVal) { bestVal = ucb; best = a; }
        }
        return best;
    }

    public void update(AgentAction.Type a, double reward) {
        totalVisits++;
        int n = visits.merge(a, 1, Integer::sum);
        qValue.merge(a, reward, (old, r) -> old + (r - old) / n);
    }

    public POMCPNode getOrCreate(AgentAction.Type a, ObservationSummary obs) {
        return children.computeIfAbsent(a.name() + "|" + obs.key(), k -> new POMCPNode());
    }

    public AgentAction.Type bestAction(List<AgentAction.Type> actions) {
        return actions.stream()
            .max(Comparator.comparingDouble(a -> qValue.getOrDefault(a, Double.NEGATIVE_INFINITY)))
            .orElse(AgentAction.Type.REST);
    }

    public boolean isLeaf() { return totalVisits == 0; }

    public int getTotalVisits() { return totalVisits; }

    // Компактная строка Q(a)(n) для лога: {MOVE:-1.2(n=120),LOAD:23.5(n=210),...} 
    public String getQSummary(List<AgentAction.Type> actions) {
        StringBuilder sb = new StringBuilder("{");
        for (AgentAction.Type a : actions) {
            if (sb.length() > 1) sb.append(",");
            int n = visits.getOrDefault(a, 0);
            double q = qValue.getOrDefault(a, 0.0);
            sb.append(a).append(":").append(String.format(Locale.US, "%.1f", q))
              .append("(n=").append(n).append(")");
        }
        return sb.append("}").toString();
    }
}
