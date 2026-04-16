package bayesian_agent.util;

import adf.core.agent.info.AgentInfo;

/**
 * Логирование агентов.
 *
 * Уровни:
 *   info()  - ключевые события (STATE, METRICS, BELIEF, FSM, BELIEF_SNAP)
 *   warn()  - аномалии (нет пути, все убежища недоступны)
 *   debug() - подробности (decay per-victim, per-tick observation detail)
 *             включается свойством -Dbayesian.debug=true
 *
 * Теги для grep/анализа:
 *   [STATE]        - текущее макросостояние FSM
 *   [FSM]          - переход между макросостояниями
 *   [METRICS]      - агрегированные метрики (каждые 50 тиков)
 *   [FIRE_METRICS] - метрики пожарного
 *   [POLICE_METRICS] - метрики полицейского
 *   [BELIEF]       - событие изменения состава belief (новая/удалённая жертва)
 *   [BELIEF_SNAP]  - компактный снимок убеждения для post-analysis (каждые 25 тиков)
 *   [DECAY]        - применение decay к жертве (только debug)
 */
public class Logger {

    /** Включить подробный режим: -Dbayesian.debug=true */
    public static final boolean DEBUG = Boolean.getBoolean("bayesian.debug");

    public static void info(AgentInfo ai, String msg) {
        System.err.printf("[%-12s %s] %s%n",
                agentLabel(ai), ai.getID(), msg);
    }

    public static void warn(AgentInfo ai, String msg) {
        System.err.printf("[%-12s %s] WARN: %s%n",
                agentLabel(ai), ai.getID(), msg);
    }

    /** Подробный лог - печатается только если bayesian.debug=true */
    public static void debug(AgentInfo ai, String msg) {
        if (!DEBUG) return;
        System.err.printf("[%-12s %s] DBG: %s%n",
                agentLabel(ai), ai.getID(), msg);
    }

    private static String agentLabel(AgentInfo ai) {
        String cls = ai.me().getClass().getSimpleName();
        if (cls.contains("Ambulance"))   return "AMBULANCE";
        if (cls.contains("FireBrigade")) return "FIRE";
        if (cls.contains("Police"))      return "POLICE";
        return cls;
    }
}