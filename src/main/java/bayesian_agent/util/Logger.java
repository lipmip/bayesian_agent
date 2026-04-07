package bayesian_agent.util;

import adf.core.agent.info.AgentInfo;

public class Logger {

    public static void info(AgentInfo ai, String msg) {
        System.err.printf("[%-12s %s] %s%n",
                agentLabel(ai), ai.getID(), msg);
    }

    public static void warn(AgentInfo ai, String msg) {
        System.err.printf("[%-12s %s] WARN: %s%n",
                agentLabel(ai), ai.getID(), msg);
    }

    private static String agentLabel(AgentInfo ai) {
        String cls = ai.me().getClass().getSimpleName();
        if (cls.contains("Ambulance"))  return "AMBULANCE";
        if (cls.contains("FireBrigade")) return "FIRE";
        if (cls.contains("Police"))     return "POLICE";
        return cls;
    }
}