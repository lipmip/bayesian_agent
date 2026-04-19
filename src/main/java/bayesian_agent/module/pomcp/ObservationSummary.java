package bayesian_agent.module.pomcp;

// Компактное резюме наблюдения для модели Z(s', o)
public class ObservationSummary {
    public final int visibleVictimCount;
    public final int visibleBlockedRoads;
    public final boolean carrying;

    public ObservationSummary(int victims, int blockedRoads, boolean carrying) {
        this.visibleVictimCount  = victims;
        this.visibleBlockedRoads = blockedRoads;
        this.carrying            = carrying;
    }

    public String key() {
        return visibleVictimCount + "v" + visibleBlockedRoads + "r" + (carrying ? "C" : "E");
    }
}
