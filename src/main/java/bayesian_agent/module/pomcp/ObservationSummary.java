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

    // Грубый 4-значный ключ: V/N (любая жертва видна?) + C/E (несём жертву?)
    // Точные счётчики меняются каждый тик → дерево никогда не переиспользуется
    public String key() {
        return (visibleVictimCount > 0 ? "V" : "N") + (carrying ? "C" : "E");
    }
}
