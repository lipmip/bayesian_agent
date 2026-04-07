package bayesian_agent.module.policy;

import rescuecore2.worldmodel.EntityID;
import java.util.List;

/**
 * Логическое действие агента aₜ — внутреннее представление.
 *
 * Не является ADF Action напрямую.
 * ActionExecutor переводит его в конкретный ADF-объект.
 *
 * Поддерживает действия всех трёх типов агентов:
 *   - AmbulanceTeam:  MOVE, RESCUE, LOAD, UNLOAD
 *   - FireBrigade:    MOVE, EXTINGUISH
 *   - PoliceForce:    MOVE, CLEAR
 *   - Все:            REST
 */
public class AgentAction {

    public enum Type {
        MOVE,        // Двигаться по маршруту               [все]
        RESCUE,      // Извлечь погребённую жертву           [AmbulanceTeam]
        LOAD,        // Загрузить жертву в машину            [AmbulanceTeam]
        UNLOAD,      // Выгрузить жертву в убежище           [AmbulanceTeam]
        EXTINGUISH,  // Тушить горящее здание                [FireBrigade]
        CLEAR,       // Расчистить завал на дороге           [PoliceForce]
        REST         // Ничего не делать (заглушка / ожидание)
    }

    public final Type type;

    /** Для MOVE: список waypoints (EntityID дорог / зданий). */
    public final List<EntityID> path;

    /** Для RESCUE / LOAD / EXTINGUISH / CLEAR: EntityID цели. */
    public final EntityID targetId;

    // ── Фабричные методы ────────────────────────────────────────────

    public static AgentAction move(List<EntityID> path) {
        return new AgentAction(Type.MOVE, path, null);
    }

    public static AgentAction rescue(EntityID target) {
        return new AgentAction(Type.RESCUE, null, target);
    }

    public static AgentAction load(EntityID target) {
        return new AgentAction(Type.LOAD, null, target);
    }

    public static AgentAction unload() {
        return new AgentAction(Type.UNLOAD, null, null);
    }

    public static AgentAction extinguish(EntityID target) {
        return new AgentAction(Type.EXTINGUISH, null, target);
    }

    public static AgentAction clear(EntityID target) {
        return new AgentAction(Type.CLEAR, null, target);
    }

    public static AgentAction rest() {
        return new AgentAction(Type.REST, null, null);
    }

    // ── Конструктор ─────────────────────────────────────────────────

    private AgentAction(Type type, List<EntityID> path, EntityID targetId) {
        this.type     = type;
        this.path     = path;
        this.targetId = targetId;
    }

    @Override
    public String toString() {
        return "AgentAction{" + type
               + (targetId != null ? ", target=" + targetId : "")
               + (path     != null ? ", pathLen=" + path.size() : "")
               + "}";
    }
}