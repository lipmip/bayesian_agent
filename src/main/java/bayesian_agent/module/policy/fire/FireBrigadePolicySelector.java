package bayesian_agent.module.policy.fire;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

/**
 * Выбор действия для агента-пожарного: bₜ → aₜ.
 * [ЗАМЕНИТЬ: POMCP]
 */
public class FireBrigadePolicySelector {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;
    private AgentAction selectedAction = AgentAction.rest();

    public FireBrigadePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo) {
        this.agentInfo = agentInfo;
        this.worldInfo = worldInfo;
    }

    public void select(Belief belief) {
        EntityID nearby = findBurningInRange(belief);
        if (nearby != null) {
            selectedAction = AgentAction.extinguish(nearby);
            return;
        }
        EntityID known = pickBestBurningBuilding(belief);
        if (known != null) {
            selectedAction = AgentAction.move(Collections.singletonList(known));
            return;
        }
        selectedAction = AgentAction.rest();
    }

    /**
     * Ищет горящее здание рядом с позицией агента.
     *
     * ИСПРАВЛЕНИЕ: убрана проверка isNeighboursDefined() — метода нет
     * в данной версии rescuecore2. Используем getNeighbours() напрямую
     * (возвращает пустой список если соседей нет).
     * Также агент стоит на Road, а не внутри Building — проверяем соседей.
     */
    private EntityID findBurningInRange(Belief belief) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return null;

        StandardEntity posEntity = worldInfo.getEntity(pos);
        if (posEntity instanceof Area) {
            Area area = (Area) posEntity;
            // getNeighbours() возвращает List<EntityID> соседних Area
            for (EntityID neighbourId : area.getNeighbours()) {
                if (belief.burningBuildings.contains(neighbourId)) {
                    return neighbourId;
                }
            }
        }
        return null;
    }

    private EntityID pickBestBurningBuilding(Belief belief) {
        return belief.burningBuildings.isEmpty() ? null
                : belief.burningBuildings.iterator().next();
    }

    public AgentAction getSelectedAction() { return selectedAction; }
}