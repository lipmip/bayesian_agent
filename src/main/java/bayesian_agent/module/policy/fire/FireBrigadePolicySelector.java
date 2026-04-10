package bayesian_agent.module.policy.fire;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;
import java.util.*;

/**
 * Выбор действия для агента-пожарного: b_t → a_t.
 * [ЗАМЕНИТЬ: POMCP]
 */
public class FireBrigadePolicySelector {

    private final AgentInfo    agentInfo;
    private final WorldInfo    worldInfo;
    private final PathPlanning pathPlanning;
    private AgentAction selectedAction = AgentAction.rest();

    public FireBrigadePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo,
                                   PathPlanning pathPlanning) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.pathPlanning = pathPlanning;
    }

    private EntityID findBurningInRange(Belief belief) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return null;

        StandardEntity posEntity = worldInfo.getEntity(pos);
        if (posEntity instanceof Area) {
            Area area = (Area) posEntity;
            for (EntityID neighbourId : area.getNeighbours()) {
                if (belief.burningBuildings.contains(neighbourId)) {
                    return neighbourId;
                }
            }
        }

        return null;
    }

    public void select(Belief belief) {
        EntityID nearby = findBurningInRange(belief);
        if (nearby != null) {
            selectedAction = AgentAction.extinguish(nearby);
            return;
        }

        EntityID known = pickBestBurningBuilding(belief);
        if (known != null) {
            selectedAction = buildMoveToFire(known);
            return;
        }

        selectedAction = buildSearchMove();
    }

    // Ищет горящее здание рядом с позицией агента
    private AgentAction buildMoveToFire(EntityID target) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return AgentAction.rest();

        List<EntityID> path = pathPlanning
            .setFrom(pos)
            .setDestination(target)
            .calc()
            .getResult();
        if (path != null && !path.isEmpty()) return AgentAction.move(path);

        return AgentAction.rest();
    }

    private EntityID pickBestBurningBuilding(Belief belief) {
        return belief.burningBuildings.isEmpty() ? null
                : belief.burningBuildings.iterator().next();
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

    public AgentAction getSelectedAction() { return selectedAction; }
}