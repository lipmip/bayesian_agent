package bayesian_agent.module.policy.fire;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.component.module.algorithm.PathPlanning;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;
import java.util.*;


public class FireBrigadePolicySelector {

    private final AgentInfo    agentInfo;
    private final WorldInfo    worldInfo;
    private final PathPlanning pathPlanning;
    private AgentAction selectedAction = AgentAction.rest();
    private final Map<EntityID, Integer> lastVisitedTick = new HashMap<>();
    // Жертвы, полученные через коммуникацию: victimId → positionAreaId
    private final Map<EntityID, EntityID> communicatedVictims = new HashMap<>();

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
        // Приоритет: тушить огонь рядом → идти к огню → откапывать жертв (AKRescue) → разведка

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

        // Нет огня - откапываем засыпанных жертв (AKRescue принимается только от FireBrigade)
        EntityID buriedNearby = findBuriedCivilianInRange();
        if (buriedNearby != null) {
            selectedAction = AgentAction.rescue(buriedNearby);
            return;
        }

        EntityID buriedKnown = findNearestBuriedCivilian();
        if (buriedKnown != null) {
            selectedAction = buildMoveTo(buriedKnown);
            return;
        }

        selectedAction = buildSearchMove();
    }

    private EntityID findBuriedCivilianInRange() {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return null;
        for (StandardEntity e : worldInfo.getEntitiesOfType(StandardEntityURN.CIVILIAN)) {
            Civilian civ = (Civilian) e;
            if (!civ.isBuriednessDefined() || civ.getBuriedness() <= 0) continue;
            if (!civ.isPositionDefined()) continue;
            if (civ.isHPDefined() && civ.getHP() <= 0) continue;
            if (pos.equals(civ.getPosition())) return civ.getID();
        }
        return null;
    }

    private EntityID findNearestBuriedCivilian() {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return null;
        StandardEntity posEntity = worldInfo.getEntity(pos);
        if (!(posEntity instanceof Area)) return null;
        int ax = ((Area) posEntity).getX(), ay = ((Area) posEntity).getY();

        EntityID best = null;
        double bestDist = Double.MAX_VALUE;

        // Жертвы из worldInfo
        for (StandardEntity e : worldInfo.getEntitiesOfType(StandardEntityURN.CIVILIAN)) {
            Civilian civ = (Civilian) e;
            if (!civ.isBuriednessDefined() || civ.getBuriedness() <= 0) continue;
            if (!civ.isPositionDefined()) continue;
            if (civ.isHPDefined() && civ.getHP() <= 0) continue;
            StandardEntity civPos = worldInfo.getEntity(civ.getPosition());
            if (!(civPos instanceof Area)) continue;
            double d = Math.hypot(((Area) civPos).getX() - ax, ((Area) civPos).getY() - ay);
            if (d < bestDist) { bestDist = d; best = civ.getID(); }
        }

        // Жертвы из коммуникации (ещё не наблюдались напрямую)
        for (Map.Entry<EntityID, EntityID> entry : communicatedVictims.entrySet()) {
            if (best != null && entry.getKey().equals(best)) continue;  // уже в worldInfo
            StandardEntity civPos = worldInfo.getEntity(entry.getValue());
            if (!(civPos instanceof Area)) continue;
            double d = Math.hypot(((Area) civPos).getX() - ax, ((Area) civPos).getY() - ay);
            if (d < bestDist) { bestDist = d; best = entry.getKey(); }
        }

        return best;
    }

    private AgentAction buildMoveTo(EntityID targetCivilian) {
        EntityID pos = agentInfo.getPosition();
        if (pos == null) return AgentAction.rest();
        Civilian civ = (Civilian) worldInfo.getEntity(targetCivilian);
        if (civ == null || !civ.isPositionDefined()) return AgentAction.rest();
        EntityID dest = civ.getPosition();
        List<EntityID> path = pathPlanning.setFrom(pos).setDestination(dest).calc().getResult();
        if (path != null && !path.isEmpty()) return AgentAction.move(path);
        return AgentAction.rest();
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

        int t = agentInfo.getTime();
        lastVisitedTick.put(pos, t);

        rescuecore2.standard.entities.StandardEntity e = worldInfo.getEntity(pos);
        if (!(e instanceof rescuecore2.standard.entities.Area)) return AgentAction.rest();

        List<EntityID> neighbours = new ArrayList<>(
            ((rescuecore2.standard.entities.Area) e).getNeighbours());
        if (neighbours.isEmpty()) return AgentAction.rest();

        List<EntityID> unvisited = new ArrayList<>();
        List<EntityID> visited   = new ArrayList<>();
        for (EntityID nb : neighbours) {
            (lastVisitedTick.containsKey(nb) ? visited : unvisited).add(nb);
        }

        List<EntityID> candidates;
        if (!unvisited.isEmpty()) {
            unvisited.sort(Comparator.comparingInt(EntityID::getValue));
            int start = Math.abs(agentInfo.getID().getValue()) % unvisited.size();
            candidates = new ArrayList<>();
            for (int i = 0; i < unvisited.size(); i++)
                candidates.add(unvisited.get((start + i) % unvisited.size()));
        } else {
            visited.sort((a, b) -> Integer.compare(lastVisitedTick.get(a), lastVisitedTick.get(b)));
            candidates = visited;
        }

        for (EntityID next : candidates) {
            List<EntityID> path = pathPlanning
                .setFrom(pos).setDestination(next).calc().getResult();
            if (path != null && !path.isEmpty()) return AgentAction.move(path);
            lastVisitedTick.put(next, t);
        }
        return AgentAction.rest();
    }

    // Добавить жертву из коммуникации (ID + позиция). buriedness=0 → удалить из списка
    public void addCommunicatedVictim(EntityID victimId, EntityID positionId, int buriedness) {
        if (buriedness == 0) {
            communicatedVictims.remove(victimId);
        } else {
            communicatedVictims.put(victimId, positionId);
        }
    }

    public AgentAction getSelectedAction() { return selectedAction; }
}