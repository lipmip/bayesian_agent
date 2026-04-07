package bayesian_agent.module.observation;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;

public class ObservationProcessor {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;

    private Observation currentObservation = new Observation();

    public ObservationProcessor(AgentInfo agentInfo, WorldInfo worldInfo) {
        this.agentInfo = agentInfo;
        this.worldInfo = worldInfo;
    }

    public void process(ChangeSet changeSet) {
        currentObservation = new Observation();
        currentObservation.agentPosition = agentInfo.getPosition();
        currentObservation.isCarrying    = checkIsCarrying();

        for (EntityID id : changeSet.getChangedEntities()) {
            StandardEntity entity = worldInfo.getEntity(id);
            if (entity == null) continue;

            if      (entity instanceof Civilian)  processCivilian((Civilian)  entity);
            else if (entity instanceof Road)      processRoad((Road)          entity);
            else if (entity instanceof Building)  processBuilding((Building)  entity, id);
        }
    }

    private boolean checkIsCarrying() {
        StandardEntity me = agentInfo.me();
        if (me instanceof AmbulanceTeam) {
            AmbulanceTeam ambulance = (AmbulanceTeam) me;
            // TODO (Этап 0.2): уточнить реальное имя метода:
            //   grep -r "isLoaded\|someoneOnBoard\|loadedHuman" adf-core-java/src/
            // Вариант A: return ambulance.isLoaded();
            // Вариант B: return ambulance.getLoadedHuman() != null;
            return false; // временная заглушка
        }
        return false;
    }

    private void processCivilian(Civilian civ) {
        int hp = civ.isHPDefined() ? civ.getHP() : 10000;
        currentObservation.victims.put(civ.getID(),
                Observation.VictimStatus.fromHP(hp));
    }

    private void processRoad(Road road) {
        if (road.isBlockadesDefined() && !road.getBlockades().isEmpty()) {
            currentObservation.blockedRoads.add(road.getID());
            for (EntityID blockadeId : road.getBlockades()) {
                currentObservation.blockadeIds.add(blockadeId);
            }
        }
    }

    private void processBuilding(Building building, EntityID id) {
        if (building instanceof Refuge) {
            currentObservation.visibleRefuges.add(id);
        }
        if (building.isFierynessDefined() && building.getFieryness() > 0) {
            currentObservation.burningBuildings.add(id);
        }
    }

    public Observation getObservation() {
        return currentObservation;
    }
}