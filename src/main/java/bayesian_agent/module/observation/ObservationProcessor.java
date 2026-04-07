package bayesian_agent.module.observation;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Civilian;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
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
        // agentInfo.someoneOnBoard() возвращает Human если агент везёт жертву,
        // null если нет. Работает только для AmbulanceTeam — для остальных
        // типов агентов метод также безопасен (вернёт null).
        return agentInfo.someoneOnBoard() != null;
    }

    private void processCivilian(Civilian civ) {
        int hp = civ.isHPDefined() ? civ.getHP() : 10000;
        currentObservation.victims.put(civ.getID(),
                                       Observation.VictimStatus.fromHP(hp));

        // НОВОЕ: читаем buriedness
        int buried = civ.isBuriednessDefined() ? civ.getBuriedness() : 0;
        currentObservation.buriedStatus.put(civ.getID(),
                                            Observation.BuriedStatus.from(buried));
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
            Refuge refuge = (Refuge) building;
            currentObservation.visibleRefuges.add(id);

            Observation.RefugeCapacity cap = Observation.RefugeCapacity.UNKNOWN;
            if (refuge.isBedCapacityDefined() && refuge.isOccupiedBedsDefined()) {
                cap = refuge.getOccupiedBeds() >= refuge.getBedCapacity()
                        ? Observation.RefugeCapacity.FULL
                        : Observation.RefugeCapacity.AVAILABLE;
            }
            currentObservation.refugeCapacity.put(id, cap);
        }        
        if (building.isFierynessDefined() && building.getFieryness() > 0) {
            currentObservation.burningBuildings.add(id);
            currentObservation.fireIntensity.put(id,
                    Observation.FireIntensity.from(building.getFieryness()));
        }
        // Здание видно и не горит — значит потушено или никогда не горело
        if (building.isFierynessDefined() && building.getFieryness() == 0) {
            currentObservation.extinguishedBuildings.add(id);
        }
    }

    public Observation getObservation() {
        return currentObservation;
    }
}