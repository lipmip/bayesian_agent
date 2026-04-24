package bayesian_agent.module.communication;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.communication.standard.bundle.information.MessageRoad;
import adf.core.component.communication.CommunicationMessage;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Road;
import rescuecore2.worldmodel.EntityID;

import java.util.ArrayList;
import java.util.List;

public class CommunicationManager {

    // Ambulance → broadcast: эта дорога заблокирована, нужна расчистка 
    public void sendRoadBlocked(MessageManager msg, Road road, Blockade blockade) {
        msg.addMessage(new MessageRoad(true, road, blockade, false, true));
    }

    // PoliceForce → broadcast: эта дорога расчищена 
    public void sendRoadCleared(MessageManager msg, Road road) {
        msg.addMessage(new MessageRoad(true, road, null, true, false));
    }

    // PoliceOffice → broadcast: расчисти эту дорогу (MessageRoad т.к. CommandPolice не доставляется полевым агентам)
    public void sendPriorityRoad(MessageManager msg, Road road) {
        msg.addMessage(new MessageRoad(true, road, null, false, true));
    }

    // Принять сообщения о заблокированных дорогах (passable=false) 
    public List<EntityID> receiveBlockedRoads(MessageManager msg) {
        List<EntityID> result = new ArrayList<>();
        for (CommunicationMessage cm : msg.getReceivedMessageList(MessageRoad.class)) {
            MessageRoad m = (MessageRoad) cm;
            if (Boolean.FALSE.equals(m.isPassable()) && m.getRoadID() != null)
                result.add(m.getRoadID());
        }
        return result;
    }

    // Принять сообщения о расчищенных дорогах (passable=true) 
    public List<EntityID> receiveClearedRoads(MessageManager msg) {
        List<EntityID> result = new ArrayList<>();
        for (CommunicationMessage cm : msg.getReceivedMessageList(MessageRoad.class)) {
            MessageRoad m = (MessageRoad) cm;
            if (Boolean.TRUE.equals(m.isPassable()) && m.getRoadID() != null)
                result.add(m.getRoadID());
        }
        return result;
    }

    // Принять команды на расчистку (CommandPolice с ACTION_CLEAR) 
    public List<EntityID> receivePriorityRoads(MessageManager msg) {
        List<EntityID> result = new ArrayList<>();
        for (CommunicationMessage cm : msg.getReceivedMessageList(CommandPolice.class)) {
            CommandPolice c = (CommandPolice) cm;
            if (c.getAction() == CommandPolice.ACTION_CLEAR && c.getTargetID() != null)
                result.add(c.getTargetID());
        }
        return result;
    }
}
