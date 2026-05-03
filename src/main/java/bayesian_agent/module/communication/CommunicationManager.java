package bayesian_agent.module.communication;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.communication.standard.bundle.information.MessageCivilian;
import adf.core.agent.communication.standard.bundle.information.MessageRoad;
import adf.core.component.communication.CommunicationMessage;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Civilian;
import rescuecore2.standard.entities.Road;
import rescuecore2.worldmodel.EntityID;

import java.util.ArrayList;
import java.util.List;

public class CommunicationManager {

    // Ambulance → broadcast: эта дорога заблокирована, нужна расчистка
    // isRadio=false: голосовой канал 0; радиоканал 1 агентам недоступен по умолчанию (max.platoon=1)
    public void sendRoadBlocked(MessageManager msg, Road road, Blockade blockade) {
        msg.addMessage(new MessageRoad(false, road, blockade, false, true));
    }

    // PoliceForce → broadcast: эта дорога расчищена
    public void sendRoadCleared(MessageManager msg, Road road) {
        msg.addMessage(new MessageRoad(false, road, null, true, false));
    }

    // PoliceOffice → broadcast: расчисти эту дорогу
    public void sendPriorityRoad(MessageManager msg, Road road) {
        msg.addMessage(new MessageRoad(false, road, null, false, true));
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

    // Broadcast: обнаружена засыпанная жертва ИЛИ жертва освобождена (buriedness=0)
    // isRadio=false: голосовой канал 0 (range=30000mm)
    public void sendVictimFound(MessageManager msg, Civilian civ) {
        msg.addMessage(new MessageCivilian(false, civ));
    }

    // Принять информацию о жертвах от других агентов
    public List<MessageCivilian> receiveVictimInfo(MessageManager msg) {
        List<MessageCivilian> result = new ArrayList<>();
        for (CommunicationMessage cm : msg.getReceivedMessageList(MessageCivilian.class)) {
            result.add((MessageCivilian) cm);
        }
        return result;
    }
}
