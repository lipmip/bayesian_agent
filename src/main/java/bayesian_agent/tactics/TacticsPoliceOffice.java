package bayesian_agent.tactics;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import bayesian_agent.module.communication.CommunicationManager;
import bayesian_agent.util.Logger;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;

public class TacticsPoliceOffice
        extends adf.core.component.tactics.TacticsPoliceOffice {

    private final CommunicationManager commManager = new CommunicationManager();

    @Override
    public void initialize(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                           ModuleManager mm, MessageManager msg, DevelopData dd) {}

    @Override
    public void resume(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                       ModuleManager mm, PrecomputeData pd, DevelopData dd) {}

    @Override
    public void preparate(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                          ModuleManager mm, DevelopData dd) {}

    @Override
    public void think(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                      ModuleManager mm, MessageManager msg, DevelopData dd) {
        for (EntityID blockedRoad : commManager.receiveBlockedRoads(msg)) {
            for (EntityID pf : wi.getEntityIDsOfType(StandardEntityURN.POLICE_FORCE)) {
                commManager.sendPriorityRoad(msg, pf, blockedRoad);
            }
            Logger.info(ai, "[COMM] dispatched priority road=" + blockedRoad);
        }
    }
}
