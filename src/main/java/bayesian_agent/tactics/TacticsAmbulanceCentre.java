package bayesian_agent.tactics;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;

/**
 * Заглушка центра управления TacticsAmbulanceCentre.
 * TacticsCenter.think() возвращает void - центры не планируют действия,
 * а только координируют полевых агентов через CommandPicker.
 * Нет метода precompute() - только resume() и preparate().
 */
public class TacticsAmbulanceCentre
        extends adf.core.component.tactics.TacticsAmbulanceCentre {

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
                      ModuleManager mm, MessageManager msg, DevelopData dd) {}
}