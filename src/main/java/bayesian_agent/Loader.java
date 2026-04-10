package bayesian_agent;

import adf.core.component.AbstractLoader;
import adf.core.component.tactics.TacticsAmbulanceCentre;
import adf.core.component.tactics.TacticsAmbulanceTeam;
import adf.core.component.tactics.TacticsFireBrigade;
import adf.core.component.tactics.TacticsFireStation;
import adf.core.component.tactics.TacticsPoliceForce;
import adf.core.component.tactics.TacticsPoliceOffice;

/**
 * Загрузчик команды bayesian_agent.
 *
 * Передаётся первым аргументом в adf.core.Main:
 *   ./launch.sh -all
 * → Main вызывает: AgentLauncher("bayesian_agent.Loader", "-t", "-1,-1,-1,-1,-1,-1", ...)
 *
 * AbstractLoader связывает каждый тип агента с реализацией тактики.
 * Центры управления (Centre/Station/Office) используют заглушки -
 * их логика не влияет на результат симуляции в базовых сценариях.
 */
public class Loader extends AbstractLoader {

    @Override
    public TacticsAmbulanceTeam getTacticsAmbulanceTeam() {
        return new bayesian_agent.tactics.TacticsAmbulanceTeam();
    }

    @Override
    public TacticsFireBrigade getTacticsFireBrigade() {
        return new bayesian_agent.tactics.TacticsFireBrigade();
    }

    @Override
    public TacticsPoliceForce getTacticsPoliceForce() {
        return new bayesian_agent.tactics.TacticsPoliceForce();
    }

    @Override
    public TacticsAmbulanceCentre getTacticsAmbulanceCentre() {
        return new bayesian_agent.tactics.TacticsAmbulanceCentre();
    }

    @Override
    public TacticsFireStation getTacticsFireStation() {
        return new bayesian_agent.tactics.TacticsFireStation();
    }

    @Override
    public TacticsPoliceOffice getTacticsPoliceOffice() {
        return new bayesian_agent.tactics.TacticsPoliceOffice();
    }
}