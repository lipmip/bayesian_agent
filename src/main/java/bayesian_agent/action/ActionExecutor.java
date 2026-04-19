package bayesian_agent.action;

import adf.core.agent.action.Action;
import adf.core.agent.action.ambulance.ActionLoad;
import adf.core.agent.action.ambulance.ActionUnload;
import adf.core.agent.action.ambulance.ActionRescue;
import adf.core.agent.action.fire.ActionExtinguish;
import adf.core.agent.action.police.ActionClear;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.action.common.ActionRest;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;

import java.util.Collections;
import java.util.List;

// ШАГ 4 ТИКА: a_t → ADF Action
// В новом ADF think() возвращает Action напрямую - agentInfo.act() удалён.
// translate() публичный, вызывается из оркестратора через return
public class ActionExecutor {

    private final AgentInfo    agentInfo;
    private final WorldInfo    worldInfo;
    private final ScenarioInfo scenarioInfo;


    public ActionExecutor(AgentInfo agentInfo, WorldInfo worldInfo,
                          ScenarioInfo scenarioInfo) {
        this.agentInfo    = agentInfo;
        this.worldInfo    = worldInfo;
        this.scenarioInfo = scenarioInfo;
    }

    // Переводит внутреннее действие в ADF Action для возврата из think()
    public Action translate(AgentAction action) {
        switch (action.type) {

            case MOVE:
                List<EntityID> path = action.path;
                if (path == null || path.isEmpty()) return new ActionRest();
                return new ActionMove(path);

            case MOVE_TO_POINT:
                return new ActionMove(
                    Collections.singletonList(agentInfo.getPosition()),
                    action.destX, action.destY);

            case RESCUE:
                if (action.targetId == null) return new ActionRest();
                return new ActionRescue(action.targetId);

            case LOAD:
                if (action.targetId == null) return new ActionRest();
                return new ActionLoad(action.targetId);

            case UNLOAD:
                return new ActionUnload();

            case EXTINGUISH:
                if (action.targetId == null) return new ActionRest();
                return new ActionExtinguish(action.targetId,
                        scenarioInfo.getFireExtinguishMaxSum());

            case CLEAR:
                if (action.targetId == null) return new ActionRest();
                StandardEntity blockadeEntity = worldInfo.getEntity(action.targetId);
                if (blockadeEntity instanceof Blockade) {
                    Blockade blockade = (Blockade) blockadeEntity;
                    int agentX    = (int) agentInfo.getX();
                    int agentY    = (int) agentInfo.getY();
                    int clearDist = scenarioInfo.getClearRepairDistance();

                    // Цель AKClearArea = центроид завала (getX/getY = среднее оставшихся тайлов).
                    // Центроид обновляется симулятором по мере расчистки → всегда указывает
                    // на оставшийся кластер тайлов. Дальний-апекс подход давал clearing freeze:
                    // после сдвига агента на север дальний апекс оказывался в уже расчищенной зоне.
                    int targetX = blockade.getX();
                    int targetY = blockade.getY();
                    double distToCent = Math.hypot(targetX - agentX, targetY - agentY);
                    if (distToCent > clearDist && distToCent >= 1.0) {
                        double scale = (double) clearDist / distToCent;
                        targetX = agentX + (int)((targetX - agentX) * scale);
                        targetY = agentY + (int)((targetY - agentY) * scale);
                    }
                    return new ActionClear(targetX, targetY, blockade);
                }
                return new ActionRest();

            case REST:
            default:
                return new ActionRest();
        }
    }
}