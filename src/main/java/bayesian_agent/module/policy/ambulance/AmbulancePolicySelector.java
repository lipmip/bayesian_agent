package bayesian_agent.module.policy.ambulance;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import bayesian_agent.module.belief.Belief;
import bayesian_agent.module.policy.AgentAction;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

/**
 * Выбор действия для агента-санитара: bₜ → aₜ.
 * [ЗАМЕНИТЬ: POMCP]
 */
public class AmbulancePolicySelector {

    private final AgentInfo agentInfo;
    private final WorldInfo worldInfo;
    private AgentAction selectedAction = AgentAction.rest();

    public AmbulancePolicySelector(AgentInfo agentInfo, WorldInfo worldInfo) {
        this.agentInfo = agentInfo;
        this.worldInfo = worldInfo;
    }

    public void select(Belief belief) {
        // Случай 1: везём жертву — едем в убежище или выгружаем
        if (agentInfo.someoneOnBoard() != null) {
            selectedAction = buildUnloadAction(belief);
            return;
        }

        // Случай 2: есть известные живые жертвы
        EntityID best = pickBestVictim(belief);
        if (best != null) {
            selectedAction = buildRescueOrMoveAction(best, belief);
            return;
        }

        // Случай 3: нечего делать
        // TODO (Этап 2): Search — обход непосещённых зданий
        selectedAction = AgentAction.rest();
    }

    private EntityID pickBestVictim(Belief belief) {
        EntityID critical = null, injured = null;
        for (Map.Entry<EntityID, Belief.VictimBelief> e : belief.victims.entrySet()) {
            Belief.VictimBelief vb = e.getValue();
            if (vb.pAlive() < 0.01) continue;
            if (vb.pCritical > 0.5 && critical == null) critical = e.getKey();
            else if (vb.pInjured > 0.5 && injured == null) injured = e.getKey();
        }
        return critical != null ? critical : injured;
    }

    private AgentAction buildRescueOrMoveAction(EntityID victimId, Belief belief) {
        EntityID agentPos  = agentInfo.getPosition();
        EntityID victimPos = getEntityPosition(victimId);

        if (agentPos != null && agentPos.equals(victimPos)) {
            Belief.VictimBelief vb = belief.victims.get(victimId);
            if (vb != null && vb.likelyBuried) return AgentAction.rescue(victimId);
            return AgentAction.load(victimId);
        }

        List<EntityID> path = new ArrayList<>();
        if (victimPos != null) path.add(victimPos);
        return AgentAction.move(path);
    }

    /**
     * Строит действие для доставки жертвы в убежище.
     * Учитывает ёмкость убежища — не едет в полное.
     */
    private AgentAction buildUnloadAction(Belief belief) {
        EntityID pos = agentInfo.getPosition();

        // Уже стоим в доступном убежище — выгружаем
        if (pos != null && belief.knownRefuges.containsKey(pos)
                && belief.knownRefuges.get(pos) != Belief.RefugeState.FULL) {
            return AgentAction.unload();
        }

        // Едем к ближайшему доступному убежищу
        for (Map.Entry<EntityID, Belief.RefugeState> e
                : belief.knownRefuges.entrySet()) {
            if (e.getValue() != Belief.RefugeState.FULL) {
                return AgentAction.move(
                        Collections.singletonList(e.getKey()));
            }
        }

        // Убежищ не знаем — стоим
        // TODO (Этап 2): Search для обнаружения убежища
        return AgentAction.rest();
    }

    /**
     * Возвращает EntityID позиции сущности.
     * Human.getPosition() возвращает EntityID напрямую.
     */
    private EntityID getEntityPosition(EntityID entityId) {
        StandardEntity entity = worldInfo.getEntity(entityId);
        if (entity instanceof Human) {
            return ((Human) entity).getPosition();
        }
        return null;
    }

    public AgentAction getSelectedAction() { return selectedAction; }
}