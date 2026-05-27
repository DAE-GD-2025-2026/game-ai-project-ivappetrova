#include "BTSearchAction.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

namespace GameAI::BT
{
	SearchAction::SearchAction(float ArrivalRadius, float SearchTimeout)
		: m_ArrivalRadius(ArrivalRadius), m_SearchTimeout(SearchTimeout)
	{
	}

	void SearchAction::OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		m_ArrivedAtLastKnown = false;
		Agent.SetSteeringBehavior(&m_Seek);

		if (Blackboard)
		{
			float worldTime = Agent.GetWorld() ? Agent.GetWorld()->GetTimeSeconds() : 0.f;
			Blackboard->SetValueAsFloat(BB_SEARCH_START_TIME, worldTime);
		}
	}

	ENodeStatus SearchAction::Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (!Blackboard) return ENodeStatus::Failed;

		float searchStart = Blackboard->GetValueAsFloat(BB_SEARCH_START_TIME);
		float now = Agent.GetWorld() ? Agent.GetWorld()->GetTimeSeconds() : 0.f;
		if (searchStart >= 0.f && (now - searchStart) >= m_SearchTimeout)
		{
			return ENodeStatus::Failed;
		}

		if (!m_ArrivedAtLastKnown)
		{
			FVector lastKnownVec = Blackboard->GetValueAsVector(BB_LAST_KNOWN);
			FVector2D lastKnown{ lastKnownVec.X, lastKnownVec.Y };

			FTargetData targetData{};
			targetData.Position = lastKnown;
			m_Seek.SetTarget(targetData);

			float distSq = FVector2D::DistSquared(Agent.GetPosition(), lastKnown);
			if (distSq < m_ArrivalRadius * m_ArrivalRadius)
			{
				m_ArrivedAtLastKnown = true;
				Agent.SetSteeringBehavior(&m_Wander);
			}
		}

		return ENodeStatus::Running;
	}

	void SearchAction::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(nullptr);
	}
}