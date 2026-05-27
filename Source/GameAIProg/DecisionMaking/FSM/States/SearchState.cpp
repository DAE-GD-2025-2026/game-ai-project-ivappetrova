#include "SearchState.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

namespace GameAI::FSM
{
	SearchState::SearchState(float ArrivalRadius)
		: m_ArrivalRadius(ArrivalRadius)
	{
	}

	void SearchState::OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		m_bArrivedAtLastKnown = false;

		// Record the time we started searching
		if (Blackboard)
		{
			float WorldTime = Agent.GetWorld() ? Agent.GetWorld()->GetTimeSeconds() : 0.f;
			Blackboard->SetValueAsFloat(BB_SEARCH_START_TIME, WorldTime);
		}

		// Start by seeking the last known location
		Agent.SetSteeringBehavior(&m_Seek);
	}

	void SearchState::Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (!Blackboard) return;

		if (!m_bArrivedAtLastKnown)
		{
			//Move to last known location
			FVector lastKnownVec = Blackboard->GetValueAsVector(BB_LAST_KNOWN);
			FVector2D lastKnown{ lastKnownVec.X, lastKnownVec.Y };

			FTargetData targetData{};
			targetData.Position = lastKnown;
			m_Seek.SetTarget(targetData);

			float distSq = FVector2D::DistSquared(Agent.GetPosition(), lastKnown);
			if (distSq < m_ArrivalRadius * m_ArrivalRadius)
			{
				m_bArrivedAtLastKnown = true;
				Agent.SetSteeringBehavior(&m_Wander);
			}
		}
		// Wander is active-> SteeringAgent::Tick handles movement automatically
	}

	void SearchState::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(nullptr);
	}

}