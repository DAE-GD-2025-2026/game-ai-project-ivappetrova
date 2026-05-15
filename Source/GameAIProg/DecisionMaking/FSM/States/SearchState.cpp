#include "SearchState.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

namespace GameAI::FSM
{
	SearchState::SearchState(float ArrivalRadius)
		: m_ArrivalRadius(ArrivalRadius)
	{}

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
			//Move to last known location ---
			FVector LastKnownVec = Blackboard->GetValueAsVector(BB_LAST_KNOWN);
			FVector2D LastKnown{ LastKnownVec.X, LastKnownVec.Y };

			FTargetData TargetData{};
			TargetData.Position = LastKnown;
			m_Seek.SetTarget(TargetData);

			float DistSq = FVector2D::DistSquared(Agent.GetPosition(), LastKnown);
			if (DistSq < m_ArrivalRadius * m_ArrivalRadius)
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