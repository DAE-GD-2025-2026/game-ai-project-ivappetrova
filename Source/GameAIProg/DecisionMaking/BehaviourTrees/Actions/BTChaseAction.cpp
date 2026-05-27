#include "BTChaseAction.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

namespace GameAI::BT
{
	void ChaseAction::OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(&m_Pursuit);

		if (Blackboard)
		{
			float worldTime = Agent.GetWorld() ? Agent.GetWorld()->GetTimeSeconds() : 0.f;
			Blackboard->SetValueAsFloat(BB_CHASE_START_TIME, worldTime);
			// Reset search timer so Search gets a fresh window when thief is next lost
			Blackboard->SetValueAsFloat(BB_SEARCH_START_TIME, -999.f);
		}
	}

	ENodeStatus ChaseAction::Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (!Blackboard) return ENodeStatus::Failed;

		FVector targetVec = Blackboard->GetValueAsVector(BB_TARGET_LOCATION);
		Blackboard->SetValueAsVector(BB_LAST_KNOWN, targetVec);

		FTargetData targetData{};
		targetData.Position = FVector2D{ targetVec.X, targetVec.Y };
		FVector thiefVel = Blackboard->GetValueAsVector(TEXT("TargetVelocity"));
		targetData.LinearVelocity = FVector2D{ thiefVel.X, thiefVel.Y };
		m_Pursuit.SetTarget(targetData);

		return ENodeStatus::Running;
	}

	void ChaseAction::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(nullptr);
	}
}