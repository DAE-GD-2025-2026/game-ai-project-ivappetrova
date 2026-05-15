#include "ChaseState.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

namespace GameAI::FSM
{
	void ChaseState::OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(&m_Pursuit);

		if (Blackboard)
		{
			float WorldTime = Agent.GetWorld() ? Agent.GetWorld()->GetTimeSeconds() : 0.f;
			Blackboard->SetValueAsFloat(BB_CHASE_START_TIME, WorldTime);
		}
	}

	void ChaseState::Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (!Blackboard) return;

		FVector TargetVec = Blackboard->GetValueAsVector(BB_TARGET_LOCATION);
		FVector2D TargetPos{ TargetVec.X, TargetVec.Y };

		// Keep last known location up to date while we have sight
		Blackboard->SetValueAsVector(BB_LAST_KNOWN, TargetVec);

		FTargetData TargetData{};
		TargetData.Position = TargetPos;
		FVector ThiefVel = Blackboard->GetValueAsVector(TEXT("TargetVelocity"));
		TargetData.LinearVelocity = FVector2D{ ThiefVel.X, ThiefVel.Y };
		m_Pursuit.SetTarget(TargetData);
	}

	void ChaseState::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(nullptr);
	}
}