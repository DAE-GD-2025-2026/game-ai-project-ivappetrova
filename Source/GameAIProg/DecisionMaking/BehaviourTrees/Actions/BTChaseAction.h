#pragma once

#include "../BehaviorTree.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "DecisionMaking/BlackboardKeys.h"

namespace GameAI::BT
{
	class ChaseAction : public Action
	{
	public:
		ChaseAction() = default;
		virtual ~ChaseAction() override = default;

		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual ENodeStatus Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		Pursuit m_Pursuit;
	};
}