#pragma once

#include "../FSM.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "BlackboardKeys.h"

namespace GameAI::FSM
{
	class ChaseState : public State
	{
	public:
		ChaseState() = default;
		virtual ~ChaseState() override = default;

		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		Pursuit m_Pursuit;
	};
}