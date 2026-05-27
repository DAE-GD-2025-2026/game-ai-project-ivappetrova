#pragma once

#include "../FSM.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "DecisionMaking/BlackboardKeys.h"

namespace GameAI::FSM
{
	class SearchState : public State
	{
	public:
		explicit SearchState(float ArrivalRadius = 120.f);
		virtual ~SearchState() override = default;

		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		Seek   m_Seek;
		Wander m_Wander;

		float m_ArrivalRadius{};
		bool  m_bArrivedAtLastKnown{ false };
	};
}