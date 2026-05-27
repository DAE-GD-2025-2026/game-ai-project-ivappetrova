#pragma once

#include "../BehaviorTree.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "DecisionMaking/BlackboardKeys.h"

namespace GameAI::BT
{
	class SearchAction : public Action
	{
	public:
		explicit SearchAction(float ArrivalRadius = 120.f, float SearchTimeout = 8.f);
		virtual ~SearchAction() override = default;

		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual ENodeStatus Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		Seek   m_Seek;
		Wander m_Wander;
		float  m_ArrivalRadius{};
		float  m_SearchTimeout{};
		bool   m_ArrivedAtLastKnown{ false };
	};
}