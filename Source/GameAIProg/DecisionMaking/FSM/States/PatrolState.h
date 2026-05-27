#pragma once

#include "../FSM.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "DecisionMaking/BlackboardKeys.h"

namespace GameAI::FSM
{
	class PatrolState : public State
	{
	public:
		explicit PatrolState(TArray<FVector2D> Waypoints, float ArrivalRadius = 100.f);
		virtual ~PatrolState() override = default;

		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		TArray<FVector2D> m_Waypoints;
		Seek m_Seek;
		float m_ArrivalRadius{};
		int32 m_WaypointIndex{};

		FVector2D m_LastPosition{};
		float m_StuckTimer{};
		static constexpr float STUCK_THRESHOLD{2.f};   // seconds before declared stuck
		static constexpr float STUCK_MIN_DIST{5.f};   // pixels of movement to not be stuck

	};
}