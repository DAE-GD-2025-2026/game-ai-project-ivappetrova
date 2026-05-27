#pragma once

#include "../BehaviorTree.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

namespace GameAI::BT
{
	class PatrolAction : public Action
	{
	public:
		explicit PatrolAction(TArray<FVector2D> Waypoints, float ArrivalRadius = 100.f);
		virtual ~PatrolAction() override = default;

		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual ENodeStatus Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		TArray<FVector2D> m_Waypoints;
		Seek m_Seek;
		float m_ArrivalRadius{};
		int32 m_WaypointIndex{ 0 };

		FVector2D m_LastPosition{ 0.f, 0.f };
		float m_StuckTimer{ 0.f };
		static constexpr float STUCK_THRESHOLD{ 2.f };
		static constexpr float STUCK_MIN_DIST{ 10.f };
	};
}
