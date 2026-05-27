#include "BTPatrolAction.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

namespace GameAI::BT
{
	PatrolAction::PatrolAction(TArray<FVector2D> Waypoints, float ArrivalRadius)
		: m_Waypoints(MoveTemp(Waypoints)), m_ArrivalRadius(ArrivalRadius)
	{
	}

	void PatrolAction::OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(&m_Seek);
	}

	ENodeStatus PatrolAction::Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (m_Waypoints.IsEmpty()) return ENodeStatus::Failed;

		m_WaypointIndex = m_WaypointIndex % m_Waypoints.Num();
		FVector2D target = m_Waypoints[m_WaypointIndex];

		FTargetData targetData{};
		targetData.Position = target;
		m_Seek.SetTarget(targetData);

		FVector2D currentPos = Agent.GetPosition();

		// Arrival check
		float distSq = FVector2D::DistSquared(currentPos, target);
		if (distSq < m_ArrivalRadius * m_ArrivalRadius)
		{
			m_WaypointIndex = (m_WaypointIndex + 1) % m_Waypoints.Num();
			m_StuckTimer = 0.f;
			m_LastPosition = currentPos;
			return ENodeStatus::Running;
		}

		// Stuck detection
		float movedSq = FVector2D::DistSquared(currentPos, m_LastPosition);
		if (movedSq < STUCK_MIN_DIST * STUCK_MIN_DIST)
		{
			m_StuckTimer += DeltaTime;
			if (m_StuckTimer >= STUCK_THRESHOLD)
			{
				m_WaypointIndex = (m_WaypointIndex + 1) % m_Waypoints.Num();
				m_StuckTimer = 0.f;
			}
		}
		else
		{
			m_StuckTimer = 0.f;
			m_LastPosition = currentPos;
		}

		return ENodeStatus::Running;
	}

	void PatrolAction::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(nullptr);
	}
}
