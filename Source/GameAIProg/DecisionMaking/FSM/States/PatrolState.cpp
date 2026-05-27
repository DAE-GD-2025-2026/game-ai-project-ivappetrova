#include "PatrolState.h"

#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

namespace GameAI::FSM
{
	PatrolState::PatrolState(TArray<FVector2D> Waypoints, float ArrivalRadius)
		: m_Waypoints(MoveTemp(Waypoints)), m_ArrivalRadius(ArrivalRadius)
	{
	}

	void PatrolState::OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(&m_Seek);
	}

	void PatrolState::Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
{
    if (m_Waypoints.IsEmpty()) return;

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
        return;
    }

    // Stuck detection — if barely moving, skip to next waypoint
    float movedSq = FVector2D::DistSquared(currentPos, m_LastPosition);
    if (movedSq < STUCK_MIN_DIST * STUCK_MIN_DIST)
    {
        m_StuckTimer += DeltaTime;
        if (m_StuckTimer >= STUCK_THRESHOLD)
        {
            // Give up on this waypoint, move to next
            m_WaypointIndex = (m_WaypointIndex + 1) % m_Waypoints.Num();
            m_StuckTimer = 0.f;
        }
    }
    else
    {
        // Moving ìs fine, reset timer
        m_StuckTimer = 0.f;
        m_LastPosition = currentPos;
    }
}

	void PatrolState::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(nullptr);
	}

} 