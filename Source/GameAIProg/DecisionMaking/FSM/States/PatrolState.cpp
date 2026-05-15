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
    FVector2D Target = m_Waypoints[m_WaypointIndex];

    FTargetData TargetData{};
    TargetData.Position = Target;
    m_Seek.SetTarget(TargetData);

    FVector2D CurrentPos = Agent.GetPosition();

    // Arrival check
    float DistSq = FVector2D::DistSquared(CurrentPos, Target);
    if (DistSq < m_ArrivalRadius * m_ArrivalRadius)
    {
        m_WaypointIndex = (m_WaypointIndex + 1) % m_Waypoints.Num();
        m_StuckTimer = 0.f;
        m_LastPosition = CurrentPos;
        return;
    }

    // Stuck detection — if barely moving, skip to next waypoint
    float MovedSq = FVector2D::DistSquared(CurrentPos, m_LastPosition);
    if (MovedSq < STUCK_MIN_DIST * STUCK_MIN_DIST)
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
        // Moving fine, reset timer
        m_StuckTimer = 0.f;
        m_LastPosition = CurrentPos;
    }
}

	void PatrolState::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		Agent.SetSteeringBehavior(nullptr);
	}

} 