#include "PathFollowSteeringBehavior.h"
#include "../SteeringAgent.h"

PathFollow::PathFollow()
{
	pSeek = new Seek();
	pArrive = new Arrive();
	pArrive->SetTargetRadius(10.0f);
}

PathFollow::~PathFollow()
{
	delete pArrive;
	delete pSeek;
}

void PathFollow::SetPath(std::vector<FVector2D>& path)
{
	pathVec = path;  
	
	currentPathIndex = -1;
	pCurrentSteering = nullptr; // reset before going to next point
	GotoNextPathPoint();
}

SteeringOutput PathFollow::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	if (currentPathIndex < static_cast<int>(pathVec.size()))
	{
		float agentRadius = Agent.GetCapsuleRadius();
		FVector2D ToPathPoint{pathVec[currentPathIndex] - Agent.GetPosition()};
		
		if (ToPathPoint.SizeSquared() < agentRadius * agentRadius)
		{
			// Set max linear speed back to original in case it was modified by Arrive behavior
			Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxLinearSpeed());
			//Reached point of the path
			GotoNextPathPoint();
		}
	}

	if (pCurrentSteering != nullptr)
	{
		return pCurrentSteering->CalculateSteering(DeltaTime, Agent);
	}
	return SteeringOutput{};
}

void PathFollow::GotoNextPathPoint()
{
	++currentPathIndex;
	if (currentPathIndex >= static_cast<int>(pathVec.size())) return;

	FTargetData PathTarget{ pathVec[currentPathIndex] };

	if (currentPathIndex == static_cast<int>(pathVec.size()) - 1)
	{
		pArrive->SetTarget(PathTarget);
		pCurrentSteering = pArrive;
	}
	else
	{
		pSeek->SetTarget(PathTarget);
		pCurrentSteering = pSeek;
	}
}
