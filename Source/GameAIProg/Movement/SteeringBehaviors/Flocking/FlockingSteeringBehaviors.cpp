#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = pFlock->GetAverageNeighborPos() - pAgent.GetPosition();

	return steering;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering{};

	FVector2D totalForce = FVector2D::ZeroVector;

	const TArray<ASteeringAgent*>& neighbors = pFlock->GetNeighbors();
	int nrNeighbors = pFlock->GetNrOfNeighbors();

	for (int index{}; index < nrNeighbors; ++index)
	{
		FVector2D toAgent = pAgent.GetPosition() - neighbors[index]->GetPosition();
		float distance = toAgent.Size();

		if (distance > 0.01f) // Avoid division by zero
		{
			// closer = stronger
			toAgent.Normalize();
			totalForce += toAgent / distance;
		}
	}

	steering.LinearVelocity = totalForce;
	return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput Allignment::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();

	return steering;
}
