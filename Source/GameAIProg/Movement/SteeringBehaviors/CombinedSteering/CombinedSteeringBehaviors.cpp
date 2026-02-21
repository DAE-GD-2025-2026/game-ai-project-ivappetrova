
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput blendedSteering = {};
	float totalWeight{};

	for (const WeightedBehavior& weightedBehavior : WeightedBehaviors)
	{
		if (weightedBehavior.pBehavior && weightedBehavior.Weight > 0.f)
		{
			SteeringOutput steering = weightedBehavior.pBehavior->CalculateSteering(DeltaT, Agent);
			blendedSteering.LinearVelocity += steering.LinearVelocity * weightedBehavior.Weight;
			blendedSteering.AngularVelocity += steering.AngularVelocity * weightedBehavior.Weight;
			totalWeight += weightedBehavior.Weight;
		}
	}

	if (totalWeight > 0.f)
	{
		blendedSteering.LinearVelocity /= totalWeight;
		blendedSteering.AngularVelocity /= totalWeight;
	}

	blendedSteering.IsValid = true;

	// Debug rendering
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugLine( Agent.GetWorld(),FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 10.f), FVector(Agent.GetPosition().X + blendedSteering.LinearVelocity.X,
				Agent.GetPosition().Y + blendedSteering.LinearVelocity.Y, 10.f), FColor::Magenta, false, -1.f, 0, 3.f );
	}

	return blendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}