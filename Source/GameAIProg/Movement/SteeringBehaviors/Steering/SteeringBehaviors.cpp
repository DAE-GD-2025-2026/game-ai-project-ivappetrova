#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - Agent.GetPosition();
	//Steering.LinearVleocity.Normalize(); // I dont need this cuz the tick function -> addmovementinput function normalizes it automatically
	return steering;
}

//FLEE
//*******
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	return steering;
}

//ARRIVE
//*******
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float distance = toTarget.Size();

	// Adjust speed based on distance
	if (distance < steering.TargetRadius)
	{
		Agent.SetMaxLinearSpeed(0.f);
		steering.LinearVelocity = FVector2D::ZeroVector;
	}
	else if (distance < steering.SlowRadius)
	{
		float speedFactor = distance / steering.SlowRadius;
		Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxLinearSpeed() * speedFactor);
		steering.LinearVelocity = toTarget;
	}
	else
	{
		Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxLinearSpeed());
		steering.LinearVelocity = toTarget;
	}

	return steering;
}

//FACE
//*******
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float distance = toTarget.Size();
	float orbitRadius = 200.f; 

	// Move towards target if far away, orbit if close
	if (distance > orbitRadius)
	{
		// Move towards target
		steering.LinearVelocity = toTarget;
	}
	else
	{
		// Orbit around target
		FVector2D perpendicular = FVector2D(-toTarget.Y, toTarget.X); // 90 degree rotation
		steering.LinearVelocity = perpendicular.GetSafeNormal() * Agent.GetMaxLinearSpeed();
	}

	// Always face the target
	float desiredOrientation = FMath::Atan2(toTarget.Y, toTarget.X) * (180.f / PI);
	float currentOrientation = Agent.GetRotation();
	float angleDiff = desiredOrientation - currentOrientation;

	// Normalize angle to [-180, 180]
	while (angleDiff > 180.f) angleDiff -= 360.f;
	while (angleDiff < -180.f) angleDiff += 360.f;

	steering.AngularVelocity = angleDiff;

	return steering;
}