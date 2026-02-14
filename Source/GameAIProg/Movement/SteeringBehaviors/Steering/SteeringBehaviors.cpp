#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	//SteeringOutput steering{};

	//steering.LinearVelocity = Target.Position - Agent.GetPosition();

	//////Steering.LinearVleocity.Normalize(); // I dont need this cuz the tick function -> addmovementinput function normalizes it automatically
	//return steering;

	SteeringOutput steering{};
	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float distance = toTarget.Size();

	if (distance < 1.f)
	{
		steering.LinearVelocity = FVector2D::ZeroVector;
	}
	else
	{
		steering.LinearVelocity = toTarget;
	}

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

	if (distance < steering.TargetRadius)
	{
		steering.LinearVelocity = FVector2D::ZeroVector;
		return steering;
	}

	float targetSpeed;

	if (distance > steering.SlowRadius)
	{
		targetSpeed = Agent.GetMaxLinearSpeed();
	}
	else
	{
		targetSpeed = Agent.GetMaxLinearSpeed() * (distance / steering.SlowRadius);
	}

	FVector2D desiredVelocity{ toTarget.GetSafeNormal() * targetSpeed };

	steering.LinearVelocity = desiredVelocity;

	return steering;
}

//FACE
//*******
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D toTarget = Target.Position - Agent.GetPosition();
	float distance = toTarget.Size();
	const float ORBIT_RADIUS{200.f};

	// Move towards target if far away, orbit if close
	if (distance > ORBIT_RADIUS)
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

//PURSUIT
//*******
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D distanceToTarget = Target.Position - Agent.GetPosition();

	// callculate time to reach the target
	float d = distanceToTarget.Size();
	float v = Agent.GetMaxLinearSpeed();
	float t = d / (v + 0.01f);

	// calculate future position of the target
	auto targetVelocity = Target.LinearVelocity;
	auto futurePosition = Target.Position + targetVelocity * t;

	// seek the future position
	steering.LinearVelocity = futurePosition - Agent.GetPosition();

	return steering;
}

//EVADE
//*******
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D distanceToTarget = Target.Position - Agent.GetPosition();

	// callculate time to reach the target
	float d = distanceToTarget.Size();
	float v = Agent.GetMaxLinearSpeed();
	float t = d / (v + 0.01f);

	// calculate future position of the target
	auto targetVelocity = Target.LinearVelocity;
	auto futurePosition = Target.Position + targetVelocity * t;

	// seek the future position
	steering.LinearVelocity = Agent.GetPosition() - futurePosition;

	return steering;
}
