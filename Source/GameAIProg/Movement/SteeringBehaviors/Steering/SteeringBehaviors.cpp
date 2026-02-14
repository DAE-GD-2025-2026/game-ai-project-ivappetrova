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

//WANDER
//*******
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	const float CIRCLE_DISTANCE{ 100.f };
	const float CIRCLE_RADIUS{ 50.f };
	const float ANGLE_CHANGE{ 45.f }; // deg/sec

	float angleOffset = FMath::RandRange(-1.f, 1.f) * ANGLE_CHANGE * DeltaT;
	m_WanderAngle += angleOffset;

	// Use velocity direction if available, otherwise fall back to rotation
	FVector2D velocity = Agent.GetLinearVelocity();
	float directionAngle;

	if (velocity.SizeSquared() > 0.01f)
	{
		directionAngle = FMath::Atan2(velocity.Y, velocity.X);
	}
	else
	{
		directionAngle = Agent.GetRotation() * (PI / 180.f);
	}

	FVector2D forward(
		FMath::Cos(directionAngle),
		FMath::Sin(directionAngle));

	FVector2D circleCenter = Agent.GetPosition() + forward * CIRCLE_DISTANCE;

	float targetAngle = directionAngle + FMath::DegreesToRadians(m_WanderAngle);

	FVector2D displacement(
		FMath::Cos(targetAngle),
		FMath::Sin(targetAngle));

	displacement *= CIRCLE_RADIUS;

	FVector2D targetPoint = circleCenter + displacement;

	steering.LinearVelocity = targetPoint - Agent.GetPosition();

	// Fixed debug sphere
	DrawDebugSphere(
		Agent.GetWorld(),
		FVector(targetPoint.X, targetPoint.Y, 20.f),  // ✅ Fixed
		10.f,
		8,
		FColor::Green,
		false,
		0.f
	);

	// Draw the wander circle
	DrawDebugCircle(
		Agent.GetWorld(),
		FVector(circleCenter.X, circleCenter.Y, 20.f),
		CIRCLE_RADIUS,
		32,
		FColor::Blue,
		false,
		0.f,
		0,
		2.f,
		FVector(0, 0, 1),  // Up vector
		FVector(1, 0, 0)   // Axis vector
	);

	// Draw line from agent to target point
	DrawDebugLine(
		Agent.GetWorld(),
		FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 20.f),
		FVector(targetPoint.X, targetPoint.Y, 20.f),
		FColor::Yellow,
		false,
		0.f,
		0,
		3.f
	);

	return steering;
}