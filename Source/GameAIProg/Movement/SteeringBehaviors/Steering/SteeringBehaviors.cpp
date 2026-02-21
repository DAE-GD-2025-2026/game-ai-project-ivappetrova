#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

namespace
{
	constexpr float DEBUG_Z = 10.f;
}

static void DrawArrow( UWorld* World, const FVector2D& From, const FVector2D& To, const FColor& Color, float Thickness = 3.f)
{
	DrawDebugDirectionalArrow( World, FVector(From.X, From.Y, DEBUG_Z), FVector(To.X, To.Y, DEBUG_Z), 30.f, Color, false, 0.f, 0, Thickness );
}

static void DrawCircle( UWorld* World, const FVector2D& Center, float Radius, const FColor& Color, float Thickness = 2.f)
{
	DrawDebugCircle( World, FVector(Center.X, Center.Y, DEBUG_Z), Radius, 64, Color, false, 0.f, 0, Thickness, FVector(1, 0, 0), FVector(0, 1, 0) );
}

///////////////// SEEK /////////////////
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D toTarget{ Target.Position - Agent.GetPosition() };
	steering.LinearVelocity = (toTarget.Size() < 1.f) ? FVector2D::ZeroVector : toTarget;

	// DEBUG
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugSphere(Agent.GetWorld(), FVector(Target.Position.X, Target.Position.Y, DEBUG_Z), 10.f, 12, FColor::Green, false, 0.f);
		DrawArrow(Agent.GetWorld(), Agent.GetPosition(), Agent.GetPosition() + steering.LinearVelocity, FColor::Green);
	}

	return steering;
}

///////////////// FLEE /////////////////
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	steering.LinearVelocity = Agent.GetPosition() - Target.Position;

	// DEBUG
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugSphere(Agent.GetWorld(), FVector(Target.Position.X, Target.Position.Y, DEBUG_Z), 10.f, 12, FColor::Red, false, 0.f);
		DrawArrow(Agent.GetWorld(), Agent.GetPosition(), Agent.GetPosition() + steering.LinearVelocity, FColor::Red);
	}

	return steering;
}

///////////////// ARRIVE /////////////////
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	FVector2D toTarget{ Target.Position - Agent.GetPosition() };
	float distance = toTarget.Size();

	if (distance < steering.TargetRadius)
	{
		Agent.SetMaxLinearSpeed(0.f);
		steering.LinearVelocity = FVector2D::ZeroVector;
	}
	else if (distance < steering.SlowRadius)
	{
		float factor{ distance / steering.SlowRadius };
		Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxLinearSpeed() * factor);
		steering.LinearVelocity = toTarget;
	}
	else
	{
		Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxLinearSpeed());
		steering.LinearVelocity = toTarget;
	}

	// DEBUG
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugSphere(Agent.GetWorld(), FVector(Target.Position.X, Target.Position.Y, DEBUG_Z), 8.f, 12, FColor::Green, false, 0.f);
		DrawCircle(Agent.GetWorld(), Target.Position, steering.TargetRadius, FColor::Green);
		DrawCircle(Agent.GetWorld(), Target.Position, steering.SlowRadius, FColor::Cyan);
	}

	return steering;
}

////////////////// FACE /////////////////
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};
	FVector2D toTarget{ Target.Position - Agent.GetPosition() };
	float distance = toTarget.Size();
	const float ORBIT_RADIUS{ 200.f };

	if (distance > ORBIT_RADIUS)
	{
		steering.LinearVelocity = toTarget;
	}
	else
	{
		FVector2D perp(-toTarget.Y, toTarget.X);
		steering.LinearVelocity = perp.GetSafeNormal() * Agent.GetMaxLinearSpeed();
	}

	float desiredOrientation = FMath::RadiansToDegrees(FMath::Atan2(toTarget.Y, toTarget.X));
	float angleDiff{ desiredOrientation - Agent.GetRotation() };

	while (angleDiff > 180.f)
	{
		angleDiff -= 360.f;
	}
	while (angleDiff < -180.f)
	{
		angleDiff += 360.f;
	}

	steering.AngularVelocity = angleDiff;

	//DEBUG
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugSphere(Agent.GetWorld(), FVector(Target.Position.X, Target.Position.Y, DEBUG_Z), 8.f, 12, FColor::Green, false, 0.f);
		DrawCircle(Agent.GetWorld(), Target.Position, ORBIT_RADIUS, FColor::Blue);
		float rad{ FMath::DegreesToRadians(Agent.GetRotation()) };
		FVector2D facing(FMath::Cos(rad), FMath::Sin(rad));
		DrawArrow(Agent.GetWorld(), Agent.GetPosition(), Agent.GetPosition() + facing * 100.f, FColor::Magenta);
	}

	return steering;
}

//////////////// PURSUIT /////////////////
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D toTarget{ Target.Position - Agent.GetPosition() };
	float t = toTarget.Size() / (Agent.GetMaxLinearSpeed() + 0.01f);
	FVector2D futurePos = Target.Position + Target.LinearVelocity * t;

	steering.LinearVelocity = futurePos - Agent.GetPosition();

	// DEBUG
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugSphere(Agent.GetWorld(), FVector(Target.Position.X, Target.Position.Y, DEBUG_Z), 8.f, 12, FColor::Red, false, 0.f);
		DrawDebugSphere(Agent.GetWorld(), FVector(futurePos.X, futurePos.Y, DEBUG_Z), 8.f, 12, FColor::Yellow, false, 0.f);
		DrawDebugLine(Agent.GetWorld(), FVector(Target.Position.X, Target.Position.Y, DEBUG_Z), FVector(futurePos.X, futurePos.Y, DEBUG_Z),
			FColor::Yellow, false, 0.f, 0, 2.f);
		DrawArrow(Agent.GetWorld(), Agent.GetPosition(), Agent.GetPosition() + steering.LinearVelocity, FColor::Green);
	}

	return steering;
}

///////////////// EVADE /////////////////
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	FVector2D toTarget{ Target.Position - Agent.GetPosition() };

	// Out of range - return invalid so PrioritySteering falls through to blended
	if (toTarget.Size() > m_EvadeRadius)
	{
		steering.IsValid = false;
		return steering;
	}

	float t = toTarget.Size() / (Agent.GetMaxLinearSpeed() + 0.01f);
	FVector2D futurePos{ Target.Position + Target.LinearVelocity * t };

	steering.LinearVelocity = Agent.GetPosition() - futurePos;
	steering.IsValid = true; // <-- THIS is the critical line

	// DEBUG
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugSphere(Agent.GetWorld(), FVector(futurePos.X, futurePos.Y, DEBUG_Z), 8.f, 12, FColor::Yellow, false, 0.f);
		DrawArrow(Agent.GetWorld(), Agent.GetPosition(), Agent.GetPosition() + steering.LinearVelocity, FColor::Red);
	}

	return steering;
}

////////////////// WANDER /////////////////
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput steering{};

	const float CIRCLE_DISTANCE{ 100.f };
	const float CIRCLE_RADIUS { 50.f};
	const float ANGLE_CHANGE{ 45.f };

	m_WanderAngle += FMath::RandRange(-1.f, 1.f) * ANGLE_CHANGE * DeltaT;

	FVector2D velocity{ Agent.GetLinearVelocity() };
	float dirAngle =(velocity.SizeSquared() > 0.01f) ? FMath::Atan2(velocity.Y, velocity.X) : FMath::DegreesToRadians(Agent.GetRotation());

	FVector2D forward(FMath::Cos(dirAngle), FMath::Sin(dirAngle));
	FVector2D circleCenter = Agent.GetPosition() + forward * CIRCLE_DISTANCE;

	float targetAngle{ dirAngle + FMath::DegreesToRadians(m_WanderAngle) };
	FVector2D displacement(FMath::Cos(targetAngle), FMath::Sin(targetAngle));
	displacement *= CIRCLE_RADIUS;

	FVector2D targetPoint{ circleCenter + displacement };
	steering.LinearVelocity = targetPoint - Agent.GetPosition();

	// DEBUG
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawCircle(Agent.GetWorld(), circleCenter, CIRCLE_RADIUS, FColor::Blue);
		DrawArrow(Agent.GetWorld(), Agent.GetPosition(), targetPoint, FColor::Green);
	}

	return steering;
}
