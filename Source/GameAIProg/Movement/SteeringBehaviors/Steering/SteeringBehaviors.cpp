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
	//Steering.LinearVleocity.Normalize(); // I dont need this cuz the tick function -> addmovementinput function normalizes it automatically
	return steering;
}