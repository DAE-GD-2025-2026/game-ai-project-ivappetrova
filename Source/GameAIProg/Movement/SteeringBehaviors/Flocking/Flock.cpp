#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{ pWorld }
	, FlockSize{ FlockSize }
	, pAgentToEvade{ pAgentToEvade }
{
	Agents.SetNum(FlockSize);
	pNeighbors.SetNum(FlockSize - 1);

	// ==========================================
	// STEP 2: Blended Steering - behavior setup
	// ==========================================
	/*pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pAlignmentBehavior = std::make_unique<Allignment>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();

	std::vector<BlendedSteering::WeightedBehavior> weightedBehaviors;
	weightedBehaviors.push_back({ pSeparationBehavior.get(), 0.7f });
	weightedBehaviors.push_back({ pCohesionBehavior.get(), 0.3f });
	weightedBehaviors.push_back({ pAlignmentBehavior.get(), 0.3f });
	weightedBehaviors.push_back({ pSeekBehavior.get(), 0.2f });
	weightedBehaviors.push_back({ pWanderBehavior.get(), 0.5f });

	pBlendedSteering = std::make_unique<BlendedSteering>(weightedBehaviors);*/
	// ==========================================

	// ==========================================
	// STEP 3: Priority Steering - evade setup
	// ==========================================
	// pEvadeBehavior = std::make_unique<Evade>();
	// pPrioritySteering = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>
	// {
	// 	pEvadeBehavior.get(),
	// 	pBlendedSteering.get()
	// });
	// ==========================================

	// Spawn agents
	for (int index{}; index < FlockSize; ++index)
	{
		float randomX = FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
		float randomY = FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
		FVector SpawnLocation(randomX, randomY, 200.f);
		FRotator SpawnRotation(0.f, FMath::RandRange(0.f, 360.f), 0.f);

		Agents[index] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnLocation, SpawnRotation);

		//if (Agents[index])
		//{
		//	// STEP 2: assign blended steering
		//	Agents[index]->SetSteeringBehavior(pBlendedSteering.get());

		//	// STEP 3: swap to priority steering instead
		//	// Agents[index]->SetSteeringBehavior(pPrioritySteering.get());
		//}
	}
}

Flock::~Flock()
{
	// TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
	for (ASteeringAgent* agent : Agents)
	{
		// STEP 1: RegisterNeighbors - fills pNeighbors memory pool for this agent
		RegisterNeighbors(agent);

		if (agent == nullptr) continue;
		agent->Tick(DeltaTime);
	}

	// STEP 3: update evade target each tick
	// if (pAgentToEvade && pEvadeBehavior)
	// {
	// 	FTargetData evadeTarget;
	// 	evadeTarget.Position = FVector2D(pAgentToEvade->GetPosition());
	// 	FVector vel = pAgentToEvade->GetVelocity();
	// 	evadeTarget.LinearVelocity = FVector2D(vel.X, vel.Y);
	// 	pEvadeBehavior->SetTarget(evadeTarget);
	// }

	RenderDebug();
}

void Flock::RenderDebug()
{
	if (DebugRenderNeighborhood)
	{
		RenderNeighborhood();
	}

	if (DebugRenderSteering)
	{
		for (ASteeringAgent* agent : Agents)
		{
			if (agent)
			{
				agent->SetDebugRenderingEnabled(true);
			}
		}
	}
	else
	{
		for (ASteeringAgent* agent : Agents)
		{
			if (agent)
			{
				agent->SetDebugRenderingEnabled(false);
			}
		}
	}
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	{
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		ImGui::Checkbox("Debug Steering", &DebugRenderSteering);
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// ==========================================
		// STEP 2: ImGui weight sliders
		// ==========================================
		/*ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		auto& weightedBehaviors = pBlendedSteering->GetWeightedBehaviorsRef();

		for (auto& wb : weightedBehaviors)
		{
			if (wb.pBehavior == pSeparationBehavior.get())
			{
				ImGui::SliderFloat("Separation", &wb.Weight, 0.0f, 2.0f);
			}
			else if (wb.pBehavior == pCohesionBehavior.get())
			{
				ImGui::SliderFloat("Cohesion", &wb.Weight, 0.0f, 2.0f);
			}
			else if (wb.pBehavior == pAlignmentBehavior.get())
			{
				ImGui::SliderFloat("Alignment", &wb.Weight, 0.0f, 2.0f);
			}
			else if (wb.pBehavior == pSeekBehavior.get())
			{
				ImGui::SliderFloat("Seek", &wb.Weight, 0.0f, 2.0f);
			}
			else if (wb.pBehavior == pWanderBehavior.get())
			{
				ImGui::SliderFloat("Wander", &wb.Weight, 0.0f, 2.0f);
			}
		}*/
		// ==========================================

		ImGui::Spacing();
		ImGui::SliderFloat("Neighborhood Radius", &NeighborhoodRadius, 50.0f, 500.0f);

		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	if (Agents.Num() == 0) return;

	ASteeringAgent* agent = Agents[0];
	RegisterNeighbors(agent);

	for (int index{}; index < NrOfNeighbors; ++index)
	{
		DrawDebugLine(pWorld,
			FVector(agent->GetPosition().X, agent->GetPosition().Y, 150.f),
			FVector(pNeighbors[index]->GetPosition().X, pNeighbors[index]->GetPosition().Y, 150.f),
			FColor::Green, false, -1.f, 0, 3.f);
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	NrOfNeighbors = 0;

	const float NEIGHBORHOOD_RADIUS_SQUARED{ NeighborhoodRadius * NeighborhoodRadius };

	if (pAgent == nullptr) return;
	const FVector2D AGENT_POS{ pAgent->GetPosition() };

	for (ASteeringAgent* other : Agents)
	{
		if (other == pAgent || other == nullptr) continue; // never include self
		
		FVector2D toNeighbor{ other->GetPosition() - AGENT_POS };

		if (toNeighbor.SizeSquared() <= NEIGHBORHOOD_RADIUS_SQUARED)
		{
			pNeighbors[NrOfNeighbors] = other;
			++NrOfNeighbors;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	if (NrOfNeighbors == 0) return FVector2D::ZeroVector;

	FVector2D sumNeighbourPos = FVector2D::ZeroVector;

	for (int index{}; index < NrOfNeighbors; ++index)
	{
		sumNeighbourPos += pNeighbors[index]->GetPosition();
	}

	return sumNeighbourPos / (float)NrOfNeighbors;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	if (NrOfNeighbors == 0) return FVector2D::ZeroVector;

	FVector2D sum = FVector2D::ZeroVector;

	for (int index{}; index < NrOfNeighbors; ++index)
	{
		FVector velocity3D = pNeighbors[index]->GetVelocity();
		sum += FVector2D(velocity3D.X, velocity3D.Y);
	}

	return sum / (float)NrOfNeighbors;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	//FTargetData targetData;
	//targetData.Position = Target.Position;
	//targetData.Orientation = Target.Orientation;

	//pSeekBehavior->SetTarget(targetData);
}