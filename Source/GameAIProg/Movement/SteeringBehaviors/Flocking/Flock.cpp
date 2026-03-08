#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SpacePartitioning/SpacePartitioning.h" 
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
	OldPositions.SetNum(FlockSize);

	// ---- Spatial partitioning setup ----
	const int NR_OF_CELLS_X{ 10 };
	const int NR_OF_CELLS_Y{ 10 };
	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld,
		WorldSize, WorldSize,
		NR_OF_CELLS_X, NR_OF_CELLS_X,
		FlockSize);

	// Blended steering
	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pAlignmentBehavior = std::make_unique<Allignment>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();

	std::vector<BlendedSteering::WeightedBehavior> weightedBehaviors;
	weightedBehaviors.push_back({ pSeparationBehavior.get(), 0.7f });
	weightedBehaviors.push_back({ pCohesionBehavior.get(),   0.3f });
	weightedBehaviors.push_back({ pAlignmentBehavior.get(),  0.3f });
	weightedBehaviors.push_back({ pSeekBehavior.get(),       0.2f });
	weightedBehaviors.push_back({ pWanderBehavior.get(),     0.5f });

	pBlendedSteering = std::make_unique<BlendedSteering>(weightedBehaviors);

	// Priority steering 
	pEvadeBehavior = std::make_unique<Evade>();
	pEvadeBehavior->SetEvadeRadius(700.f);
	pPrioritySteering = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>{ pEvadeBehavior.get(), pBlendedSteering.get() });

	// Spawn agents
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	for (size_t index{}; index < FlockSize; ++index)
	{
		float randomX = FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
		float randomY = FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
		FVector SpawnLocation(randomX, randomY, 200.f);
		FRotator SpawnRotation(0.f, FMath::RandRange(0.f, 360.f), 0.f);

		Agents[index] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnLocation, SpawnRotation, SpawnParams);

		if (Agents[index])
		{
			// Disable auto-tick: the Flock manually calls Tick() so agents don't update twice
			Agents[index]->SetActorTickEnabled(false);

			Agents[index]->SetSteeringBehavior(pPrioritySteering.get());

			// Register in cell space
			pPartitionedSpace->AddAgent(*Agents[index]);
			OldPositions[index] = Agents[index]->GetPosition();
		}
	}
}

Flock::~Flock()
{
	// unique_ptrs clean themselves up; nothing extra needed
}

void Flock::Tick(float DeltaTime)
{
	// Print FPS to output log
	if (DeltaTime > 0.f)
	{
		float FPS = 1.f / DeltaTime;
		UE_LOG(LogTemp, Display, TEXT("FPS: %.1f | Space Partitioning: %s"), FPS, bUseSpacePartitioning ? TEXT("ON") : TEXT("OFF"));
	}

	for (size_t index{}; index < Agents.Num(); ++index)
	{
		ASteeringAgent* agent = Agents[index];
		if (agent == nullptr) continue;

		// Before ticking, save old position and update cell if needed
		if (bUseSpacePartitioning)
		{
			pPartitionedSpace->UpdateAgentCell(*agent, OldPositions[index]);
			OldPositions[index] = agent->GetPosition();
		}

		RegisterNeighbors(agent);
		agent->Tick(DeltaTime);
	}

	// Update evade target
	if (pAgentToEvade && pEvadeBehavior)
	{
		FTargetData evadeTarget;
		evadeTarget.Position = pAgentToEvade->GetPosition();
		FVector vel = pAgentToEvade->GetVelocity();
		evadeTarget.LinearVelocity = FVector2D(vel.X, vel.Y);
		pEvadeBehavior->SetTarget(evadeTarget);
	}

	RenderDebug();
}

void Flock::RenderDebug()
{
	if (DebugRenderNeighborhood)
	{
		RenderNeighborhood();
	}

	if (DebugRenderPartitions && bUseSpacePartitioning)
	{
		pPartitionedSpace->RenderCells();
	}

	for (ASteeringAgent* agent : Agents)
	{
		if (agent)
		{
			agent->SetDebugRenderingEnabled(DebugRenderSteering);
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

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Spacing();

		if (ImGui::Button("Snapshot FPS"))
		{
			if (bUseSpacePartitioning)
			{
				FpsWithPartitioning = ImGui::GetIO().Framerate;
			}
			else
			{
				FpsWithoutPartitioning = ImGui::GetIO().Framerate;
			}
		}
		ImGui::SameLine();
		ImGui::TextDisabled("(capture current mode)");

		ImGui::Spacing();
		if (FpsWithoutPartitioning > 0.f)
		{
			ImGui::TextColored(ImVec4(1.f, 0.6f, 0.2f, 1.f), "Brute-force:     %.1f FPS", FpsWithoutPartitioning);
		}
		else
		{
			ImGui::TextDisabled("Brute-force:     -- (not captured)");
		}

		if (FpsWithPartitioning > 0.f)
		{
			ImGui::TextColored(ImVec4(0.2f, 1.f, 0.4f, 1.f), "Space Part.:     %.1f FPS", FpsWithPartitioning);
		}
		else
		{
			ImGui::TextDisabled("Space Part.:     -- (not captured)");
		}

		if (FpsWithPartitioning > 0.f && FpsWithoutPartitioning > 0.f)
		{
			float speedup = FpsWithPartitioning / FpsWithoutPartitioning;
			ImGui::Spacing();
			ImGui::Text("Speedup: %.2fx", speedup);
		}

		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		ImGui::Checkbox("Debug Steering", &DebugRenderSteering);
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);

		// --- Spatial partitioning toggle ---
		if (ImGui::Checkbox("Use Space Partitioning", &bUseSpacePartitioning))
		{
			if (bUseSpacePartitioning)
			{
				// Rebuild the cell space with current agent positions
				pPartitionedSpace->EmptyCells();
				for (int i{}; i < Agents.Num(); ++i)
				{
					if (Agents[i])
					{
						pPartitionedSpace->AddAgent(*Agents[i]);
						OldPositions[i] = Agents[i]->GetPosition();
					}
				}
			}
		}

		if (bUseSpacePartitioning)
		{
			ImGui::Checkbox("Debug Partitions", &DebugRenderPartitions);
		}

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// Behavior weights
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		auto& weightedBehaviors = pBlendedSteering->GetWeightedBehaviorsRef();

		for (auto& wb : weightedBehaviors)
		{
			if (wb.pBehavior == pSeparationBehavior.get()) ImGui::SliderFloat("Separation", &wb.Weight, 0.0f, 2.0f);
			else if (wb.pBehavior == pCohesionBehavior.get())   ImGui::SliderFloat("Cohesion", &wb.Weight, 0.0f, 2.0f);
			else if (wb.pBehavior == pAlignmentBehavior.get())  ImGui::SliderFloat("Alignment", &wb.Weight, 0.0f, 2.0f);
			else if (wb.pBehavior == pSeekBehavior.get())       ImGui::SliderFloat("Seek", &wb.Weight, 0.0f, 2.0f);
			else if (wb.pBehavior == pWanderBehavior.get())     ImGui::SliderFloat("Wander", &wb.Weight, 0.0f, 2.0f);
		}

		ImGui::Spacing();
		ImGui::SliderFloat("Neighborhood Radius", &NeighborhoodRadius, 50.0f, 1500.0f);

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

	int nrNeighbors = GetNrOfNeighbors();
	const auto& NEIGHBORS = GetNeighbors();

	for (size_t index{}; index < nrNeighbors; ++index)
	{
		DrawDebugLine(pWorld,
			FVector(agent->GetPosition().X, agent->GetPosition().Y, 150.f),
			FVector(NEIGHBORS[index]->GetPosition().X, NEIGHBORS[index]->GetPosition().Y, 150.f),
			FColor::Green, false, 0.f, 0, 3.f);
	}

	// Draw neighborhood radius around agent[0]
	DrawDebugCircle(pWorld,
		FVector(agent->GetPosition().X, agent->GetPosition().Y, 150.f),
		NeighborhoodRadius, 64, FColor::Yellow,
		false, 0.f, 0, 2.f,
		FVector(1, 0, 0), FVector(0, 1, 0));

	// When partitioning is on: also draw the query AABB
	if (bUseSpacePartitioning)
	{
		FVector2D pos = agent->GetPosition();
		FVector minPt(pos.X - NeighborhoodRadius, pos.Y - NeighborhoodRadius, 155.f);
		FVector maxPt(pos.X + NeighborhoodRadius, pos.Y + NeighborhoodRadius, 155.f);
		DrawDebugBox(pWorld, (minPt + maxPt) * 0.5f, (maxPt - minPt) * 0.5f, FColor::Orange, false, 0.f, 0, 2.f);
	}
}

// ---- Neighbor registration (runtime dispatch) ----

void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (pAgent == nullptr) return;

	if (bUseSpacePartitioning)
	{
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
	}
	else
	{
		// Brute-force fallback
		NrOfNeighbors = 0;
		const float RADIUS_SQUARED = NeighborhoodRadius * NeighborhoodRadius;
		const FVector2D AGENT_POS = pAgent->GetPosition();

		for (ASteeringAgent* other : Agents)
		{
			if (other == pAgent || other == nullptr) continue;

			FVector2D toNeighbor{ other->GetPosition() - AGENT_POS };
			if (toNeighbor.SizeSquared() <= RADIUS_SQUARED)
			{
				pNeighbors[NrOfNeighbors] = other;
				++NrOfNeighbors;
			}
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
	return bUseSpacePartitioning
		? pPartitionedSpace->GetNrOfNeighbors()
		: NrOfNeighbors;
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	return bUseSpacePartitioning
		? pPartitionedSpace->GetNeighbors()
		: pNeighbors;
}

FVector2D Flock::GetAverageNeighborPos() const
{
	int numberOfNeighbors = GetNrOfNeighbors();
	if (numberOfNeighbors == 0) return FVector2D::ZeroVector;

	const auto& NEIGHBORS = GetNeighbors();
	FVector2D sum = FVector2D::ZeroVector;
	for (size_t index{}; index < numberOfNeighbors; ++index)
	{
		sum += NEIGHBORS[index]->GetPosition();
	}

	return sum / static_cast<float>(numberOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	int numberOfNeighbors = GetNrOfNeighbors();
	if (numberOfNeighbors == 0) return FVector2D::ZeroVector;

	const auto& neighbors = GetNeighbors();
	FVector2D sum = FVector2D::ZeroVector;
	for (size_t index{}; index < numberOfNeighbors; ++index)
	{
		FVector v = neighbors[index]->GetVelocity();
		sum += FVector2D(v.X, v.Y);
	}

	return sum / static_cast<float>(numberOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	FTargetData targetData;
	targetData.Position = Target.Position;
	targetData.Orientation = Target.Orientation;
	pSeekBehavior->SetTarget(targetData);
}