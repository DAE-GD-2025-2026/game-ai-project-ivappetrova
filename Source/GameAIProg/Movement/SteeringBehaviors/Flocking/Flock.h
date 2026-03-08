#pragma once
#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"

class CellSpace;

class Flock final
{
public:
	Flock(
		UWorld* pWorld,
		TSubclassOf<ASteeringAgent> AgentClass,
		int FlockSize = 10,
		float WorldSize = 100.f,
		ASteeringAgent* const pAgentToEvade = nullptr,
		bool bTrimWorld = false);

	~Flock();

	void Tick(float DeltaTime);
	void RenderDebug();
	void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);

	void RegisterNeighbors(ASteeringAgent* const Agent);
	int GetNrOfNeighbors() const;
	const TArray<ASteeringAgent*>& GetNeighbors() const;

	FVector2D GetAverageNeighborPos() const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const& Target);

private:
	// For debug rendering purposes
	UWorld* pWorld{ nullptr };

	int FlockSize{ 0 };
	TArray<ASteeringAgent*> Agents{};

	// Flat spatial partitioning (runtime toggle via ImGui)
	std::unique_ptr<CellSpace> pPartitionedSpace{};
	TArray<FVector2D>          OldPositions{};
	bool                       bUseSpacePartitioning{ false };

	// Brute-force neighborhood (used when partitioning is OFF)
	TArray<ASteeringAgent*> pNeighbors{};
	int NrOfNeighbors{ 0 };

	float NeighborhoodRadius{ 200.f };

	ASteeringAgent* pAgentToEvade{ nullptr };

	//Steering Behaviors
	std::unique_ptr<Separation> pSeparationBehavior{};
	std::unique_ptr<Cohesion>   pCohesionBehavior{};
	std::unique_ptr<Allignment> pAlignmentBehavior{};
	std::unique_ptr<Seek>       pSeekBehavior{};
	std::unique_ptr<Wander>     pWanderBehavior{};
	std::unique_ptr<Evade>      pEvadeBehavior{};

	std::unique_ptr<BlendedSteering>  pBlendedSteering{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	// UI and rendering
	bool DebugRenderSteering{ false };
	bool DebugRenderNeighborhood{ true };
	bool DebugRenderPartitions{ false };

	// FPS comparison
	float FpsWithoutPartitioning{};
	float FpsWithPartitioning{};

	void RenderNeighborhood();
};