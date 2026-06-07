#pragma once

#include "CoreMinimal.h"
#include "GraphTheory/Algorithms/Heuristics.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "Shared/Level_Base.h"
#include "Shared/Graph/GraphRenderer.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/TerrainGraph/TerrainGridGraph.h"
#include "Level_FallbackPath.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_FallbackPath : public ALevel_Base
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FallbackPathLevel|Input")
	UInputAction* SetStartNodeAction{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FallbackPathLevel|Input")
	UInputAction* SetEndNodeAction{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FallbackPathLevel|Input")
	UInputAction* SetNodeTerrainClearAction{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FallbackPathLevel|Input")
	UInputAction* SetNodeTerrainMudAction{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FallbackPathLevel|Input")
	UInputAction* SetNodeTerrainWaterAction{};

	ALevel_FallbackPath();

	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;
	virtual void BindLevelInputActions() override;

private:
	UPROPERTY()
	ASteeringAgent* m_pAgent{ nullptr };
	PathFollow m_PathFollow{};

	GameAI::TerrainGridGraph* m_pTerrainGraph{ nullptr };
	GameAI::GraphRenderer* m_pRenderer{ nullptr };
	GameAI::TerrainNodeFactory* m_pNodeFactory{ nullptr };

	int m_PathStartNodeId{ 44 };
	int m_PathEndNodeId{ 88 };
	int m_SelectedHeuristic{ 4 };
	GameAI::HeuristicFunctions::Heuristic m_HeuristicFunction{ GameAI::HeuristicFunctions::Chebyshev };
	std::vector<GameAI::Node*> m_FoundPath{};

	// Fallback state
	bool m_IsUsingFallback{ false };
	int  m_FallbackNodeId{ GameAI::Graphs::InvalidNodeId };

	void CalculatePath();
	void UpdateAgentPath(std::vector<GameAI::Node*> const& Path);
	void UpdateImGui();

	// Input functions
	void SetStartNodeId();
	void SetEndNodeId();
	void SetNodeTerrain(GameAI::TerrainNode::Type TerrainType);

	// Debug
	bool m_IsDrawingGrid{ true };
	bool m_IsDrawingNodeNumbers{ false };
	bool m_IsDrawingConnections{ false };
	bool m_IsDrawingConnectionsCosts{ false };
};