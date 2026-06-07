#include "Level_FallbackPath.h"

#include "GraphTheory/Algorithms/AStar.h"
#include "GraphTheory/Algorithms/Heuristics.h"
#include "Shared/GameAISpectator.h"
#include "Math/TransformCalculus2D.h"

#include <queue>
#include <unordered_set>

using namespace GameAI;

ALevel_FallbackPath::ALevel_FallbackPath()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_FallbackPath::BeginPlay()
{
	Super::BeginPlay();

	TrimWorld->bShouldTrimWorld = false;

	if (PlayerController = Cast<APlayerController>(GetWorld()->GetFirstLocalPlayerFromController()->PlayerController); PlayerController)
	{
		if (AGameAISpectator* pPlayer = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); pPlayer)
		{
			pPlayer->SetCameraProjection(ECameraProjectionMode::Orthographic);
		}
	}

	// Spawn the Agent
	m_pAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0, 0, 90 }, FRotator::ZeroRotator);
	m_pAgent->SetDebugRenderingEnabled(false);
	m_pAgent->SetSteeringBehavior(&m_PathFollow);

	// Create graph & renderer
	m_pRenderer = new GraphRenderer{ GetWorld() };
	GraphRenderOptions renderOptions{};
	renderOptions.bDrawConnectionWeights = false;
	renderOptions.bDrawConnections = false;
	renderOptions.bDrawNodeIds = false;
	renderOptions.bDrawNodes = false;
	m_pRenderer->SetRenderOptions(renderOptions);
	m_pNodeFactory = new TerrainNodeFactory{};
	m_pTerrainGraph = new TerrainGridGraph{ m_pNodeFactory, 10, 10, 200.0f, 1.0f, FVector2D{-1000.0f, -1000.0f}, false };

	CalculatePath();
}

void ALevel_FallbackPath::BeginDestroy()
{
	Super::BeginDestroy();

	delete m_pRenderer;
	delete m_pTerrainGraph;
	delete m_pNodeFactory;
}

void ALevel_FallbackPath::BindLevelInputActions()
{
	Super::BindLevelInputActions();

	PlayerEnhancedInputComponent->BindAction(SetStartNodeAction, ETriggerEvent::Triggered, this,
		&ALevel_FallbackPath::SetStartNodeId);
	PlayerEnhancedInputComponent->BindAction(SetEndNodeAction, ETriggerEvent::Triggered, this,
		&ALevel_FallbackPath::SetEndNodeId);

	PlayerEnhancedInputComponent->BindAction(SetNodeTerrainClearAction, ETriggerEvent::Started, this,
		&ALevel_FallbackPath::SetNodeTerrain, TerrainNode::Type::Clear);
	PlayerEnhancedInputComponent->BindAction(SetNodeTerrainMudAction, ETriggerEvent::Started, this,
		&ALevel_FallbackPath::SetNodeTerrain, TerrainNode::Type::Mud);
	PlayerEnhancedInputComponent->BindAction(SetNodeTerrainWaterAction, ETriggerEvent::Started, this,
		&ALevel_FallbackPath::SetNodeTerrain, TerrainNode::Type::Water);
}

void ALevel_FallbackPath::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	UpdateImGui();

	GraphRenderOptions renderOptions = m_pRenderer->GetRenderOptions();
	renderOptions.bDrawNodes = m_IsDrawingGrid;
	renderOptions.bDrawNodeIds = m_IsDrawingNodeNumbers;
	renderOptions.bDrawConnections = m_IsDrawingConnections;
	renderOptions.bDrawConnectionWeights = m_IsDrawingConnectionsCosts;
	m_pRenderer->SetRenderOptions(renderOptions);

	m_pRenderer->RenderGraph(*m_pTerrainGraph);
	m_pTerrainGraph->DrawTerrain(GetWorld());

	if (m_IsDrawingGrid)
	{
		m_pTerrainGraph->DebugDrawCells(GetWorld());
	}
}

void ALevel_FallbackPath::CalculatePath()
{
	m_IsUsingFallback = false;
	m_FallbackNodeId = Graphs::InvalidNodeId;

	if (m_PathStartNodeId != Graphs::InvalidNodeId
		&& m_PathEndNodeId != Graphs::InvalidNodeId
		&& m_PathStartNodeId != m_PathEndNodeId)
	{
		AStar pathfinder = AStar(m_pTerrainGraph, m_HeuristicFunction);
		TerrainNode* const pSTART_NODE = m_pTerrainGraph->GetNodeAs<TerrainNode>(m_PathStartNodeId);
		TerrainNode* const pEND_NODE = m_pTerrainGraph->GetNodeAs<TerrainNode>(m_PathEndNodeId);

		m_FoundPath = pathfinder.FindPath(pSTART_NODE, pEND_NODE);

		// FALLBACK: goal unreachable, find closest reachable node instead
		if (m_FoundPath.empty())
		{
			// BFS flood-fill from start to collect all reachable node IDs
			std::unordered_set<int> reachable;
			std::queue<int> queue;
			queue.push(m_PathStartNodeId);
			reachable.insert(m_PathStartNodeId);

			while (!queue.empty())
			{
				int currentId = queue.front();
				queue.pop();

				for (Connection* pConn : m_pTerrainGraph->FindConnectionsFrom(currentId))
				{
					int nextId = pConn->GetToId();
					if (reachable.find(nextId) == reachable.end())
					{
						reachable.insert(nextId);
						queue.push(nextId);
					}
				}
			}

			// Pick the reachable node closest to the goal
			FVector2D const GOAL_POS = pEND_NODE->GetPosition();
			TerrainNode* pBestNode = nullptr;
			float bestDistSq = FLT_MAX;

			for (int reachableId : reachable)
			{
				if (reachableId == m_PathStartNodeId) continue;

				TerrainNode* pCandidate = m_pTerrainGraph->GetNodeAs<TerrainNode>(reachableId);
				float distSq = FVector2D::DistSquared(pCandidate->GetPosition(), GOAL_POS);
				if (distSq < bestDistSq)
				{
					bestDistSq = distSq;
					pBestNode = pCandidate;
				}
			}

			if (pBestNode)
			{
				m_FoundPath = pathfinder.FindPath(pSTART_NODE, pBestNode);
				m_IsUsingFallback = true;
				m_FallbackNodeId = pBestNode->GetId();
			}
			else
			{
				m_FoundPath.clear();
			}
		}

		UpdateAgentPath(m_FoundPath);
	}
	else
	{
		m_FoundPath.clear();
	}

	// Update highlighted nodes
	std::vector<std::pair<int, FColor>> pathToHighlight{};
	pathToHighlight.push_back({ m_PathStartNodeId, FColor::Green });

	if (!m_FoundPath.empty())
	{
		for (size_t index{}; index < m_FoundPath.size() - 1; ++index)
		{
			pathToHighlight.push_back({ m_FoundPath[index]->GetId(), FColor::Yellow });
		}
	}

	// Fallback node in orange, original unreachable goal stays red
	if (m_IsUsingFallback && m_FallbackNodeId != Graphs::InvalidNodeId)
	{
		pathToHighlight.push_back({ m_FallbackNodeId, FColor::Orange });
	}
	pathToHighlight.push_back({ m_PathEndNodeId, FColor::Red });
	m_pRenderer->SetHighlightedNodes(pathToHighlight);
}

void ALevel_FallbackPath::UpdateAgentPath(std::vector<Node*> const& Path)
{
	std::vector<FVector2D> pathPositions{};
	pathPositions.reserve(Path.size());
	for (auto pNode : Path)
	{
		pathPositions.emplace_back(pNode->GetPosition());
	}

	m_PathFollow.SetPath(pathPositions);
	if (pathPositions.size() > 0)
	{
		m_pAgent->SetPosition(pathPositions[0]);
	}
}

void ALevel_FallbackPath::UpdateImGui()
{
#pragma region UI
	{
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: Set Path Start");
		ImGui::Text("RMB: Set Path End");
		ImGui::Text("1: Set terrain to Clear");
		ImGui::Text("2: Set terrain to Mud");
		ImGui::Text("3: Set terrain to Water");
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("Fallback Pathfinding");
		ImGui::Spacing();

		// Status
		if (m_IsUsingFallback)
		{
			ImGui::TextColored({ 1.0f, 0.5f, 0.0f, 1.0f }, "Fallback: goal unreachable");
			ImGui::TextColored({ 1.0f, 0.5f, 0.0f, 1.0f }, "Pathing to closest reachable node");
		}
		else if (!m_FoundPath.empty())
		{
			ImGui::TextColored({ 0.4f, 1.0f, 0.4f, 1.0f }, "Path found");
		}
		else if (m_PathStartNodeId != Graphs::InvalidNodeId && m_PathEndNodeId != Graphs::InvalidNodeId)
		{
			ImGui::TextColored({ 1.0f, 0.2f, 0.2f, 1.0f }, "No path found");
		}

		ImGui::Spacing();

		if (ImGui::Checkbox("Grid", &m_IsDrawingGrid)) {}
		if (ImGui::Checkbox("Node Numbers", &m_IsDrawingNodeNumbers)) {}
		if (ImGui::Checkbox("Connections", &m_IsDrawingConnections)) {}
		if (ImGui::Checkbox("Connection Costs", &m_IsDrawingConnectionsCosts)) {}
		if (ImGui::Combo("Heuristic", &m_SelectedHeuristic, "Manhattan\0Euclidean\0SqEuclidean\0Octile\0Chebyshev", 4))
		{
			switch (m_SelectedHeuristic)
			{
			case 0: m_HeuristicFunction = HeuristicFunctions::Manhattan;   break;
			case 1: m_HeuristicFunction = HeuristicFunctions::Euclidean;   break;
			case 2: m_HeuristicFunction = HeuristicFunctions::SqEuclidean; break;
			case 3: m_HeuristicFunction = HeuristicFunctions::Octile;      break;
			default:
			case 4: m_HeuristicFunction = HeuristicFunctions::Chebyshev;  break;
			}
			CalculatePath();
		}

		ImGui::Spacing();
		ImGui::End();
	}
#pragma endregion
}

void ALevel_FallbackPath::SetStartNodeId()
{
	int const NEW_START = m_pTerrainGraph->GetNodeIdAtPosition(FVector2D{ LatestMouseWorldPos });
	if (NEW_START >= 0 && NEW_START != m_PathEndNodeId)
	{
		m_PathStartNodeId = NEW_START;
		CalculatePath();
	}
}

void ALevel_FallbackPath::SetEndNodeId()
{
	int const NEW_END = m_pTerrainGraph->GetNodeIdAtPosition(FVector2D{ LatestMouseWorldPos });
	if (NEW_END >= 0 && NEW_END != m_PathStartNodeId)
	{
		m_PathEndNodeId = NEW_END;
		CalculatePath();
	}
}

void ALevel_FallbackPath::SetNodeTerrain(TerrainNode::Type TerrainType)
{
	m_pTerrainGraph->PaintNodeAtPosition(FVector2D{ LatestMouseWorldPos }, TerrainType);
	CalculatePath();
}