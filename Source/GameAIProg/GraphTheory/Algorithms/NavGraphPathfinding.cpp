#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	// A. Get start and end triangles
	FVector2D startOutPos{}, endOutPos{};
	TriPolygon const* pNavPoly = pNavGraph->GetNavPolygon();

	TriPolygon::Triangle const* pStartTriangle = pNavPoly->GetClosestTriangleToPosition(startPos, startOutPos);
	TriPolygon::Triangle const* pEndTriangle = pNavPoly->GetClosestTriangleToPosition(endPos, endOutPos);

	if (!pStartTriangle || !pEndTriangle)
		return finalPath;

	// Same triangle - direct path
	if (pStartTriangle == pEndTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	// B. Clone the graph
	std::unique_ptr<NavGraph> pClonedGraph = pNavGraph->Clone();

	// C. Create start node and connect to edges of start triangle
	auto pStartNode = std::make_unique<NavGraphNode>(startOutPos, -1);
	int startNodeId = pClonedGraph->AddNode(std::move(pStartNode));

	for (auto const& Edge : pStartTriangle->GetEdges())
	{
		int EdgeIdx = pNavPoly->FindEdgeIndex(Edge).value_or(-1);
		int NodeId = pClonedGraph->GetNodeIdFromEdgeIndex(EdgeIdx);
		if (NodeId != Graphs::InvalidNodeId)
		{
			float Cost = FVector2D::Distance(startOutPos,
				pClonedGraph->GetNode(NodeId)->GetPosition());
			auto NewConn = std::make_unique<Connection>(startNodeId, NodeId);
			NewConn->SetWeight(Cost);
			pClonedGraph->AddConnection(std::move(NewConn));
		}
	}

	// D. Create end node and connect to edges of end triangle
	auto pEndNode = std::make_unique<NavGraphNode>(endOutPos, -1);
	int endNodeId = pClonedGraph->AddNode(std::move(pEndNode));

	for (auto const& Edge : pEndTriangle->GetEdges())
	{
		int EdgeIdx = pNavPoly->FindEdgeIndex(Edge).value_or(-1);
		int NodeId = pClonedGraph->GetNodeIdFromEdgeIndex(EdgeIdx);
		if (NodeId != Graphs::InvalidNodeId)
		{
			float Cost = FVector2D::Distance(endOutPos,
				pClonedGraph->GetNode(NodeId)->GetPosition());
			auto NewConn = std::make_unique<Connection>(NodeId, endNodeId);
			NewConn->SetWeight(Cost);
			pClonedGraph->AddConnection(std::move(NewConn));
		}
	}

	// E. Run A*
	AStar pathfinder(pClonedGraph.get(), HeuristicFunctions::Euclidean);
	Node* pStart = pClonedGraph->GetNode(startNodeId).get();
	Node* pEnd = pClonedGraph->GetNode(endNodeId).get();
	std::vector<Node*> nodePath = pathfinder.FindPath(pStart, pEnd);

	// Debug positions
	for (Node* pNode : nodePath)
	{
		debugNodePositions.push_back(pNode->GetPosition());
	}

	// Convert to positions
	for (Node* pNode : nodePath)
	{
		finalPath.push_back(pNode->GetPosition());
	}

	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}