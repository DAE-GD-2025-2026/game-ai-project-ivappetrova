#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*static_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	// 1. Go over all the edges of the navigation mesh and create nodes
	std::vector<TriPolygon::Edge> const& Edges = pNavPoly->GetEdges();
	for (size_t EdgeIdx{}; EdgeIdx < Edges.size(); ++EdgeIdx)
	{
		TriPolygon::Edge const& Edge = Edges[EdgeIdx];

		// Only create a node if this edge is shared between 2 triangles
		std::vector<int> Neighbors = pNavPoly->GetTriangleNeighbors(
			pNavPoly->GetTriangles()[0]);

		// Count how many triangles use this edge
		int TriangleCount = 0;
		for (auto const& Triangle : pNavPoly->GetTriangles())
		{
			if (Triangle.HasEdge(Edge)) ++TriangleCount;
		}

		if (TriangleCount >= 2)
		{
			// Midpoint of the edge
			FVector P1 = Edge.GetP1(*pNavPoly);
			FVector P2 = Edge.GetP2(*pNavPoly);
			FVector2D MidPoint = FVector2D{ (P1.X + P2.X) / 2.f, (P1.Y + P2.Y) / 2.f };

			auto NewNode = std::make_unique<NavGraphNode>(MidPoint, EdgeIdx);
			AddNode(std::move(NewNode));
		}
	}

	//2. Create connections now that every node is created	
	for (auto const& Triangle : pNavPoly->GetTriangles())
	{
		std::vector<int> NodeIds{};

		for (auto const& Edge : Triangle.GetEdges())
		{
			int EdgeIdx = pNavPoly->FindEdgeIndex(Edge).value_or(-1);
			int NodeId = GetNodeIdFromEdgeIndex(EdgeIdx);
			if (NodeId != Graphs::InvalidNodeId)
			{
				NodeIds.push_back(NodeId);
			}
		}

		// 2 valid nodes -> 1 connection
		// 3 valid nodes -> 3 connections
		for (size_t i{}; i < NodeIds.size(); ++i)
		{
			for (size_t j = i + 1; j < NodeIds.size(); ++j)
			{
				AddConnection(NodeIds[i], NodeIds[j]);
			}
		}
	}

	// 3. Set connection costs to distance
	SetConnectionCostsToDistances();
}
