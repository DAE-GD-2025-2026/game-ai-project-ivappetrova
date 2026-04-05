#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path;
	std::vector<NodeRecord> openList;
	std::vector<NodeRecord> closedList;

	// 1. Kickstart: create start record and add to openList
	NodeRecord startRecord;
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0.f;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);
	openList.push_back(startRecord);

	NodeRecord currentRecord;

	// 2. While loop
	while (!openList.empty())
	{
		// A. Get record with lowest F-score
		currentRecord = *std::min_element(openList.begin(), openList.end());

		// B. Check if this is the goal node
		if (currentRecord.pNode == pGoalNode)
			break;

		// C. Get all connections from current node
		for (Connection* pConnection : pGraph->FindConnectionsFrom(currentRecord.pNode->GetId()))
		{
			Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			float newCostSoFar = currentRecord.costSoFar + pConnection->GetWeight();

			// D. Check closedList
			auto closedIt = std::find_if(closedList.begin(), closedList.end(),
				[pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });

			if (closedIt != closedList.end())
			{
				if (closedIt->costSoFar <= newCostSoFar)
				{
					// existing is cheaper, skip
					continue; 
				}
				// remove to replace
				closedList.erase(closedIt); 
			}

			// E. Check openList
			auto openIt = std::find_if(openList.begin(), openList.end(),
				[pNextNode](const NodeRecord& r) { return r.pNode == pNextNode; });

			if (openIt != openList.end())
			{
				if (openIt->costSoFar <= newCostSoFar)
				{
					continue;
				}
				openList.erase(openIt);
			}

			// F. Create new record and add to openList
			NodeRecord newRecord;
			newRecord.pNode = pNextNode;
			newRecord.pConnection = pConnection;
			newRecord.costSoFar = newCostSoFar;
			newRecord.estimatedTotalCost = newCostSoFar + GetHeuristicCost(pNextNode, pGoalNode);
			openList.push_back(newRecord);
		}

		// G. Move current from openList to closedList
		openList.erase(std::remove(openList.begin(), openList.end(), currentRecord), openList.end());
		closedList.push_back(currentRecord);
	}

	// No path found
	if (currentRecord.pNode != pGoalNode)
	{
		return path;
	}

	// 3. Backtrack: reconstruct path
	while (currentRecord.pNode != pStartNode)
	{
		path.push_back(currentRecord.pNode);
		int fromId = currentRecord.pConnection->GetFromId();
		auto parentIt = std::find_if(closedList.begin(), closedList.end(),
			[fromId](const NodeRecord& r) 
			{ 
				return r.pNode->GetId() == fromId; 
			});
		currentRecord = *parentIt;
	}
	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}