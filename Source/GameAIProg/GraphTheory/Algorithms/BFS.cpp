#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	std::queue<Node*> openList;
	std::map<Node*, Node*> cameFrom;

	openList.push(pStartNode);
	cameFrom[pStartNode] = nullptr;

	while (!openList.empty())
	{
		Node* currentNode = openList.front();
		openList.pop();

		if (currentNode == pDestinationNode)
			break;

		for (Connection* pConnection : pGraph->FindConnectionsFrom(currentNode->GetId()))
		{
			Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			if (cameFrom.find(pNextNode) == cameFrom.end())
			{
				openList.push(pNextNode);
				cameFrom[pNextNode] = currentNode;
			}
		}
	}

	// Reconstruct path
	if (cameFrom.find(pDestinationNode) == cameFrom.end())
	{
		return path; // no path found
	}

	Node* current = pDestinationNode;
	while (current != nullptr)
	{
		path.push_back(current);
		current = cameFrom[current];
	}
	std::reverse(path.begin(), path.end());
	return path;
}
