#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// TODO If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
		{
			return Eulerianity::notEulerian;
		}

		// TODO Count nodes with odd degree 
		int oddDegreeCount{ 0 };
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();

		for (Node* pNode : Nodes)
		{
			int degree = static_cast<int>(m_pGraph->FindConnectionsFrom(pNode->GetId()).size());
			if (degree % 2 != 0)
			{
				++oddDegreeCount;
			}
		}

		// TODO A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (oddDegreeCount > 2)
		{
			return Eulerianity::notEulerian;
		}

		// TODO A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// TODO An Euler trail can be made, but only starting and ending in these 2 nodes
		if (oddDegreeCount == 2)
		{
			return Eulerianity::semiEulerian;
		}

		// TODO A connected graph with no odd nodes is Eulerian
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		// TODO Check if there can be an Euler path
		eulerianity = IsEulerian();

		// TODO If this graph is not eulerian, return the empty path
		if (eulerianity == Eulerianity::notEulerian)
		{
			return Path;
		}

		// Choose starting node:
		// - For semiEulerian: start at a node with odd degree
		// - For eulerian: start at any node
		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (Node* pNode : Nodes)
			{
				int degree = static_cast<int>(graphCopy.FindConnectionsFrom(pNode->GetId()).size());
				if (degree % 2 != 0)
				{
					currentNodeId = pNode->GetId();
					break;
				}
			}
		}
		else
		{
			currentNodeId = Nodes[0]->GetId();
		}
		
		// TODO Start algorithm loop
		std::stack<int> nodeStack;

		while (!nodeStack.empty() || !graphCopy.FindConnectionsFrom(currentNodeId).empty())
		{
			std::vector<Connection*> connections = graphCopy.FindConnectionsFrom(currentNodeId);

			if (!connections.empty())
			{
				// Has neighbours: push current, move to neighbour, remove edge
				nodeStack.push(currentNodeId);
				int nextId = connections[0]->GetToId();
				graphCopy.RemoveConnection(currentNodeId, nextId);
				// set neighbour as current — don't push yet
				currentNodeId = nextId; 
			}
			else
			{
				// Dead end: add to path, pop stack to backtrack
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());
				currentNodeId = nodeStack.top();
				nodeStack.pop();
			}
		}
		// Step 5: also add the last node after the loop
		Path.push_back(m_pGraph->GetNode(currentNodeId).get());
		
		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// TODO Mark the visited node
		visited[startIndex] = true;

		// TODO Ask the graph for the connections from that node
		int nodeId = Nodes[startIndex]->GetId();
		std::vector<Connection*> connections = m_pGraph->FindConnectionsFrom(nodeId);

		// TODO recursively visit any valid connected nodes that were not visited before
		for (auto pConn : connections)
		{
			int toId = pConn->GetToId();

			// TODO Tip: use an index-based for-loop to find the correct index
			for (size_t index{}; index < static_cast<int>(Nodes.size()); ++index)
			{
				if (Nodes[index]->GetId() == toId && !visited[index])
				{
					VisitAllNodesDFS(Nodes, visited, index);
					break;
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;

		// TODO choose a starting node
		int startNodeIndex{ 0 };
		for (size_t index{}; index < static_cast<int>(Nodes.size()); ++index)
		{
			if (!m_pGraph->FindConnectionsFrom(Nodes[index]->GetId()).empty())
			{
				startNodeIndex = index;
				break;
			}
		}

		// TODO start a depth-first-search traversal from the node that has at least one connection
		std::vector<bool> visited(Nodes.size(), false);
		VisitAllNodesDFS(Nodes, visited, startNodeIndex);

		// TODO if a node was never visited, this graph is not connected
		for (int index{}; index < static_cast<int>(Nodes.size()); ++index)
		{
			if (!visited[index])
			{
				return false;
			}
		}

		return true;
	}
}