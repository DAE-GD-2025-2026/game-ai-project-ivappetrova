#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
	{
		std::vector<NavLine> Portals{};

		for (size_t i{}; i < (Path.size() - 1); ++i)
		{
			NavGraphNode* pCurrent = static_cast<NavGraphNode*>(Path[i]);
			NavGraphNode* pNext = static_cast<NavGraphNode*>(Path[i + 1]);

			int currentEdgeIdx = pCurrent->GetEdgeIdx();
			int nextEdgeIdx = pNext->GetEdgeIdx();

			// Find the shared triangle between current and next node's edges
			for (auto const& Triangle : NavPoly.GetTriangles())
			{
				bool hasCurrentEdge = currentEdgeIdx >= 0 &&
					Triangle.HasEdge(NavPoly.GetEdges()[currentEdgeIdx]);
				bool hasNextEdge = nextEdgeIdx >= 0 &&
					Triangle.HasEdge(NavPoly.GetEdges()[nextEdgeIdx]);

				if (hasCurrentEdge && hasNextEdge)
				{
					// The portal is the edge of the next node
					TriPolygon::Edge const& PortalEdge = NavPoly.GetEdges()[nextEdgeIdx];
					FVector P1 = PortalEdge.GetP1(NavPoly);
					FVector P2 = PortalEdge.GetP2(NavPoly);

					// P1 = right point, P2 = left point relative to path direction
					NavLine Portal{ FVector2D{P1}, FVector2D{P2} };
					Portals.push_back(Portal);
					break;
				}
			}
		}

		// Degenerate end portal
		if (!Path.empty())
		{
			FVector2D EndPos = Path.back()->GetPosition();
			Portals.push_back({ EndPos, EndPos });
		}

		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
	{
		std::vector<FVector2D> Path{};
		if (Portals.empty()) return Path;

		// 2D cross product (Z component)
		auto Cross2D = [](FVector2D const& A, FVector2D const& B) -> float
			{
				return A.X * B.Y - A.Y * B.X;
			};

		FVector2D apex = Portals[0].P1; // start point
		FVector2D rightLeg = Portals[0].P1 - apex;
		FVector2D leftLeg = Portals[0].P2 - apex;
		int rightLegIndex = 0;
		int leftLegIndex = 0;

		Path.push_back(apex);

		size_t amtPortals = Portals.size();

		for (size_t portalIdx{1}; portalIdx < amtPortals; ++portalIdx)
		{
			NavLine const& CurrentPortal = Portals[portalIdx];

			// RIGHT CHECK
			FVector2D newRightLeg = CurrentPortal.P1 - apex;
			if (Cross2D(rightLeg, newRightLeg) >= 0) // going inwards (CCW)
			{
				if (Cross2D(leftLeg, newRightLeg) > 0) // crosses over left leg
				{
					// Add left leg apex
					apex = apex + leftLeg;
					Path.push_back(apex);
					int apexIndex = leftLegIndex;
					portalIdx = leftLegIndex + 1;
					leftLegIndex = portalIdx;
					rightLegIndex = portalIdx;

					if (portalIdx < amtPortals)
					{
						rightLeg = Portals[rightLegIndex].P1 - apex;
						leftLeg = Portals[leftLegIndex].P2 - apex;
						continue;
					}
				}
				else
				{
					rightLeg = newRightLeg;
					rightLegIndex = portalIdx;
				}
			}

			// LEFT CHECK
			FVector2D newLeftLeg = CurrentPortal.P2 - apex;
			if (Cross2D(leftLeg, newLeftLeg) <= 0) // going inwards (CW)
			{
				if (Cross2D(rightLeg, newLeftLeg) < 0) // crosses over right leg
				{
					// Add right leg apex
					apex = apex + rightLeg;
					Path.push_back(apex);
					int apexIndex = rightLegIndex;
					portalIdx = rightLegIndex + 1;
					rightLegIndex = portalIdx;
					leftLegIndex = portalIdx;

					if (portalIdx < amtPortals)
					{
						rightLeg = Portals[rightLegIndex].P1 - apex;
						leftLeg = Portals[leftLegIndex].P2 - apex;
						continue;
					}
				}
				else
				{
					leftLeg = newLeftLeg;
					leftLegIndex = portalIdx;
				}
			}
		}

		// Add last point
		Path.push_back(Portals.back().P1);
		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}
