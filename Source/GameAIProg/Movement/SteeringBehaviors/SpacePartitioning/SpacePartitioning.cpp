#include "SpacePartitioning.h"

// Cell
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float LEFT = BoundingBox.Min.X;
	const float BOTTOM = BoundingBox.Min.Y;
	const float WIDTH = BoundingBox.Max.X - BoundingBox.Min.X;
	const float HEIGHT = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ LEFT , BOTTOM  },
		{ LEFT , BOTTOM + HEIGHT  },
		{ LEFT + WIDTH , BOTTOM + HEIGHT },
		{ LEFT + WIDTH , BOTTOM  },
	};

	return rectPoints;
}

// Partitioned Space
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{ pWorld }
	, SpaceWidth{ Width }
	, SpaceHeight{ Height }
	, NrOfRows{ Rows }
	, NrOfCols{ Cols }
	, NrOfNeighbors{ 0 }
{
	Neighbors.SetNum(MaxEntities);

	CellWidth = Width / Cols;
	CellHeight = Height / Rows;
	CellOrigin = FVector2D(-Width * 0.5f, -Height * 0.5f);

	// Create all cells, row by row (bottom-top), column by column (left-right)
	Cells.reserve(Rows * Cols);
	for (int row{0}; row < Rows; ++row)
	{
		for (int col{0}; col < Cols; ++col)
		{
			float left = CellOrigin.X + col * CellWidth;
			float bottom = CellOrigin.Y + row * CellHeight;
			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int index = PositionToIndex(Agent.GetPosition());
	if (index >= 0 && index < static_cast<int>(Cells.size()))
	{
		Cells[index].Agents.push_back(&Agent);
	}
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	int oldIndex = PositionToIndex(OldPos);
	int newIndex = PositionToIndex(Agent.GetPosition());

	if (oldIndex == newIndex) return; // still in the same cell – no work needed

	// Remove from old cell
	if (oldIndex >= 0 && oldIndex < static_cast<int>(Cells.size()))
	{
		Cells[oldIndex].Agents.remove(&Agent);
	}

	// Insert into new cell
	if (newIndex >= 0 && newIndex < static_cast<int>(Cells.size()))
	{
		Cells[newIndex].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	const FVector2D AGENT_POS = Agent.GetPosition();
	const float     RADIUS_SQUARED = QueryRadius * QueryRadius;

	// bounding box around the neighborhood radius
	FRect queryBox;
	queryBox.Min = AGENT_POS - FVector2D(QueryRadius, QueryRadius);
	queryBox.Max = AGENT_POS + FVector2D(QueryRadius, QueryRadius);

	for (Cell& cell : Cells)
	{
		// Skip cells that don't overlap the query box at all
		if (!DoRectsOverlap(queryBox, cell.BoundingBox)) continue;

		for (ASteeringAgent* pOther : cell.Agents)
		{
			if (pOther == &Agent || pOther == nullptr) continue;

			// Only keep actual neighbors
			FVector2D toOther = pOther->GetPosition() - AGENT_POS;
			if (toOther.SizeSquared() <= RADIUS_SQUARED)
			{
				Neighbors[NrOfNeighbors] = pOther;
				++NrOfNeighbors;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
	{
		c.Agents.clear();
	}
}

void CellSpace::RenderCells() const
{
	for (size_t index{}; index < static_cast<int>(Cells.size()); ++index)
	{
		const Cell& CELL = Cells[index];
		const FRect& BOX = CELL.BoundingBox;
		constexpr float Z = 150.f; // high enough to be visible above the ground

		// Only draw occupied cells
		if (CELL.Agents.empty()) continue;

		FVector corners[4] =
		{
			FVector(BOX.Min.X, BOX.Min.Y, Z),
			FVector(BOX.Min.X, BOX.Max.Y, Z),
			FVector(BOX.Max.X, BOX.Max.Y, Z),
			FVector(BOX.Max.X, BOX.Min.Y, Z),
		};

		for (int corner{}; corner < 4; ++corner)
		{
			DrawDebugLine(pWorld, corners[corner], corners[(corner + 1) % 4], FColor(80, 200, 80), false, 0.f, 0, 3.f);
		}

		// Agent count — placed at the center of the cell, scaled up so it's readable
		FVector2D center = (BOX.Min + BOX.Max) * 0.5f;
		FString countStr = FString::Printf(TEXT("%d"), static_cast<int>(CELL.Agents.size()));
		DrawDebugString(pWorld, FVector(center.X, center.Y, Z + 50.f), countStr, nullptr, FColor::Yellow, 0.f, true, 2.f);
	}
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	int col = static_cast<int>((Pos.X - CellOrigin.X) / CellWidth);
	int row = static_cast<int>((Pos.Y - CellOrigin.Y) / CellHeight);

	// Clamp so edge/boundary agents don't go out of bounds
	col = FMath::Clamp(col, 0, NrOfCols - 1);
	row = FMath::Clamp(row, 0, NrOfRows - 1);

	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const& RectA, FRect const& RectB)
{
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
	return true;
}