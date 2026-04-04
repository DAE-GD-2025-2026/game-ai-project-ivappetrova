// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_GraphTheory.h"

#include "Algorithms/EulerianPath.h"
#include "Shared/GameAISpectator.h"

using namespace GameAI;

// Sets default values
ALevel_GraphTheory::ALevel_GraphTheory()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_GraphTheory::BeginPlay()
{
	Super::BeginPlay();
	
	// Add the graph editor to our player
	if (PlayerController = Cast<APlayerController>(GetWorld()->GetFirstLocalPlayerFromController()->PlayerController); 
		GraphEditorClass && PlayerController)
	{
		PlayerGraphEditor = NewObject<UGraphEditorComponent>(PlayerController->GetPawn(), GraphEditorClass);
		PlayerGraphEditor->RegisterComponent();
		PlayerGraphEditor->SetEditedGraph(&Graph);
		PlayerGraphEditor->SetNodeFactory(&NodeFactory);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Unable to get PlayerController from LocalPlayer or GraphEditorClass is null"))
		return;
	}


	// set the world for the renderer so it can draw debug shapes
	Renderer = new GraphRenderer{ GetWorld() };
	
	// Make the view orthogonal for less perspective issues
	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}
	
	//// TODO Make the graph and a couple connected nodes here...
	int NodeA = Graph.AddNode(std::make_unique<Node>(FVector2D{ 200.f,  0.f })); // top
	int NodeB = Graph.AddNode(std::make_unique<Node>(FVector2D{ 0.f,    -200.f })); // left
	int NodeC = Graph.AddNode(std::make_unique<Node>(FVector2D{ 0.f,    200.f })); // right
	int NodeD = Graph.AddNode(std::make_unique<Node>(FVector2D{ -200.f, -200.f })); // bottom-left
	int NodeE = Graph.AddNode(std::make_unique<Node>(FVector2D{ -200.f, 200.f })); // bottom-right

	Graph.AddConnection(NodeA, NodeB);
	Graph.AddConnection(NodeA, NodeC);
	Graph.AddConnection(NodeB, NodeC);
	Graph.AddConnection(NodeB, NodeD);
	Graph.AddConnection(NodeB, NodeE);
	Graph.AddConnection(NodeC, NodeE);
	Graph.AddConnection(NodeD, NodeE);

	Graph.SetConnectionCostsToDistances();
	
	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass,
		FVector{ 0, 0, 90 }, FRotator::ZeroRotator);

	Agent->SetSteeringBehavior(&PathFollow);

	// Calculate initial path
	EulerianPath ep{ &Graph };
	Eulerianity eulerianity{};
	std::vector<Node*> trail = ep.FindPath(eulerianity);
	if (eulerianity != Eulerianity::notEulerian && !trail.empty())
	{
		UpdateAgentPath(trail);
	}
}

void ALevel_GraphTheory::BeginDestroy()
{
	Super::BeginDestroy();
	delete Renderer;
	Renderer = nullptr;
}

void ALevel_GraphTheory::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);
		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("Graph Theory");
		ImGui::Spacing();
		ImGui::Spacing();

		//End
		ImGui::End();
	}
#pragma endregion UI
	
	if (Renderer) Renderer->RenderGraph(Graph);
	
	// TODO Check if the graph has updated
	if (PlayerGraphEditor->HasGraphUpdated())
	{
		Graph.SetConnectionCostsToDistances();

		// TODO if so, run the EulerianPath algorithm
		EulerianPath ep{ &Graph };
		Eulerianity eulerianity{};
		std::vector<Node*> trail = ep.FindPath(eulerianity);

		// TODO if a path is found, have the agent follow it
		if (eulerianity != Eulerianity::notEulerian && !trail.empty())
		{
			UpdateAgentPath(trail);
		}
		else
		{
			// Graph is no longer Eulerian — stop the agent
			std::vector<FVector2D> emptyPath{};
			PathFollow.SetPath(emptyPath);
		}
	}

}

void ALevel_GraphTheory::UpdateAgentPath(std::vector<Node*> const& Trail)
{
	std::vector<FVector2D> path{};

	// TODO convert Node vector to positions vector
	path.reserve(Trail.size());
	for (Node* pNode : Trail)
	{
		path.push_back(pNode->GetPosition());
	}

	PathFollow.SetPath(path);
	if (path.size() > 0)
	{
		Agent->SetPosition(path[0]);
	}
}




