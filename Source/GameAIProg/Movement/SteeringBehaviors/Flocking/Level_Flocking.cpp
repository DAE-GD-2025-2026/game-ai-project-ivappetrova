// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_Flocking.h"


// Sets default values
ALevel_Flocking::ALevel_Flocking()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_Flocking::BeginPlay()
{
	Super::BeginPlay();

	TrimWorld->SetTrimWorldSize(3000.f);
	TrimWorld->bShouldTrimWorld = true;

	FVector SpawnLocation(0.f, 0.f, 200.f);
	FRotator SpawnRotation(0.f, 0.f, 0.f);
	pAgentToEvade = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, SpawnLocation, SpawnRotation);

	if (pAgentToEvade)
	{
		// Make it bigger so it's visually distinct from the flock
		pAgentToEvade->SetActorScale3D(FVector(3.f, 3.f, 3.f));

		// Give it a wander behavior so it moves around
		pEvadeAgentWander = std::make_unique<Wander>();
		pAgentToEvade->SetSteeringBehavior(pEvadeAgentWander.get());
	}


	pFlock = TUniquePtr<Flock>(
		new Flock(
			GetWorld(),
			SteeringAgentClass,
			FlockSize,
			TrimWorld->GetTrimWorldSize(),
			pAgentToEvade,
			true)
			);
}

// Called every frame
void ALevel_Flocking::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (pAgentToEvade)
	{
		pAgentToEvade->Tick(DeltaTime);
	}

	pFlock->ImGuiRender(WindowPos, WindowSize);
	pFlock->Tick(DeltaTime);
	pFlock->RenderDebug();
	if (bUseMouseTarget)
		pFlock->SetTarget_Seek(MouseTarget);
}

