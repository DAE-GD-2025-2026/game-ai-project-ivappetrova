// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "DecisionMaking/FSM/FSMComponent.h"
#include "DecisionMaking/GameAIController.h"
#include "States/PatrolState.h"
#include "States/ChaseState.h"
#include "States/SearchState.h"
#include "Kismet/GameplayStatics.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"


// Sets default values
ALevel_FSM::ALevel_FSM()
{
	PrimaryActorTick.bCanEverTick = true;

	PatrolWaypoints = {
	FVector2D{ -200.f,  630.f },
	FVector2D{ -200.f, -340.f },
	FVector2D{ -650.f, -340.f },
	FVector2D{ -650.f, -800.f },
	FVector2D{  800.f, -800.f },
	FVector2D{  800.f,  650.f } };
}

// Called when the game starts or when spawned
void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();
	
	SetupThief();
	SetupGuard();
}

// Called every frame
void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Move thief toward mouse
	if (Thief && ThiefSeekOwned)
	{
		FTargetData MouseTargetData{};
		MouseTargetData.Position = FVector2D{ LatestMouseWorldPos.X, LatestMouseWorldPos.Y };
		ThiefSeekOwned->SetTarget(MouseTargetData);
	}

	// Update guard's blackboard with THIEF's actual position & velocity (not mouse)
	if (Guard && Thief)
	{
		if (AGameAIController* GuardCtrl = Cast<AGameAIController>(Guard->GetController()))
		{
			if (UBlackboardComponent* BB = GuardCtrl->GetBlackboardComponent())
			{
				FVector2D ThiefPos = Thief->GetPosition();
				FVector2D ThiefVel = Thief->GetLinearVelocity();

				BB->SetValueAsVector(BB_TARGET_LOCATION,
					FVector{ ThiefPos.X, ThiefPos.Y, 0.f });
				BB->SetValueAsVector(TEXT("TargetVelocity"),
					FVector{ ThiefVel.X, ThiefVel.Y, 0.f });
			}
		}
	}

	DrawDebug();
}

void ALevel_FSM::DrawDebug() const
{
	if (!Guard || !GetWorld()) return;

	FVector GuardPos3D{ Guard->GetPosition().X, Guard->GetPosition().Y, 90.f };

	// Detection radius circle (green = not visible, red = visible)
	FColor RadiusColor = IsTargetVisible() ? FColor::Red : FColor::Green;
	DrawDebugCircle(GetWorld(), GuardPos3D, DetectionRadius,
		64, RadiusColor, false, -1.f, 0, 2.f,
		FVector(1, 0, 0), FVector(0, 1, 0)); // draw in XY plane

	// Line from guard to thief when chasing
	if (Thief && IsTargetVisible())
	{
		FVector ThiefPos3D{ Thief->GetPosition().X, Thief->GetPosition().Y, 90.f };
		DrawDebugLine(GetWorld(), GuardPos3D, ThiefPos3D, FColor::Orange, false, -1.f, 0, 2.f);
	}

	// Patrol waypoints
	for (int32 i = 0; i < PatrolWaypoints.Num(); ++i)
	{
		FVector WP{ PatrolWaypoints[i].X, PatrolWaypoints[i].Y, 90.f };
		DrawDebugSphere(GetWorld(), WP, 20.f, 8, FColor::Cyan, false, -1.f, 0, 1.5f);

		// Draw edges of patrol path
		int32 Next = (i + 1) % PatrolWaypoints.Num();
		FVector WPNext{ PatrolWaypoints[Next].X, PatrolWaypoints[Next].Y, 90.f };
		DrawDebugLine(GetWorld(), WP, WPNext, FColor::Cyan, false, -1.f, 0, 1.f);
	}

	// Last known location (yellow dot) — only meaningful during Search
	if (AGameAIController* GuardCtrl = Cast<AGameAIController>(Guard->GetController()))
	{
		if (UBlackboardComponent* BB = GuardCtrl->GetBlackboardComponent())
		{
			FVector LastKnown = BB->GetValueAsVector(BB_LAST_KNOWN);
			if (!LastKnown.IsNearlyZero())
			{
				DrawDebugSphere(GetWorld(), LastKnown + FVector(0, 0, 90.f),
					25.f, 8, FColor::Yellow, false, -1.f, 0, 2.f);
			}
		}
	}

}

void ALevel_FSM::SetupThief()
{
	Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 200.f, 0.f, 90.f }, FRotator::ZeroRotator);
	if (!Thief) return;

	Thief->SetDebugRenderingEnabled(true);
	Thief->SetMaxLinearSpeed(Thief->GetMaxLinearSpeed() * 1.5f);

	ThiefSeekOwned = MakeUnique<Seek>();
	ThiefSeek = ThiefSeekOwned.Get();
	Thief->SetSteeringBehavior(ThiefSeek);
}

void ALevel_FSM::SetupGuard()
{
	Guard = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0.f, 0.f, 90.f }, FRotator::ZeroRotator);
	if (!Guard) return;

	Guard->SetDebugRenderingEnabled(false);

	AGameAIController* AIController = Cast<AGameAIController>(Guard->GetController());
	if (!ensure(AIController)) return;

	UFSMComponent* FSMComp = Cast<UFSMComponent>(AIController->GetBrainComponent());
	if (!ensure(FSMComp)) return;

	UBlackboardComponent* BB = AIController->GetBlackboardComponent();

	// Write patrol waypoints into the blackboard
	WritePatrolWaypointsToBlackboard(BB);

	// Add states
	GameAI::FSM::State* Patrol = FSMComp->AddState(std::make_unique<GameAI::FSM::PatrolState>(PatrolWaypoints, 100.f));
	GameAI::FSM::State* Chase = FSMComp->AddState(std::make_unique<GameAI::FSM::ChaseState>());
	GameAI::FSM::State* Search = FSMComp->AddState(std::make_unique<GameAI::FSM::SearchState>(120.f));

	// Wire transitions
	FSMComp->AddTransition(Patrol, Chase, [this]() { return IsTargetVisible(); });
	FSMComp->AddTransition(Chase, Search, [this]() { return IsTargetNotVisible(); });
	FSMComp->AddTransition(Search, Chase, [this]() { return IsTargetVisible(); });
	FSMComp->AddTransition(Search, Patrol, [this]() { return IsSearchingTooLong(); });

	// Start
	AIController->RunFiniteStateMachine();
}

void ALevel_FSM::WritePatrolWaypointsToBlackboard(UBlackboardComponent* BB) const
{
	if (!BB) return;

	BB->SetValueAsInt(TEXT("WP_Count"), PatrolWaypoints.Num());
	for (int32 i = 0; i < PatrolWaypoints.Num(); ++i)
	{
		FName Key = *FString::Printf(TEXT("WP_%d"), i);
		BB->SetValueAsVector(Key, FVector{ PatrolWaypoints[i].X, PatrolWaypoints[i].Y, 0.f });
	}
}

// Transition predicates

bool ALevel_FSM::IsTargetVisible() const
{
	if (!Guard || !Thief) return false;

	float DistSq = FVector2D::DistSquared(Guard->GetPosition(), Thief->GetPosition());
	if (DistSq > DetectionRadius * DetectionRadius) return false;

	FHitResult Hit;
	FVector GuardEye{ Guard->GetPosition().X, Guard->GetPosition().Y, 80.f };
	FVector ThiefPos{ Thief->GetPosition().X, Thief->GetPosition().Y, 80.f };

	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Guard);
	Params.AddIgnoredActor(Thief);

	bool bBlocked = GetWorld()->LineTraceSingleByChannel(Hit, GuardEye, ThiefPos, ECC_Visibility, Params);
	return !bBlocked;
}

bool ALevel_FSM::IsTargetNotVisible() const
{
	return !IsTargetVisible();
}

bool ALevel_FSM::IsSearchingTooLong() const
{
	if (!Guard) return false;

	AGameAIController* AIController = Cast<AGameAIController>(Guard->GetController());
	if (!AIController) return false;

	UBlackboardComponent* BB = AIController->GetBlackboardComponent();
	if (!BB) return false;

	float SearchStart = BB->GetValueAsFloat(BB_SEARCH_START_TIME);
	float Now = GetWorld()->GetTimeSeconds();
	return (Now - SearchStart) >= SearchTimeout;
}


