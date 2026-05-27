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

ALevel_FSM::ALevel_FSM()
{
	PrimaryActorTick.bCanEverTick = true;

	m_PatrolWaypoints = {
	FVector2D{ -200.f,  630.f },
	FVector2D{ -200.f, -340.f },
	FVector2D{ -650.f, -340.f },
	FVector2D{ -650.f, -800.f },
	FVector2D{  800.f, -800.f },
	FVector2D{  800.f,  650.f } };
}

void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();
	
	SetupThief();
	SetupGuard();
}

void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Move thief toward mouse
	if (m_pThief && m_ThiefSeekOwned)
	{
		FTargetData mouseTargetData{};
		mouseTargetData.Position = FVector2D{ LatestMouseWorldPos.X, LatestMouseWorldPos.Y };
		m_ThiefSeekOwned->SetTarget(mouseTargetData);
	}

	// Update guard's blackboard with THIEF's actual position & velocity (not mouse)
	if (m_pGuard && m_pThief)
	{
		if (IsTargetVisible())
		{
			m_HasEverChased = true;
		}

		if (AGameAIController* guardController = Cast<AGameAIController>(m_pGuard->GetController()))
		{
			if (UBlackboardComponent* pBB = guardController->GetBlackboardComponent())
			{
				FVector2D thiefPos = m_pThief->GetPosition();
				FVector2D thiefVel = m_pThief->GetLinearVelocity();

				pBB->SetValueAsVector(BB_TARGET_LOCATION,
					FVector{ thiefPos.X, thiefPos.Y, 0.f });
				pBB->SetValueAsVector(TEXT("TargetVelocity"),
					FVector{ thiefVel.X, thiefVel.Y, 0.f });
			}
		}
	}

	DrawDebug();
}

void ALevel_FSM::DrawDebug() const
{
	if (!m_pGuard || !GetWorld()) return;

	FVector guardPos3D{ m_pGuard->GetPosition().X, m_pGuard->GetPosition().Y, 90.f };

	// Detection radius circle (green = not visible, red = visible)
	FColor radiusColor;
	if (IsTargetVisible())
	{
		radiusColor = FColor::Red;
	}
	else if (m_HasEverChased && !IsSearchingTooLong())
	{
		radiusColor = FColor::Blue;
	}
	else
	{
		radiusColor = FColor::Green;
	}
	DrawDebugCircle(GetWorld(), guardPos3D, m_DetectionRadius, 64, radiusColor, false, -1.f, 0, 2.f, 
					FVector(1, 0, 0), FVector(0, 1, 0)); // draw in XY plane

	// Line from guard to thief when chasing
	if (m_pThief && IsTargetVisible())
	{
		FVector thiefPos3D{ m_pThief->GetPosition().X, m_pThief->GetPosition().Y, 90.f };
		DrawDebugLine(GetWorld(), guardPos3D, thiefPos3D, FColor::Orange, false, -1.f, 0, 2.f);
	}

	// Patrol waypoints
	for (size_t index{}; index < m_PatrolWaypoints.Num(); ++index)
	{
		FVector WP{ m_PatrolWaypoints[index].X, m_PatrolWaypoints[index].Y, 90.f };
		DrawDebugSphere(GetWorld(), WP, 20.f, 8, FColor::Cyan, false, -1.f, 0, 1.5f);

		// Draw edges of patrol path
		size_t next = (index + 1) % m_PatrolWaypoints.Num();
		FVector wpNext{ m_PatrolWaypoints[next].X, m_PatrolWaypoints[next].Y, 90.f };
		DrawDebugLine(GetWorld(), WP, wpNext, FColor::Cyan, false, -1.f, 0, 1.f);
	}

	// Last known location (yellow dot) — only meaningful during Search
	if (AGameAIController* guardController = Cast<AGameAIController>(m_pGuard->GetController()))
	{
		if (UBlackboardComponent* pBB = guardController->GetBlackboardComponent())
		{
			FVector lastKnown = pBB->GetValueAsVector(BB_LAST_KNOWN);
			if (!lastKnown.IsNearlyZero())
			{
				DrawDebugSphere(GetWorld(), lastKnown + FVector(0, 0, 90.f), 25.f, 8, FColor::Yellow, false, -1.f, 0, 2.f);
			}
		}
	}

}

void ALevel_FSM::SetupThief()
{
	m_pThief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 200.f, 0.f, 90.f }, FRotator::ZeroRotator);
	if (!m_pThief) return;

	m_pThief->SetDebugRenderingEnabled(true);
	m_pThief->SetMaxLinearSpeed(m_pThief->GetMaxLinearSpeed() * 1.5f);
	m_ThiefSeekOwned = MakeUnique<Seek>();
	m_ThiefSeek = m_ThiefSeekOwned.Get();
	m_pThief->SetSteeringBehavior(m_ThiefSeek);
}

void ALevel_FSM::SetupGuard()
{
	m_pGuard = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0.f, 0.f, 90.f }, FRotator::ZeroRotator);
	if (!m_pGuard) return;

	m_pGuard->SetDebugRenderingEnabled(false);

	AGameAIController* pAIController = Cast<AGameAIController>(m_pGuard->GetController());
	if (!ensure(pAIController)) return;

	UFSMComponent* pFSMComp = Cast<UFSMComponent>(pAIController->GetBrainComponent());
	if (!ensure(pFSMComp)) return;

	UBlackboardComponent* pBB = pAIController->GetBlackboardComponent();

	// Write patrol waypoints into the blackboard
	WritePatrolWaypointsToBlackboard(pBB);

	// Add states
	GameAI::FSM::State* pPatrol = pFSMComp->AddState(std::make_unique<GameAI::FSM::PatrolState>(m_PatrolWaypoints, 100.f));
	GameAI::FSM::State* pChase = pFSMComp->AddState(std::make_unique<GameAI::FSM::ChaseState>());
	GameAI::FSM::State* pSearch = pFSMComp->AddState(std::make_unique<GameAI::FSM::SearchState>(120.f));

	// Wire transitions
	pFSMComp->AddTransition(pPatrol, pChase, [this]() { return IsTargetVisible(); });
	pFSMComp->AddTransition(pChase, pSearch, [this]() { return IsTargetNotVisible(); });
	pFSMComp->AddTransition(pSearch, pChase, [this]() { return IsTargetVisible(); });
	pFSMComp->AddTransition(pSearch, pPatrol, [this]() { return IsSearchingTooLong(); });

	// Start
	pAIController->RunFiniteStateMachine();
}

void ALevel_FSM::WritePatrolWaypointsToBlackboard(UBlackboardComponent* BB) const
{
	if (!BB) return;

	BB->SetValueAsInt(TEXT("WP_Count"), m_PatrolWaypoints.Num());
	for (size_t index{}; index < m_PatrolWaypoints.Num(); ++index)
	{
		FName key = *FString::Printf(TEXT("WP_%d"), index);
		BB->SetValueAsVector(key, FVector{ m_PatrolWaypoints[index].X, m_PatrolWaypoints[index].Y, 0.f });
	}
}

// Transition predicates

bool ALevel_FSM::IsTargetVisible() const
{
	if (!m_pGuard || !m_pThief) return false;

	float distSq = FVector2D::DistSquared(m_pGuard->GetPosition(), m_pThief->GetPosition());
	if (distSq > m_DetectionRadius * m_DetectionRadius) return false;

	FHitResult hit;
	FVector guardEye{ m_pGuard->GetPosition().X, m_pGuard->GetPosition().Y, 80.f };
	FVector thiefPos{ m_pThief->GetPosition().X, m_pThief->GetPosition().Y, 80.f };

	FCollisionQueryParams params;
	params.AddIgnoredActor(m_pGuard);
	params.AddIgnoredActor(m_pThief);
	bool bBlocked = GetWorld()->LineTraceSingleByChannel(hit, guardEye, thiefPos, ECC_Visibility, params);
	return !bBlocked;
}

bool ALevel_FSM::IsTargetNotVisible() const
{
	return !IsTargetVisible();
}

bool ALevel_FSM::IsSearchingTooLong() const
{
	if (!m_pGuard) return false;

	AGameAIController* pAIController = Cast<AGameAIController>(m_pGuard->GetController());
	if (!pAIController) return false;

	UBlackboardComponent* pBB = pAIController->GetBlackboardComponent();
	if (!pBB) return false;

	float searchStart = pBB->GetValueAsFloat(BB_SEARCH_START_TIME);
	float now = GetWorld()->GetTimeSeconds();
	return (now - searchStart) >= m_SearchTimeout;
}


