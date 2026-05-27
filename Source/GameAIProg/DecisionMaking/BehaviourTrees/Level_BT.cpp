#include "Level_BT.h"

#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "BTComponent.h"
#include "Actions/BTPatrolAction.h"
#include "Actions/BTChaseAction.h"
#include "Actions/BTSearchAction.h"
#include "ABTAIController.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

// Sets default values
ALevel_BT::ALevel_BT()
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

// Called when the game starts or when spawned
void ALevel_BT::BeginPlay()
{
	Super::BeginPlay();

	SetupThief();
	SetupGuard();
}

// Called every frame
void ALevel_BT::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Move thief toward mouse
	if (m_pThief && m_pThiefSeekOwned)
	{
		FTargetData mouseTargetData{};
		mouseTargetData.Position = FVector2D{ LatestMouseWorldPos.X, LatestMouseWorldPos.Y };
		m_pThiefSeekOwned->SetTarget(mouseTargetData);
	}

	// Push thief position & velocity into the guard's blackboard only when visible
	if (m_pGuard && m_pThief && IsTargetVisible())
	{
		m_HasEverChased = true;

		if (ABTAIController* pGuardCtrl = Cast<ABTAIController>(m_pGuard->GetController()))
		{
			if (UBlackboardComponent* pBB = pGuardCtrl->GetBlackboardComponent())
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

void ALevel_BT::DrawDebug() const
{
	if (!m_pGuard || !GetWorld()) return;

	FVector guardPos3D{ m_pGuard->GetPosition().X, m_pGuard->GetPosition().Y, 90.f };

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

	DrawDebugCircle(GetWorld(), guardPos3D, m_DetectionRadius, 64, radiusColor, false, -1.f, 0, 2.f, FVector(1, 0, 0), FVector(0, 1, 0));

	// Line to thief when chasing
	if (m_pThief && IsTargetVisible())
	{
		FVector thiefPos3D{ m_pThief->GetPosition().X, m_pThief->GetPosition().Y, 90.f };
		DrawDebugLine(GetWorld(), guardPos3D, thiefPos3D, FColor::Orange, false, -1.f, 0, 2.f);
	}

	// Patrol waypoints
	for (int32 index{ }; index < m_PatrolWaypoints.Num(); ++index)
	{
		FVector wp{ m_PatrolWaypoints[index].X, m_PatrolWaypoints[index].Y, 90.f };
		DrawDebugSphere(GetWorld(), wp, 20.f, 8, FColor::Cyan, false, -1.f, 0, 1.5f);

		int32 next = (index + 1) % m_PatrolWaypoints.Num();
		FVector wpNext{ m_PatrolWaypoints[next].X, m_PatrolWaypoints[next].Y, 90.f };
		DrawDebugLine(GetWorld(), wp, wpNext, FColor::Cyan, false, -1.f, 0, 1.f);
	}

	// Last-known location (yellow dot)
	if (ABTAIController* pGuardCtrl = Cast<ABTAIController>(m_pGuard->GetController()))
	{
		if (UBlackboardComponent* pBB = pGuardCtrl->GetBlackboardComponent())
		{
			FVector LastKnown = pBB->GetValueAsVector(BB_LAST_KNOWN);
			if (!LastKnown.IsNearlyZero())
			{
				DrawDebugSphere(GetWorld(), LastKnown + FVector(0, 0, 90.f), 25.f, 8, FColor::Yellow, false, -1.f, 0, 2.f);
			}
		}
	}
}

void ALevel_BT::SetupThief()
{
	m_pThief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 200.f, 0.f, 90.f }, FRotator::ZeroRotator);
	if (!m_pThief) return;

	m_pThief->SetDebugRenderingEnabled(true);
	m_pThief->SetMaxLinearSpeed(m_pThief->GetMaxLinearSpeed() * 1.5f);
	m_pThiefSeekOwned = MakeUnique<Seek>();
	m_pThiefSeek = m_pThiefSeekOwned.Get();
	m_pThief->SetSteeringBehavior(m_pThiefSeek);
}

void ALevel_BT::SetupGuard()
{
	m_pGuard = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass,
		FVector{ -200.f, 630.f, 90.f },
		FRotator::ZeroRotator );
	if (!m_pGuard) return;

	m_pGuard->SetDebugRenderingEnabled(false);

	// Detach whatever controller auto-possessed the guard
	if (AController* pOldController = m_pGuard->GetController())
	{
		pOldController->UnPossess();
	}

	// Spawn and attach the correct controller
	ABTAIController* pAIController = GetWorld()->SpawnActor<ABTAIController>(m_AIControllerClass);
	if (!ensure(pAIController)) return;
	pAIController->Possess(m_pGuard);

	UBTComponent* pBTComp = Cast<UBTComponent>(pAIController->GetBrainComponent());
	if (!ensure(pBTComp)) return;

	pAIController->InitBehaviorTree();

	UBlackboardComponent* pBB = pAIController->GetBlackboardComponent();
	WritePatrolWaypointsToBlackboard(pBB);

	if (pBB)
	{
		pBB->SetValueAsFloat(BB_SEARCH_START_TIME, -999.f);
	}

	// Chase branch: visible → chase
	auto chaseSequence = std::make_unique<GameAI::BT::Sequence>();
	chaseSequence->AddChild(std::make_unique<GameAI::BT::Condition>([this]() { return IsTargetVisible(); }));
	chaseSequence->AddChild(std::make_unique<GameAI::BT::ChaseAction>());

	// Search branch: not visible + has chased before → search
	auto searchSequence = std::make_unique<GameAI::BT::Sequence>();
	searchSequence->AddChild(std::make_unique<GameAI::BT::Condition>([this]() { return IsTargetNotVisible(); }));
	searchSequence->AddChild(std::make_unique<GameAI::BT::Condition>([this]() { return m_HasEverChased; }));
	searchSequence->AddChild(std::make_unique<GameAI::BT::SearchAction>(120.f, m_SearchTimeout));

	// Root selector: Chase > Search > Patrol
	auto root = std::make_unique<GameAI::BT::Selector>();
	root->AddChild(std::move(chaseSequence));
	root->AddChild(std::move(searchSequence));
	root->AddChild(std::make_unique<GameAI::BT::PatrolAction>(m_PatrolWaypoints, 100.f));

	pBTComp->SetRoot(std::move(root));

	// Start
	pAIController->RunBehaviorTree();
}

void ALevel_BT::WritePatrolWaypointsToBlackboard(UBlackboardComponent* BB) const
{
	if (!BB) return;

	BB->SetValueAsInt(TEXT("WP_Count"), m_PatrolWaypoints.Num());
	for (int32 index{}; index < m_PatrolWaypoints.Num(); ++index)
	{
		FName Key = *FString::Printf(TEXT("WP_%d"), index);
		BB->SetValueAsVector(Key, FVector{ m_PatrolWaypoints[index].X, m_PatrolWaypoints[index].Y, 0.f });
	}
}

bool ALevel_BT::IsTargetVisible() const
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

	bool blocked = GetWorld()->LineTraceSingleByChannel(hit, guardEye, thiefPos, ECC_Visibility, params);
	return !blocked;
}

bool ALevel_BT::IsTargetNotVisible() const
{
	return !IsTargetVisible();
}

bool ALevel_BT::IsSearchingTooLong() const
{
	if (!m_pGuard) return false;

	ABTAIController* pAIController = Cast<ABTAIController>(m_pGuard->GetController());
	if (!pAIController) return false;

	UBlackboardComponent* pBB = pAIController->GetBlackboardComponent();
	if (!pBB) return false;

	float searchStart = pBB->GetValueAsFloat(BB_SEARCH_START_TIME);
	if (searchStart < 0.f) return false;

	return (GetWorld()->GetTimeSeconds() - searchStart) >= m_SearchTimeout;
}