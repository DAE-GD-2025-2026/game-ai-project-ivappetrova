#include "BTComponent.h"

#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"


UBTComponent::UBTComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	m_pBTInstance = std::make_unique<GameAI::BT::BehaviorTree>();
}

void UBTComponent::SetRoot(std::unique_ptr<GameAI::BT::Node>&& Root)
{
	check(m_pBTInstance);
	m_pBTInstance->SetRoot(std::move(Root));
}

void UBTComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UBTComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!m_IsRunning || !m_pBTInstance) return;

	AAIController* pController = Cast<AAIController>(GetOwner());
	if (!pController) return;

	ASteeringAgent* pAgent = Cast<ASteeringAgent>(pController->GetPawn());
	UBlackboardComponent* pBB = pController->GetBlackboardComponent();
	if (pAgent)
	{
		m_pBTInstance->Update(DeltaTime, *pAgent, pBB);
	}
}

void UBTComponent::StartLogic()
{
	Super::StartLogic();

	if (!m_pBTInstance) return;

	AAIController* pController = Cast<AAIController>(GetOwner());
	if (!pController) return;

	ASteeringAgent* pAgent = Cast<ASteeringAgent>(pController->GetPawn());
	UBlackboardComponent* pBB = pController->GetBlackboardComponent();

	if (pAgent)
	{
		m_pBTInstance->Start(*pAgent, pBB);
		m_IsRunning = true;
	}
}

void UBTComponent::StopLogic(const FString& Reason)
{
	if (!m_pBTInstance) return;

	AAIController* pController = Cast<AAIController>(GetOwner());
	if (!pController) return;

	ASteeringAgent* pAgent = Cast<ASteeringAgent>(pController->GetPawn());
	UBlackboardComponent* pBB = pController->GetBlackboardComponent();

	if (pAgent)
	{
		m_pBTInstance->Stop(*pAgent, pBB);
	}

	m_IsRunning = false;
}

bool UBTComponent::IsRunning() const
{
	return m_IsRunning;
}
