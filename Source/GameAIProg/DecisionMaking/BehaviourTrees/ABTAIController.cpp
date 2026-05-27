#include "ABTAIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "DecisionMaking/BehaviourTrees/BTComponent.h"

ABTAIController::ABTAIController()
{
	PrimaryActorTick.bCanEverTick = true;
	BrainComponent = CreateDefaultSubobject<UBTComponent>(TEXT("BTComponent"));
}

void ABTAIController::BeginPlay()
{
	Super::BeginPlay();
	InitBehaviorTree();
}

void ABTAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ABTAIController::InitBehaviorTree()
{
	UBTComponent* pBTComp = Cast<UBTComponent>(BrainComponent);
	if (ensure(pBTComp) && m_BTBlackboardAsset)
	{
		UBlackboardComponent* pBlackboardComp = Blackboard;
		UseBlackboard(m_BTBlackboardAsset, pBlackboardComp);
		Blackboard = pBlackboardComp;
	}
}

void ABTAIController::RunBehaviorTree()
{
	UBTComponent* pBTComp = Cast<UBTComponent>(BrainComponent);
	if (ensure(pBTComp))
	{
		pBTComp->StartLogic();
	}
}