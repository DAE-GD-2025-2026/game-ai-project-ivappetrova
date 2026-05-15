// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"

#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "FSM.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"


// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// TODO Setup FSM
	FSMInstance = std::make_unique<GameAI::FSM::FSM>();
}


GameAI::FSM::State* UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	check(FSMInstance);
	return FSMInstance->AddState(std::move(NewState));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> Condition) const
{
	check(FSMInstance);
	FSMInstance->AddTransition(From, To, std::move(Condition));
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	
	if (!bIsRunning || !FSMInstance) return;

	// Resolve agent and blackboard from my own AI controller
	AAIController* Controller = Cast<AAIController>(GetOwner());
	if (!Controller) return;

	ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn());
	UBlackboardComponent* BB = Controller->GetBlackboardComponent();

	if (Agent)
	{
		FSMInstance->Update(DeltaTime, *Agent, BB);
	}
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();

	if (!FSMInstance) return;

	AAIController* Controller = Cast<AAIController>(GetOwner());
	if (!Controller) return;

	ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn());
	UBlackboardComponent* BB = Controller->GetBlackboardComponent();

	if (Agent)
	{
		FSMInstance->Start(*Agent, BB);
		bIsRunning = true;
	}
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	if (!FSMInstance) return;

	AAIController* Controller = Cast<AAIController>(GetOwner());
	if (!Controller) return;

	ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn());
	UBlackboardComponent* BB = Controller->GetBlackboardComponent();

	if (Agent)
	{
		FSMInstance->Stop(*Agent, BB);
	}

	bIsRunning = false;
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}


