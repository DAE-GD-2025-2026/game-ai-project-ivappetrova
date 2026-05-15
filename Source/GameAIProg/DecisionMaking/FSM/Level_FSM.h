// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "DecisionMaking/FSM/States/BlackboardKeys.h"
#include "Level_FSM.generated.h"

class Seek;
class UBlackboardComponent;

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
	GENERATED_BODY()

public:
	ALevel_FSM();
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

private:
	// Agents
	UPROPERTY()
	ASteeringAgent* Guard{ nullptr };

	UPROPERTY()
	ASteeringAgent* Thief{ nullptr };

	// Guard perception config 
	UPROPERTY(EditAnywhere, Category = "FSM|Guard")
	float DetectionRadius{ 400.f };

	UPROPERTY(EditAnywhere, Category = "FSM|Guard")
	float SearchTimeout{ 8.f };

	// Patrol route 
	UPROPERTY(EditAnywhere, Category = "FSM|Patrol")
	TArray<FVector2D> PatrolWaypoints;

	// Thief steering 
	Seek* ThiefSeek{ nullptr };
	TUniquePtr<Seek> ThiefSeekOwned;

	// Setup 
	void SetupGuard();
	void SetupThief();
	void WritePatrolWaypointsToBlackboard(UBlackboardComponent* BB) const;
	void DrawDebug() const;

	// Transition predicates
	bool IsTargetVisible() const;
	bool IsTargetNotVisible() const;
	bool IsSearchingTooLong() const;
};