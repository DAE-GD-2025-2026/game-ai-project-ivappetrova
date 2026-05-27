#pragma once

#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "DecisionMaking/BlackboardKeys.h"
#include "ABTAIController.h"
#include "Level_BT.generated.h"

class Seek;
class UBlackboardComponent;

UCLASS()
class GAMEAIPROG_API ALevel_BT : public ALevel_Base
{
	GENERATED_BODY()

public:
	ALevel_BT();
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

private:
	// Agents
	UPROPERTY()
	ASteeringAgent* m_pGuard{ nullptr };

	UPROPERTY()
	ASteeringAgent* m_pThief{ nullptr };

	// Guard perception config
	UPROPERTY(EditAnywhere, Category = "BT|Guard")
	float m_DetectionRadius{ 400.f };

	UPROPERTY(EditAnywhere, Category = "BT|Guard")
	float m_SearchTimeout{ 8.f };

	// Patrol route
	UPROPERTY(EditAnywhere, Category = "BT|Patrol")
	TArray<FVector2D> m_PatrolWaypoints;
	UPROPERTY(EditAnywhere, Category = "BT|Guard")
	TSubclassOf<ABTAIController> m_AIControllerClass;

	// Thief steering
	Seek* m_pThiefSeek{ nullptr };
	TUniquePtr<Seek> m_pThiefSeekOwned;

	// Set to true the first time ChaseAction runs — gates the Search branch
	bool m_HasEverChased{ false };
	// Setup
	void SetupGuard();
	void SetupThief();
	void WritePatrolWaypointsToBlackboard(UBlackboardComponent* BB) const;
	void DrawDebug() const;

	// Predicates used by BT Condition nodes
	bool IsTargetVisible() const;
	bool IsTargetNotVisible() const;
	bool IsSearchingTooLong() const;
};