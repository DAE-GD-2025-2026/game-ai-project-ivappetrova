#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "ABTAIController.generated.h"

UCLASS()
class GAMEAIPROG_API ABTAIController : public AAIController
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AI|BT")
	TObjectPtr<UBlackboardData> m_BTBlackboardAsset;

	ABTAIController();
	virtual void Tick(float DeltaTime) override;

	void RunBehaviorTree();
	void InitBehaviorTree();

protected:
	virtual void BeginPlay() override;
};