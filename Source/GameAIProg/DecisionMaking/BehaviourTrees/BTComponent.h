#pragma once

#include <functional>
#include <memory>

#include "CoreMinimal.h"
#include "BrainComponent.h"
#include "DecisionMaking/BehaviourTrees/BehaviorTree.h"
#include "BTComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class GAMEAIPROG_API UBTComponent : public UBrainComponent
{
	GENERATED_BODY()

public:
	UBTComponent();

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	virtual void StartLogic() override;
	virtual void StopLogic(const FString& Reason) override;

	virtual bool IsRunning() const override;

	// Transfer the fully-constructed root node into this component
	void SetRoot(std::unique_ptr<GameAI::BT::Node>&& Root);

protected:
	virtual void BeginPlay() override;

private:
	std::unique_ptr<GameAI::BT::BehaviorTree> m_pBTInstance;
	bool m_IsRunning{ false };
};
