#include "FSM.h"

namespace GameAI::FSM
{
	State* FSM::AddState(std::unique_ptr<State>&& NewState)
	{
		States.push_back(std::move(NewState));
		return States.back().get();
	}

	void FSM::AddTransition(State* From, State* To, std::function<bool()> Condition)
	{
		Transitions.push_back({ From, To, std::move(Condition) });
	}

	void FSM::Start(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (States.empty()) return;

		bRunning = true;
		TransitionTo(States.front().get(), Agent, Blackboard);
	}

	void FSM::Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (!bRunning || !CurrentState) return;

		// Evaluate transitions first (centralized model)
		for (auto& T : Transitions)
		{
			if (T.From == CurrentState && T.Condition && T.Condition())
			{
				TransitionTo(T.To, Agent, Blackboard);
				break; // only one transition per tick
			}
		}

		// Tick the (possibly new) current state
		if (CurrentState)
		{
			CurrentState->Update(DeltaTime, Agent, Blackboard);
		}
	}

	void FSM::Stop(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (CurrentState)
		{
			CurrentState->OnExit(Agent, Blackboard);
		}
		CurrentState = nullptr;
		bRunning = false;
	}

	void FSM::TransitionTo(State* NewState, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (CurrentState)
		{
			CurrentState->OnExit(Agent, Blackboard);
		}
		CurrentState = NewState;
		if (CurrentState)
		{
			CurrentState->OnEnter(Agent, Blackboard);
		}
	}

}