#pragma once

#include <functional>
#include <memory>
#include <vector>

class ASteeringAgent;
class UBlackboardComponent;

namespace GameAI::FSM
{
	////////////////////////////////////////////////////////////// State base class
	class State
	{
	public:
		virtual ~State() = default;

		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) {}
		virtual void Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) {}
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) {}

	};


	////////////////////////////////////////////////////////////// Transition- links two states
	struct Transition
	{
		State* From{ nullptr };   // non-owning ptr into States vector
		State* To{ nullptr };     // non-owning ptr into States vector
		std::function<bool()> Condition;
	};


	///////////////////////////////////////////////////////////// FSM — owns states, ticks active state, evaluates transitions
	class FSM
	{
	public:
		FSM() = default;
		~FSM() = default;

		State* AddState(std::unique_ptr<State>&& NewState);
		void AddTransition(State* From, State* To, std::function<bool()> Condition);
		void Start(ASteeringAgent& Agent, UBlackboardComponent* Blackboard);
		void Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard);
		void Stop(ASteeringAgent& Agent, UBlackboardComponent* Blackboard);

		bool IsRunning() const { return bRunning; }
		State* GetCurrentState() const { return CurrentState; }

	private:
		std::vector<std::unique_ptr<State>> States;
		std::vector<Transition> Transitions;

		State* CurrentState{ nullptr };
		bool bRunning{ false };

		void TransitionTo(State* NewState, ASteeringAgent& Agent, UBlackboardComponent* Blackboard);
	};

}