#pragma once

#include <functional>
#include <memory>
#include <vector>

class ASteeringAgent;
class UBlackboardComponent;

namespace GameAI::BT
{
	////////////////////////////////////////////////////////////// Node status
	enum class ENodeStatus : uint8
	{
		Running,
		Succeeded,
		Failed
	};


	////////////////////////////////////////////////////////////// Node base class
	class Node
	{
	public:
		virtual ~Node() = default;

		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) {}
		virtual ENodeStatus Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) = 0;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) {}
	};


	////////////////////////////////////////////////////////////// Composite base - owns child nodes
	class Composite : public Node
	{
	public:
		void AddChild(std::unique_ptr<Node>&& Child);

	protected:
		std::vector<std::unique_ptr<Node>> m_Children;
	};


	////////////////////////////////////////////////////////////// Sequence
	class Sequence : public Composite
	{
	public:
		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual ENodeStatus Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		size_t m_CurrentChildIndex{ 0 };
	};


	////////////////////////////////////////////////////////////// Selector
	class Selector : public Composite
	{
	public:
		virtual void OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual ENodeStatus Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;
		virtual void OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		size_t m_CurrentChildIndex{ 0 };
	};


	////////////////////////////////////////////////////////////// Action
	// Subclass this to implement concrete behaviors (Patrol, Chase, Search)
	class Action : public Node
	{
		// Tick() must return Running, Succeeded, or Failed.
	};


	////////////////////////////////////////////////////////////// Condition
	// Returns Succeeded when the predicate is true, Failed otherwise.
	class Condition : public Node
	{
	public:
		explicit Condition(std::function<bool()> Predicate);
		virtual ENodeStatus Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard) override;

	private:
		std::function<bool()> m_Predicate;
	};


	////////////////////////////////////////////////////////////// BehaviorTree
	// Owns the root, drives the tree
	class BehaviorTree
	{
	public:
		BehaviorTree() = default;
		~BehaviorTree() = default;

		// Transfer ownership of the root node into the tree
		void SetRoot(std::unique_ptr<Node>&& Root);

		void Start(ASteeringAgent& Agent, UBlackboardComponent* Blackboard);
		void Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard);
		void Stop(ASteeringAgent& Agent, UBlackboardComponent* Blackboard);

		bool IsRunning() const { return m_IsRunning; }

	private:
		std::unique_ptr<Node> m_Root;
		bool m_IsRunning{ false };
	};

}
