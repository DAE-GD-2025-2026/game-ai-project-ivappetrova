#include "BehaviorTree.h"

namespace GameAI::BT
{
	////////////////////////////////////////////////////////////// Composite

	void Composite::AddChild(std::unique_ptr<Node>&& Child)
	{
		m_Children.push_back(std::move(Child));
	}


	////////////////////////////////////////////////////////////// Sequence

	void Sequence::OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		m_CurrentChildIndex = 0;
		if (!m_Children.empty())
		{
			m_Children[0]->OnEnter(Agent, Blackboard);
		}
	}

	ENodeStatus Sequence::Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		for (size_t index{}; index < m_Children.size(); ++index)
		{
			ENodeStatus Status = m_Children[index]->Tick(DeltaTime, Agent, Blackboard);

			if (Status == ENodeStatus::Failed)
			{
				if (index != m_CurrentChildIndex)
				{
					m_Children[m_CurrentChildIndex]->OnExit(Agent, Blackboard);
					m_CurrentChildIndex = 0;
				}
				return ENodeStatus::Failed;
			}

			if (Status == ENodeStatus::Running)
			{
				if (index != m_CurrentChildIndex)
				{	
					m_Children[m_CurrentChildIndex]->OnExit(Agent, Blackboard);
					m_CurrentChildIndex = index;
					m_Children[m_CurrentChildIndex]->OnEnter(Agent, Blackboard);
				}
				return ENodeStatus::Running;
			}

			// Succeeded — continue to next child
		}

		return ENodeStatus::Succeeded;
	}

	void Sequence::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		// If we were mid-sequence, exit the active child
		if (m_CurrentChildIndex < m_Children.size())
		{
			m_Children[m_CurrentChildIndex]->OnExit(Agent, Blackboard);
		}
		m_CurrentChildIndex = 0;
	}


	////////////////////////////////////////////////////////////// Selector

	void Selector::OnEnter(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		m_CurrentChildIndex = 0;
		if (!m_Children.empty())
		{
			m_Children[0]->OnEnter(Agent, Blackboard);
		}
	}

	ENodeStatus Selector::Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		// Check if any higher-priority child wants to run
		for (size_t index{}; index < m_CurrentChildIndex; ++index)
		{
			ENodeStatus higherStatus = m_Children[index]->Tick(DeltaTime, Agent, Blackboard);
			if (higherStatus != ENodeStatus::Failed)
			{
				// Higher priority child is active — exit current, switch to it
				m_Children[m_CurrentChildIndex]->OnExit(Agent, Blackboard);
				m_CurrentChildIndex = index;
				m_Children[m_CurrentChildIndex]->OnEnter(Agent, Blackboard);
				return ENodeStatus::Running;
			}
		}

		// Tick the current child
		while (m_CurrentChildIndex < m_Children.size())
		{
			ENodeStatus status = m_Children[m_CurrentChildIndex]->Tick(DeltaTime, Agent, Blackboard);

			if (status == ENodeStatus::Running)
			{
				return ENodeStatus::Running;
			}

			m_Children[m_CurrentChildIndex]->OnExit(Agent, Blackboard);

			if (status == ENodeStatus::Succeeded)
			{
				return ENodeStatus::Succeeded;
			}

			// Failed — try next child
			++m_CurrentChildIndex;
			if (m_CurrentChildIndex < m_Children.size())
			{
				m_Children[m_CurrentChildIndex]->OnEnter(Agent, Blackboard);
			}
		}

		return ENodeStatus::Failed;
	}

	void Selector::OnExit(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		// If we were mid-selection, exit the active child
		if (m_CurrentChildIndex < m_Children.size())
		{
			m_Children[m_CurrentChildIndex]->OnExit(Agent, Blackboard);
		}
		m_CurrentChildIndex = 0;
	}


	////////////////////////////////////////////////////////////// Condition

	Condition::Condition(std::function<bool()> Predicate)
		: m_Predicate(std::move(Predicate))
	{
	}

	ENodeStatus Condition::Tick(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		return (m_Predicate && m_Predicate()) ? ENodeStatus::Succeeded : ENodeStatus::Failed;
	}


	////////////////////////////////////////////////////////////// BehaviorTree

	void BehaviorTree::SetRoot(std::unique_ptr<Node>&& InRoot)
	{
		m_Root = std::move(InRoot);
	}

	void BehaviorTree::Start(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (!m_Root) return;

		m_IsRunning = true;
		m_Root->OnEnter(Agent, Blackboard);
	}

	void BehaviorTree::Update(float DeltaTime, ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (!m_IsRunning || !m_Root) return;

		ENodeStatus status = m_Root->Tick(DeltaTime, Agent, Blackboard);

		if (status != ENodeStatus::Running)
		{
			m_Root->OnExit(Agent, Blackboard);
			m_Root->OnEnter(Agent, Blackboard);
		}
	}

	void BehaviorTree::Stop(ASteeringAgent& Agent, UBlackboardComponent* Blackboard)
	{
		if (m_Root)
		{
			m_Root->OnExit(Agent, Blackboard);
		}
		m_IsRunning = false;
	}

}