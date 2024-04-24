#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include "../../BehaviorTree.CPP/include/behaviortree_cpp/bt_factory.h"

using namespace BT;

class RandomWalk : public BT::SyncActionNode {
public:
	RandomWalk(const std::string& name, const BT::NodeConfig& config);

    BT::NodeStatus tick() override;

    // A node having ports MUST implement this STATIC method
    static BT::PortsList providedPorts();

};


#endif