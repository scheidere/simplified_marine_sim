#include "behavior_tree.hpp"
#include <iostream> // For std::cout
#include <string>   // For std::string

using namespace BT;

RandomWalk::RandomWalk(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus RandomWalk::tick()
{
    std::cout << "hi from BT" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// A node having ports MUST implement this STATIC method
BT::PortsList RandomWalk::providedPorts()
{
    return { BT::OutputPort<std::string>("text") };
}