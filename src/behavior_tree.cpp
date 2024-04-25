#include "behavior_tree.hpp"
#include <iostream> // For std::cout
#include <string>   // For std::string
#include "world.hpp"
#include "robot.hpp"
#include "planners.hpp"

using namespace BT;

/*RandomWalk::RandomWalk(const std::string& name, const BT::NodeConfig& config, RandomWalkPlanner* rwp) :
    BT::SyncActionNode(name, config, rwp)
{}*/

RandomWalk::RandomWalk(const std::string& name, const NodeConfig& config, RandomWalkPlanner& rwp, World& w, Robot& r, cv::Mat background)
    : SyncActionNode(name, config), _rwp(rwp), _world(w), _robot(r), _background(background)
  {}

NodeStatus RandomWalk::tick()
{
    std::cout << "Running RandomWalk..." << std::endl;
    int steps = 10;
    _rwp.performRandomWalk(_world, _background, _robot, steps);
    return NodeStatus::SUCCESS;
}

// A node having ports MUST implement this STATIC method
PortsList RandomWalk::providedPorts()
{
    return { OutputPort<std::string>("text") };
}