#include "behavior_tree.hpp"
#include <iostream> // For std::cout
#include <string>   // For std::string
#include <opencv2/opencv.hpp>
#include "world.hpp"
#include "robot.hpp"
#include "planners.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"
#include "behaviortree_cpp/blackboard.h"


using namespace BT;

struct Pose2D
{
  double x, y, theta;
};

RandomWalk::RandomWalk(const std::string& name, const NodeConfig& config, RandomWalkPlanner& rwp, World& w, Robot& r, cv::Mat background)
    : SyncActionNode(name, config), _rwp(rwp), _world(w), _robot(r), _background(background)
  {}

NodeStatus RandomWalk::tick()
{
    // This is not how this should be implemented longterm
    // The movement should not slow down ticking
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



GenerateWaypoints::GenerateWaypoints(const std::string& name, const NodeConfig& config, RandomWalkPlanner& rwp, World& w, Robot& r, cv::Mat background)
    : SyncActionNode(name, config), _rwp(rwp), _world(w), _robot(r), _background(background)
  {}

NodeStatus GenerateWaypoints::tick()
{
    auto shared_queue = std::make_shared<ProtectedQueue<Pose2D>>();
    // Generate waypoints and push them into the shared queue
    for (int i = 0; i < 5; ++i) {
        //cv::Point waypoint(i, i);
        Pose2D waypoint{ double(i), double(i), 0 };
        shared_queue->items.push_back(waypoint);
        std::cout << "Generated waypoint: (" << waypoint.x << ", " << waypoint.y << ", " << waypoint.theta << ")" << std::endl;
    }
    setOutput("waypoints", shared_queue);
    return NodeStatus::SUCCESS;
}

PortsList GenerateWaypoints::providedPorts()
{
    std::cout << "SharedQueue is outputted..." << std::endl;
    //return { OutputPort<SharedQueue<cv::Point>>("waypoints") };
    //return { OutputPort<SharedQueue<Pose2D>>("waypoints") };
    return { OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("waypoints") };
}

UseWaypoint::UseWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r, cv::Mat background)
    : ThreadedAction(name, config), _world(w), _robot(r), _background(background)
  {}

NodeStatus UseWaypoint::tick()
{
    /*cv::Point wp;
    if(getInput("waypoint", wp)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Using waypoint: " << wp.x << "/" << wp.y << std::endl;
        return NodeStatus::SUCCESS;
    }
    else {
        std::cout << "no input in use_wp" << std::endl;
        return NodeStatus::FAILURE;
    }*/
    Pose2D wp;
    if(getInput("waypoint", wp))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::cout << "Using waypoint: " << wp.x << "/" << wp.y << std::endl;
      return NodeStatus::SUCCESS;
    }
    else
    {
      std::cout << "no input in use_wp" << std::endl;
      return NodeStatus::FAILURE;
    }

}

PortsList UseWaypoint::providedPorts()
{
    //return { InputPort<cv::Point>("waypoint") };
    return { InputPort<Pose2D>("waypoint") };
    //return {};
}