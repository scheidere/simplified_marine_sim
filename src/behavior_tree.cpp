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

GenerateWaypoints::GenerateWaypoints(const std::string& name, const NodeConfig& config, World& w, Robot& r, cv::Mat image)
    : SyncActionNode(name, config), _world(w), _robot(r), _image(image)
  {}

NodeStatus GenerateWaypoints::tick()
{
    auto shared_queue = std::make_shared<ProtectedQueue<Pose2D>>();
    // Generate waypoints and push them into the shared queue
    for (int i = 0; i < 5; ++i) {
        //cv::Point waypoint(i, i);
        Pose2D waypoint{ i, i, 0 };
        shared_queue->items.push_back(waypoint);
        std::cout << "Generated waypoint: (" << waypoint.x << ", " << waypoint.y << ", " << waypoint.theta << ")" << std::endl;
    }
    setOutput("waypoints", shared_queue);
    return NodeStatus::SUCCESS;
}

PortsList GenerateWaypoints::providedPorts()
{
    std::cout << "SharedQueue is outputted..." << std::endl;
    return { OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("waypoints") };
}

GenerateNextWaypoint::GenerateNextWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r, cv::Mat image)
    : SyncActionNode(name, config), _world(w), _robot(r), _image(image)
  {}

NodeStatus GenerateNextWaypoint::tick()
{

    // Get robot location
    int x  = _robot.getX(); int y = _robot.getY();
    Pose2D current_waypoint{x, y, 0}; // do we even need theta for this sim?
    std::cout << current_waypoint.x << " and " << current_waypoint.y << std::endl;
    auto shared_queue = std::make_shared<ProtectedQueue<Pose2D>>();
    // Generate waypoint randomly one pixel (step) away from current robot location
    // Get random +/- combo and add or subtract from x/y location accordingly
    int dx = 10; int dy = -10; // Hardcoded, not random
    Pose2D next_waypoint{ current_waypoint.x + dx, current_waypoint.y + dy, 0 };
    shared_queue->items.push_back(next_waypoint);
    std::cout << "Generated waypoint: (" << next_waypoint.x << ", " << next_waypoint.y << ", " << next_waypoint.theta << ")" << std::endl;

    setOutput("next_waypoint", shared_queue);
    return NodeStatus::SUCCESS;
}

PortsList GenerateNextWaypoint::providedPorts()
{
    std::cout << "SharedQueue is outputted..." << std::endl;
    return { OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("next_waypoint") };
}

// Add mutex handling for image
PlanShortestPath::PlanShortestPath(const std::string& name, const NodeConfig& config, World& w, Robot& r, ShortestPath& sp, cv::Mat& image)
    : SyncActionNode(name, config), _world(w), _robot(r), _shortest_path(sp), _image(image)
  {}

NodeStatus PlanShortestPath::tick()
{
    // How long is a tick? What happens if the planning takes longer than a tick *should*?
    Pose2D current_pose = _robot.getPose();
    //Pose2D waypoint{0,30,0}; Passing this in as "goal" now
    Pose2D goal_pose = _robot.getGoalPose();
    std::shared_ptr<ProtectedQueue<Pose2D>> plan = _shortest_path.plan(current_pose, goal_pose,_world.getX(), _world.getY());

    // Testing path follow without shortest path
    /*auto plan = std::make_shared<BT::ProtectedQueue<Pose2D>>();
    plan->items.push_back(Pose2D{0, 0, 0});   // Start point
    plan->items.push_back(Pose2D{1, 1, 0});   // Next point
    plan->items.push_back(Pose2D{2, 2, 0});   // Next point
    plan->items.push_back(Pose2D{3, 3, 0});   // Next point
    plan->items.push_back(Pose2D{4, 4, 0});   // Next point
    plan->items.push_back(Pose2D{5, 5, 0});   // End point*/

    //cv::Point p1(5,5);
    //_shortest_path.initializeDistances(_image, p1); // Commenting out to test calling from constructor in planners.cpp

    setOutput("path", plan);
    return NodeStatus::SUCCESS;
}

PortsList PlanShortestPath::providedPorts()
{
    std::cout << "Shortest path is outputted..." << std::endl;
    return { OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("path") };
}

// Add mutex handling for image
UseWaypoint::UseWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r, cv::Mat& image, std::mutex& image_mutex)
    : ThreadedAction(name, config), _world(w), _robot(r), _image(image),  _image_mutex(image_mutex)
  {}

NodeStatus UseWaypoint::tick()
{

    Pose2D wp;
    if(getInput("waypoint", wp))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::cout << "Using waypoint: " << wp.x << "/" << wp.y << std::endl;
      std::cout << "Robot prev loc: " << _robot.getX() << "/" << _robot.getY() << std::endl;
      _robot.move(wp, _image, _image_mutex); // Publish new waypoint to image, i.e. move robot
      std::cout << "Robot new loc: " << _robot.getX() << "/" << _robot.getY() << std::endl;
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