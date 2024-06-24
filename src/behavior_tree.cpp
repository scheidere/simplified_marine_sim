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


PlanShortestPath::PlanShortestPath(const std::string& name, const NodeConfig& config, World& w, Robot& r, ShortestPath& sp)
    : SyncActionNode(name, config), _world(w), _robot(r), _shortest_path(sp)
  {}

NodeStatus PlanShortestPath::tick()
{
    // How long is a tick? What happens if the planning takes longer than a tick *should*?
    Pose2D current_pose = _robot.getPose();
    Pose2D goal_pose;
    if (!getInput<Pose2D>("goal", goal_pose)) {
        //throw BT::RuntimeError("Missing required input [goal]");
        Pose2D goal_pose = _robot.getGoalPose();
        std::cout << goal_pose.x << " " << goal_pose.y << std::endl;
        //start here - gotta figure out why getting PQ alert error
        //throw BT::RuntimeError("Missing required input [goal]");
        std::shared_ptr<ProtectedQueue<Pose2D>> plan = _shortest_path.plan(current_pose, goal_pose,_world.getX(), _world.getY());
        setOutput("path", plan);
    } else {
    std::shared_ptr<ProtectedQueue<Pose2D>> plan = _shortest_path.plan(current_pose, goal_pose,_world.getX(), _world.getY());
    setOutput("path", plan); }

    return NodeStatus::SUCCESS;
}

PortsList PlanShortestPath::providedPorts()
{
    std::cout << "Shortest path is outputted..." << std::endl;
    return { 
        InputPort<Pose2D>("goal"),
        OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("path") };
}

// Note that threaded actions need extra logic to ensure they halt: https://www.behaviortree.dev/docs/3.8/tutorial-advanced/asynchronous_nodes/
// We have not yet checked that or added it to this threaded action
UseWaypoint::UseWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r)
    : ThreadedAction(name, config), _world(w), _robot(r)
  {}

NodeStatus UseWaypoint::tick()
{

    Pose2D wp;
    if(getInput("waypoint", wp))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::cout << "Using waypoint: " << wp.x << "/" << wp.y << std::endl;
      std::cout << "Robot prev loc: " << _robot.getX() << "/" << _robot.getY() << std::endl;
      _robot.move(wp); // Publish new waypoint to image, i.e. move robot
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
    return { InputPort<Pose2D>("waypoint") };
    //return {};
}

SendMessage::SendMessage(const std::string& name, const NodeConfig& config, World& world, Robot& sender)
    : SyncActionNode(name, config), _world(world), _sender(sender) {}

NodeStatus SendMessage::tick()
{
    std::cout << "SendMessage: Broadcasting message" << std::endl;
    Message msg(_sender);
    msg.broadcastMessage(_world);
    std::cout << "SendMessage: Completed" << std::endl;
    return NodeStatus::SUCCESS;
}

PortsList SendMessage::providedPorts()
{
    // Added this because it was complaining when there were no ports
    // This input is not actually needed by this node 
    // So this is just for testing
    return { InputPort<Pose2D>("waypoint") };
    //return {};
}

ReceiveMessage::ReceiveMessage(const std::string& name, const NodeConfig& config, World& world, Robot& receiver)
    : SyncActionNode(name, config), _world(world), _receiver(receiver) {}

NodeStatus ReceiveMessage::tick()
{
    std::cout << "ReceiveMessage: Receiving message" << std::endl;
    _receiver.receiveMessages(); // does correct instance of world get passed to robot class? like only one instance of world should be used
    std::cout << "ReceiveMessage: Completed" << std::endl;
    return NodeStatus::SUCCESS;
}

PortsList ReceiveMessage::providedPorts()
{
    // Added this because it was complaining when there were no ports
    // This input is not actually needed by this node 
    // So this is just for testing
    return { InputPort<Pose2D>("waypoint") };
    //return {};
}

/*start here, make this a condition and init the condition in sim with the rest of the nodes 
then add another sequence to the tree and pass the goal waypoint from this node to _shortest_path
probably just create another action node that has the center waypoint and calls shortestpath and returns the path 
then the same node logic as elsewhere in the tree can be used for the robot(s) to traverse to converge to same spot
*/
/*Regroup::Regroup(const std::string& name, const NodeConfig& config, World& world, Robot& receiver)
    : SyncActionNode(name, config), _world(world), _receiver(receiver) {} // Make this a condition

NodeStatus Regroup::tick()
{
    if (_receiver.regroup();) {
        return return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

PortsList Regroup::providedPorts()
{
    // Added this because it was complaining when there were no ports
    // This input is not actually needed by this node 
    // So this is just for testing
    return { InputPort<Pose2D>("waypoint") };
    //return {};
}*/

Regroup::Regroup(const std::string& name, const NodeConfig& config, Robot& receiver)
    : ConditionNode(name, config), _receiver(receiver) {} // Make this a condition

NodeStatus Regroup::tick()
{
    Pose2D rendezvous{0,0,0};
    setOutput("rendezvous", rendezvous);
    std::cout << "Regroup: Checking if regroup triggered" << std::endl;
    std::cout << "regroup result: " << _receiver.regroup() << std::endl;
    if (_receiver.regroup()) {
        std::cout << "Regroup triggered" << std::endl;
        return NodeStatus::SUCCESS;
    }
    std::cout << "Regroup NOT triggered" << std::endl;
    return NodeStatus::FAILURE;
}

PortsList Regroup::providedPorts()
  {
    // Outputs goal to regroup with other robots (then shortestpath planner can take in waypoint)
    return { OutputPort<Pose2D>("rendezvous") };
  }

// Method below is for SimpleConditionNode
/*NodeStatus Regroup()
{
  if (_receiver.regroup();) {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}*/