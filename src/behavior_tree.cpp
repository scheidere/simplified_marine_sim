#include "behavior_tree.hpp"
#include <iostream> // For std::cout
#include <string>   // For std::string
#include <opencv2/opencv.hpp>
#include "world.hpp"
#include "robot.hpp"
#include "CBBA.hpp"
#include "planners.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "parser.hpp"

using namespace BT;

PlanShortestPath::PlanShortestPath(const std::string& name, const NodeConfig& config, World& w, Robot& r, ShortestPath& sp)
    : SyncActionNode(name, config), _world(w), _robot(r), _shortest_path(sp) {}

NodeStatus PlanShortestPath::tick()
{
    try {
        Pose2D current_pose = _robot.getPose();
        Pose2D goal_pose;

        if (!getInput<Pose2D>("goal", goal_pose)) {
            goal_pose = _robot.getGoalPose();
            std::cout << goal_pose.x << " " << goal_pose.y << std::endl;
        }

        std::shared_ptr<ProtectedQueue<Pose2D>> plan = _shortest_path.plan(current_pose, goal_pose, _world.getX(), _world.getY());
        setOutput("path", plan);

        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in PlanShortestPath::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList PlanShortestPath::providedPorts()
{
    std::cout << "Shortest path is outputted..." << std::endl;
    return { 
        InputPort<Pose2D>("goal"),
        OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("path")
    };
}

PlanCoveragePath::PlanCoveragePath(const std::string& name, const NodeConfig& config, World& w, Robot& r, CoveragePath& cp)
    : SyncActionNode(name, config), _world(w), _robot(r), _coverage_path(cp) {}

NodeStatus PlanCoveragePath::tick()
{
    try {
        Pose2D current_pose = _robot.getPose();
        Pose2D goal_pose = {0,0,0};
        Pose2D corner1 = {0,0,0}; 
        Pose2D corner2 = {15,15,0}; 
        Pose2D corner3 = {0,0,0}; 
        Pose2D corner4 = {0,0,0};

        std::shared_ptr<ProtectedQueue<Pose2D>> plan = _coverage_path.plan(current_pose, corner1, corner2, corner3, corner4, _world.getX(), _world.getY());
        setOutput("path", plan);

        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in PlanCoveragePath::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList PlanCoveragePath::providedPorts()
{
    std::cout << "Coverage path is outputted..." << std::endl;
    return { 
        OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("path")
    };
}

PlanRegroupPath::PlanRegroupPath(const std::string& name, const NodeConfig& config, World& w, Robot& r, ShortestPath& sp)
    : SyncActionNode(name, config), _world(w), _robot(r), _shortest_path(sp) {}

NodeStatus PlanRegroupPath::tick()
{
    try {
        Pose2D current_pose = _robot.getPose();
        Pose2D goal_pose = {110,110,0}; // {200,200,0} make this a part of world class if to be used longterm

        std::shared_ptr<ProtectedQueue<Pose2D>> plan = _shortest_path.plan(current_pose, goal_pose, _world.getX(), _world.getY());
        setOutput("path", plan);

        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in PlanRegroupPath::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList PlanRegroupPath::providedPorts()
{
    std::cout << "Shortest path to regroup location is outputted..." << std::endl;
    return { 
        OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("path")
    };
}

UseWaypoint::UseWaypoint(const std::string& name, const NodeConfig& config, World& w, Robot& r)
    : ThreadedAction(name, config), _world(w), _robot(r) {}

NodeStatus UseWaypoint::tick()
{
    try {
        Pose2D wp;
        if (getInput("waypoint", wp)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "Using waypoint: " << wp.x << "/" << wp.y << std::endl;
            std::cout << "Robot prev loc: " << _robot.getX() << "/" << _robot.getY() << std::endl;
            _robot.move(wp); // Publish new waypoint to image, i.e. move robot
            std::cout << "Robot new loc: " << _robot.getX() << "/" << _robot.getY() << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << "no input in use_wp" << std::endl;
            return NodeStatus::FAILURE;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in UseWaypoint::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList UseWaypoint::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

/*SendMessage::SendMessage(const std::string& name, const NodeConfig& config, World& world, Robot& sender)
    : SyncActionNode(name, config), _world(world), _sender(sender) {}

NodeStatus SendMessage::tick()
{
    try {
        std::cout << "SendMessage: Broadcasting message" << std::endl;
        std::string log_msg = "Robot " + std::to_string(_sender.getID()) + " broadcasting message...";
        _sender.log_info(log_msg);
        Message msg(_sender);
        msg.broadcastMessage(_world);
        std::cout << "SendMessage: Completed" << std::endl;
        // The following test works
        //std::cout << "Testing sent message by printing world message tracker" << std::endl;
        //_world.printMessageTracker();
        //std::cout << "SendMessage Test Completed" << std::endl;
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in SendMessage::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList SendMessage::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

ReceiveMessage::ReceiveMessage(const std::string& name, const NodeConfig& config, World& world, Robot& receiver)
    : SyncActionNode(name, config), _world(world), _receiver(receiver) {}

NodeStatus ReceiveMessage::tick()
{
    try {
        //std::cout << "ReceiveMessage: Receiving message" << std::endl;
        _receiver.receiveMessages(); // does correct instance of world get passed to robot class? like only one instance of world should be used
        //std::cout << "ReceiveMessage: Completed" << std::endl;
        std::string log_msg = "Robot " + std::to_string(_receiver.getID()) + " receiving message(s)...";
        _receiver.log_info(log_msg);
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ReceiveMessage::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList ReceiveMessage::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}*/

SendMessage::SendMessage(const std::string& name, const NodeConfig& config, World& world, Robot& sender)
    : ThreadedAction(name, config), _world(world), _sender(sender) {}

NodeStatus SendMessage::tick()
{
    try {
        std::cout << "SendMessage: Broadcasting message" << std::endl;
        std::string log_msg = "Robot " + std::to_string(_sender.getID()) + " broadcasting message...";
        _sender.log_info(log_msg);
        Message msg(_sender);
        msg.broadcastMessage(_world);
        std::cout << "SendMessage: Completed" << std::endl;
        // The following test works
        //std::cout << "Testing sent message by printing world message tracker" << std::endl;
        //_world.printMessageTracker();
        //std::cout << "SendMessage Test Completed" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay 
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in SendMessage::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList SendMessage::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

Ping::Ping(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : ThreadedAction(name, config), _world(world), _robot(robot) {}

NodeStatus Ping::tick()
{
    try {
        std::cout << "Ping: Pinging (both sending 1 ping and listening to pings from others)" << std::endl;
        //std::string log_msg = "Robot " + std::to_string(_sender.getID()) + " broadcasting minimal message (ping)...";
        //_sender.log_info(log_msg);
        Message msg(_robot);
        msg.ping(_world);
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allowing for time for all robots to ping before listening (can prolly remove once ongoing pinging happening)
        _robot.receivePings();
        std::string log_msg = "Robot " + std::to_string(_robot.getID()) + " receiving pings(s)...";
        std::cout << log_msg << std::endl;
        std::cout << "Ping: Completed" << std::endl;
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Ping::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList Ping::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

NewInfoAvailable::NewInfoAvailable(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : ConditionNode(name, config), _world(world), _robot(robot) {}

NodeStatus NewInfoAvailable::tick()
{
    try {
        std::cout << "Checking if new info is available (any pings heard)..." << std::endl;

        bool info_available = _robot.checkIfNewInfoAvailable();

        if (info_available) {
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in NewInfoAvailable::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList NewInfoAvailable::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

ReceiveMessage::ReceiveMessage(const std::string& name, const NodeConfig& config, World& world, Robot& receiver)
    : ThreadedAction(name, config), _world(world), _receiver(receiver) {}

NodeStatus ReceiveMessage::tick()
{
    try {
        //std::cout << "ReceiveMessage: Receiving message" << std::endl;
        _receiver.receiveMessages(); // does correct instance of world get passed to robot class? like only one instance of world should be used
        //std::cout << "ReceiveMessage: Completed" << std::endl;
        std::string log_msg = "Robot " + std::to_string(_receiver.getID()) + " receiving message(s)...";
        _receiver.log_info(log_msg);
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ReceiveMessage::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList ReceiveMessage::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

Communicate::Communicate(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : ThreadedAction(name, config), _world(world), _robot(robot) {}

NodeStatus Communicate::tick()
{
    try {

        // Send messages
        std::string log_msg = "Robot " + std::to_string(_robot.getID()) + " broadcasting message...";
        _robot.log_info(log_msg);
        Message msg(_robot);
        msg.broadcastMessage(_world);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay 

        // Receive messages
        _robot.receiveMessages(); // does correct instance of world get passed to robot class? like only one instance of world should be used
        std::string log_msg2 = "Robot " + std::to_string(_robot.getID()) + " receiving message(s)...";
        _robot.log_info(log_msg2);

        _robot.updateTimestamps();
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Communicate::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList Communicate::providedPorts()
{
    return {};
}

/*ReceivePing::ReceivePing(const std::string& name, const NodeConfig& config, World& world, Robot& receiver)
    : ThreadedAction(name, config), _world(world), _receiver(receiver) {}

NodeStatus ReceivePing::tick()
{
    try {
        //std::cout << "ReceiveMessage: Receiving message" << std::endl;
        _receiver.receivePings(); // does correct instance of world get passed to robot class? like only one instance of world should be used
        //std::cout << "ReceiveMessage: Completed" << std::endl;
        std::string log_msg = "Robot " + std::to_string(_receiver.getID()) + " receiving pings(s)...";
        std::cout << log_msg << std::endl;
        //_receiver.log_info(log_msg);
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ReceiveMessage::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList ReceivePing::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}*/

TestMessages::TestMessages(const std::string& name, const NodeConfig& config, World& world, Robot& receiver)
    : SyncActionNode(name, config), _world(world), _receiver(receiver) {}

NodeStatus TestMessages::tick()
{
    try {
        //std::cout << "ReceiveMessage: Receiving message" << std::endl;
        _receiver.getMessageQueue(); // does correct instance of world get passed to robot class? like only one instance of world should be used
        //std::cout << "ReceiveMessage: Completed" << std::endl;
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ReceiveMessage::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList TestMessages::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

NeedRegroup::NeedRegroup(const std::string& name, const NodeConfig& config, Robot& receiver)
    : ConditionNode(name, config), _receiver(receiver) {}

NodeStatus NeedRegroup::tick()
{
    try {
        Pose2D rendezvous{0,0,0};
        setOutput("rendezvous", rendezvous);
        std::cout << "Regroup: Checking if regroup triggered" << std::endl;
        std::cout << "regroup result: " << _receiver.needRegroup() << std::endl;
        if (_receiver.needRegroup()) {
            std::cout << "Regroup triggered" << std::endl;
            std::string log_msg = "Robot " + std::to_string(_receiver.getID()) + ": regroup condition True";
            _receiver.log_info(log_msg);
            return NodeStatus::SUCCESS;
        }
        std::cout << "Regroup NOT triggered" << std::endl;
        std::string log_msg = "Robot " + std::to_string(_receiver.getID()) + ": regroup condition False";
        _receiver.log_info(log_msg);
        return NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Regroup::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList NeedRegroup::providedPorts()
{
    return { OutputPort<Pose2D>("rendezvous") };
}

TestCond::TestCond(const std::string& name, const NodeConfig& config, Robot& receiver)
    : ConditionNode(name, config), _receiver(receiver) {}

NodeStatus TestCond::tick()
{
    try {
        std::cout << "in TestCond" << std::endl;
        return NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in TestCond::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList TestCond::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

DummySuccessAction::DummySuccessAction(const std::string& name, const NodeConfig& config)
    : ThreadedAction(name, config) {}

NodeStatus DummySuccessAction::tick()
{
    try {
        std::cout << "Running DummySuccessAction action to get success..." << std::endl;
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in DummySuccessAction::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList DummySuccessAction::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

RunTest::RunTest(const std::string& name, const NodeConfig& config)
    : ThreadedAction(name, config) {}

NodeStatus RunTest::tick()
{
    try {
        std::cout << "Running RunTest action to keep ticking..." << std::endl;
        return NodeStatus::RUNNING;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in RunTest::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList RunTest::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

RunTest2::RunTest2(const std::string& name, const NodeConfig& config)
    : ThreadedAction(name, config) {}

NodeStatus RunTest2::tick()
{
    try {
        std::cout << "Running RunTest action to keep ticking..." << std::endl;
        return NodeStatus::RUNNING;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in RunTest::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList RunTest2::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}

BuildBundle::BuildBundle(const std::string& name, const NodeConfig& config, Robot& r, JSONParser& p)
    : ThreadedAction(name, config), _robot(r), _parser(p) {}

NodeStatus BuildBundle::tick()
{
    try {
        std::cout << "Building bundle for robot " << _robot.getID() << "..." << std::endl;
        CBBA cbba(_robot, _parser);
        cbba.buildBundle();
        //return NodeStatus::RUNNING; // was there a purpose for this other than testing?
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in BuildBundle::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList BuildBundle::providedPorts()
{
    return { };
}

ResolveConflicts::ResolveConflicts(const std::string& name, const NodeConfig& config, Robot& r, JSONParser& p)
    : ThreadedAction(name, config), _robot(r), _parser(p) {}

NodeStatus ResolveConflicts::tick()
{
    try {
        std::cout << "Resolving conflicts for robot " << _robot.getID() << "..." << std::endl;
        CBBA cbba(_robot, _parser);
        //cbba.resolveConflicts();
        cbba.resolveConflicts(true); // for testing
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ResolveConflicts::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList ResolveConflicts::providedPorts()
{
    return { };
}
