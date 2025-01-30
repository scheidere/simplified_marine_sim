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

SendMessage::SendMessage(const std::string& name, const NodeConfig& config, World& world, Robot& sender)
    : SyncActionNode(name, config), _world(world), _sender(sender) {}

NodeStatus SendMessage::tick()
{
    try {
        std::cout << "SendMessage: Broadcasting message" << std::endl;
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

Regroup::Regroup(const std::string& name, const NodeConfig& config, Robot& receiver)
    : ConditionNode(name, config), _receiver(receiver) {}

NodeStatus Regroup::tick()
{
    try {
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
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Regroup::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList Regroup::providedPorts()
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

BuildBundle::BuildBundle(const std::string& name, const NodeConfig& config, World& w, Robot& r, CBBA& cbba)
    : ThreadedAction(name, config), _world(w), _robot(r), _cbba(cbba) {}

NodeStatus BuildBundle::tick()
{
    try {
        std::cout << "Building bundle for robot " << _robot.getID() << "..." << std::endl;
        return NodeStatus::RUNNING;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in BuildBundle::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList BuildBundle::providedPorts()
{
    return { };
}

/*Test::Test(const std::string& name, const NodeConfig& config, Robot& robot)
    : SyncActionNode(name, config), _robot(robot) {}

NodeStatus Test::tick()
{
    try {
        std::cout << "Running Test action to keep ticking..." << std::endl;
        return NodeStatus::RUNNING;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Test::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList Test::providedPorts()
{
    return {};
}*/


/*
// Action_A has a different constructor than the default one.
class Action_A: public SyncActionNode
{

public:
    // additional arguments passed to the constructor
    Action_A(const std::string& name, const NodeConfig& config,
             int arg_int, std::string arg_str):
        SyncActionNode(name, config),
        _arg1(arg_int),
        _arg2(arg_str) {}

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    // tick() can access the private members
    NodeStatus tick() override;

private:
    int _arg1;
    std::string _arg2;
};
*/

/*BuildBundle::BuildBundle(const std::string& name, const NodeConfig& config, World& w, Robot& r, CBBA& cbba)
    : ThreadedAction(name, config), _world(w), _robot(r), _cbba(cbba) {}

NodeStatus BuildBundle::tick()
{
    try {
        std::cout << "Building bundle for robot " << _robot.getID() << "..." << std::endl;
        _cbba.buildBundle(_world, _robot);
        Bundle b = _robot.getBundle();
        if (b.tasks.size() > 0) {
            std::cout << "IT WORKED A LITTLE AT LEAST: Bundle built for robot " << _robot.getID() << std::endl;
            return NodeStatus::SUCCESS;
        }
        std::cout << "No tasks added to bundle" << std::endl;
        return NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in BuildBundle::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList BuildBundle::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}*/

/*BuildBundle::BuildBundle(const std::string& name, const BT::NodeConfig& config)
    : ThreadedAction(name, config), _world(nullptr), _robot(nullptr), _cbba(nullptr) {}


void BuildBundle::setParams(World& world, Robot& robot, CBBA& cbba)
{
    _world = &world;
    _robot = &robot;
    _cbba = &cbba;
}

NodeStatus BuildBundle::tick()
{
    try {
        if (!_world || !_robot || !_cbba) {
            throw std::runtime_error("BuildBundle: World, Robot, or CBBA not initialized.");
        }
        
        std::cout << "Building bundle for robot " << _robot->getID() << "..." << std::endl;
        _cbba->buildBundle(*_world, *_robot);
        Bundle b = _robot->getBundle();
        if (b.tasks.size() > 0) {
            std::cout << "IT WORKED A LITTLE AT LEAST: Bundle built for robot " << _robot->getID() << std::endl;
            return NodeStatus::SUCCESS;
        }
        std::cout << "No tasks added to bundle" << std::endl;
        return NodeStatus::FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in BuildBundle::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList BuildBundle::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}*/