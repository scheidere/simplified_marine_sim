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
#include "utils.hpp"


using namespace BT;

// Note sync action nodes can't return running so need to update all of the following to be stateful action nodes instead 
// (threaded actions are not compatible with our threading)

// Will likely need to update to StatefulActionNode
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

// Will likely need to update to StatefulActionNode (see cbba nodes defined elsewhere in this file for how)
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

// Will likely need to update to StatefulActionNode
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

// Will likely need to update to StatefulActionNode (if this is used alone - probably not)
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

/*SendMessage::SendMessage(const std::string& name, const NodeConfig& config, World& world, Robot& sender)
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
        _sender.log_info("SendMessage node FAILURE");
        return NodeStatus::FAILURE;
    }
}

PortsList SendMessage::providedPorts()
{
    return { InputPort<Pose2D>("waypoint") };
}*/

Ping::Ping(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : StatefulActionNode(name, config), _world(world), _robot(robot) {}

NodeStatus Ping::onStart() {

    return NodeStatus::RUNNING;
}

NodeStatus Ping::onRunning() {

    try {
        // Commented out all prints/logs because pinging happens continuously now

        //std::cout << "Ping: Pinging (both sending 1 ping and listening to pings from others)" << std::endl;
        //std::string log_msg = "Robot " + std::to_string(_sender.getID()) + " broadcasting minimal message (ping)...";
        //_sender.log_info(log_msg);
        Message msg(_robot);
        msg.ping(_world);
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allowing for time for all robots to ping before listening (can prolly remove once ongoing pinging happening)
        //_robot.receivePings();
        //std::string log_msg = "Robot " + std::to_string(_robot.getID()) + " receiving pings(s)...";
        //std::cout << log_msg << std::endl;
        //std::cout << "Ping: Completed" << std::endl;
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Ping::tick: " << e.what() << std::endl;
        _robot.log_info("Ping node FAILURE");
        return NodeStatus::FAILURE;
    }
}

void Ping::onHalted() {}

/*NodeStatus Ping::tick()
{
    try {
        // Commented out all prints/logs because pinging happens continuously now

        //std::cout << "Ping: Pinging (both sending 1 ping and listening to pings from others)" << std::endl;
        //std::string log_msg = "Robot " + std::to_string(_sender.getID()) + " broadcasting minimal message (ping)...";
        //_sender.log_info(log_msg);
        Message msg(_robot);
        msg.ping(_world);
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allowing for time for all robots to ping before listening (can prolly remove once ongoing pinging happening)
        //_robot.receivePings();
        //std::string log_msg = "Robot " + std::to_string(_robot.getID()) + " receiving pings(s)...";
        //std::cout << log_msg << std::endl;
        //std::cout << "Ping: Completed" << std::endl;
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Ping::tick: " << e.what() << std::endl;
        _robot.log_info("Ping node FAILURE");
        return NodeStatus::FAILURE;
    }
}*/

PortsList Ping::providedPorts()
{
    return {};
}


NewInfoAvailable::NewInfoAvailable(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : ConditionNode(name, config), _world(world), _robot(robot) {}       

NodeStatus NewInfoAvailable::tick()
{
    try {
        //std::cout << "Checking if new info is available (any pings heard)..." << std::endl;

        bool info_available = _robot.checkIfNewInfoAvailable();

        std::cout << "[Robot " << _robot.getID() << "] info_available is " << (info_available ? "TRUE" : "FALSE") << std::endl;

        if (info_available) {
            std::string bla = "info_available is TRUE";
            //std::cout << bla << std::endl;
            _robot.resetNumCBBARounds(); // Clear rounds counted from last time CBBA ran
            _robot.log_info(bla);
            return NodeStatus::SUCCESS;
        } else {
            std::string bla1 = "info_available is FALSE";
            //std::cout << bla1 << std::endl;
            _robot.log_info(bla1);
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in NewInfoAvailable::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList NewInfoAvailable::providedPorts()
{
    return {};
}

Communicate::Communicate(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : StatefulActionNode(name, config), _world(world), _robot(robot) {}

NodeStatus Communicate::onStart() {

    return NodeStatus::RUNNING;
}

NodeStatus Communicate::onRunning()
{
    try {

        // Checking if BT operating on it's own thread - PASSED (Now that this is stateful action node it does! Unique thread id for each robot)
        /*std::thread::id thread_id = std::this_thread::get_id();
        std::hash<std::thread::id> hasher;
        size_t thread_hash = hasher(thread_id);
        std::string bla = "Robot " + std::to_string(_robot.getID()) + "'s Communicate - Thread ID: " + std::to_string(thread_hash);
        _robot.log_info(bla);*/

        // Send messages
        std::string log_msg = "Robot " + std::to_string(_robot.getID()) + " broadcasting message...";
        _robot.log_info(log_msg);

        _robot.log_info("Timestamps BEFORE change in Communicate::tick in behavior_tree.cpp:");
        utils::logUnorderedMap(_robot.getTimestamps(),_robot);

        Message msg(_robot);
        //_robot.log_info("Timestamps test: ");
        //utils::logUnorderedMap(_robot.getTimestamps(), _robot);
        msg.broadcastMessage(_world);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay 

        //_robot.log_info("in communicate node, before receiveMessages");

        // Receive messages
        _robot.receiveMessages(); // does correct instance of world get passed to robot class? like only one instance of world should be used
        std::string log_msg2 = "Robot " + std::to_string(_robot.getID()) + " receiving message(s)...";
        _robot.log_info(log_msg2);

        for (const auto& msg : _robot.getMessageQueue()) {
            std::string bla = "ID: " + std::to_string(msg.id);
            _robot.log_info(bla);
            _robot.log_info("Winners: ");
            utils::logUnorderedMap(msg.winners,_robot);
            _robot.log_info("Winning bids: ");
            utils::logUnorderedMap(msg.winning_bids,_robot);
            _robot.log_info("Timestamps: ");
            utils::logUnorderedMap(msg.timestamps,_robot);
        }
            

        _robot.updateTimestamps();


        _robot.log_info("Timestamps AFTER change in Communicate::tick in behavior_tree.cpp:");
        utils::logUnorderedMap(_robot.getTimestamps(),_robot);
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Communicate::tick: " << e.what() << std::endl;
        _robot.log_info("Communicate node FAILURE");
        return NodeStatus::FAILURE;
    }
}

void Communicate::onHalted() {}

PortsList Communicate::providedPorts()
{
    return {};
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

BuildBundle::BuildBundle(const std::string& name, const NodeConfig& config, Robot& r, World& w, JSONParser& p)
    : StatefulActionNode(name, config), _robot(r), _world(w), _parser(p) {}

NodeStatus BuildBundle::onStart()
{

    return BT::NodeStatus::RUNNING;
}

NodeStatus BuildBundle::onRunning()
{
    try {

        // Checking if BT operating on it's own thread - PASSED (Now that this is stateful action node it does! Unique thread id for each robot)
       /* std::thread::id thread_id = std::this_thread::get_id();
        std::hash<std::thread::id> hasher;
        size_t thread_hash = hasher(thread_id);
        std::string bla = "Robot " + std::to_string(_robot.getID()) + "'s BuildBundle - Thread ID: " + std::to_string(thread_hash);
        _robot.log_info(bla);*/

        std::cout << "Building bundle for robot " << _robot.getID() << "..." << std::endl;
        CBBA cbba(_robot, _world, _parser);
        cbba.buildBundle();
        //return NodeStatus::RUNNING; // was there a purpose for this other than testing?
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in BuildBundle::tick: " << e.what() << std::endl;
        _robot.log_info("BuildBundle node FAILURE");
        return NodeStatus::FAILURE;
    }

}

void BuildBundle::onHalted()
{
    
}

PortsList BuildBundle::providedPorts()
{
    return { };
}

ResolveConflicts::ResolveConflicts(const std::string& name, const NodeConfig& config, Robot& r, World& w, JSONParser& p)
    : StatefulActionNode(name, config), _robot(r), _world(w), _parser(p) {}


NodeStatus ResolveConflicts::onStart()
{
    return NodeStatus::RUNNING;
}

NodeStatus ResolveConflicts::onRunning()
{
    try {

        // Checking if BT operating on it's own thread - PASSED (Now that this is stateful action node it does! Unique thread id for each robot)
        /*std::thread::id thread_id = std::this_thread::get_id();
        std::hash<std::thread::id> hasher;
        size_t thread_hash = hasher(thread_id);
        std::string bla = "Robot " + std::to_string(_robot.getID()) + "'s ResolveConflicts - Thread ID: " + std::to_string(thread_hash);
        _robot.log_info(bla);*/

        std::cout << "Resolving conflicts for robot " << _robot.getID() << "..." << std::endl;
        CBBA cbba(_robot, _world, _parser);
        cbba.resolveConflicts();
        //cbba.resolveConflicts(true); // for testing
        int& rounds = _robot.getNumCBBARounds();
        rounds++;
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ResolveConflicts::tick: " << e.what() << std::endl;
        _robot.log_info("ResolveConflicts node FAILURE");
        return NodeStatus::FAILURE;
    }
}

void ResolveConflicts::onHalted() 
{

}

PortsList ResolveConflicts::providedPorts()
{
    return { };
}

CheckConvergence::CheckConvergence(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : ConditionNode(name, config), _world(world), _robot(robot) {}

NodeStatus CheckConvergence::tick()
{
    try {

        // Check if RepeatSequence signaled a reset by setting blackboard to 0
        auto blackboard_value = getInput<int>("cumulative_convergence_count_in");
        if (blackboard_value.has_value()) {
            std::cout << "HI HI HI HI HI cumulative_convergence_count in checkConvergence: " << blackboard_value.value() << std::endl;
            if (blackboard_value.value() == 0) {
                // Reset local (to robot) convergence count 
                _robot.resetConvergenceCount();
                std::cout << "Detected blackboard reset - resetting robot's internal count" << std::endl;
            }
        } else {
            std::cout << "cumulative_convergence_count_in has no value!" << std::endl;
        }

        std::cout << "Checking whether the CBBA has resulted in local convergence and how this has been maintained by number of iterations..." << std::endl;

        int& rounds = _robot.getNumCBBARounds();
        std::string bloo = "CBBA Round " + std::to_string(rounds) + " complete";
        _robot.log_info(bloo);

        _robot.countConvergedIterations(); // Compare robot beliefs to beliefs at previous iteration (stored in)

        int cumulative_convergence_count = _robot.getConvergenceCount(); // Number of iterations convergence has remained

        _robot.updateBeliefs();

        _robot.log_info("After convergence check at end of CBBA round, bundle and path are the following:");
        std::vector<int> bundle = _robot.getBundle();
        _robot.log_info("Bundle:");
        utils::log1DVector(bundle, _robot);
        std::vector<int> path = _robot.getPath();
        _robot.log_info("Path:");
        utils::log1DVector(path, _robot);

        setOutput("cumulative_convergence_count_out", cumulative_convergence_count); // original

        /*auto threshold_met = getInput<bool>("threshold_met");
        if (threshold_met && threshold_met.value()) { // If given a threshold_met boolean variable as input AND it is true
            cumulative_convergence_count = 0;  // Reset local counter
            setOutput("cumulative_convergence_count", 0);  // Output reset value
            std::cout << "hellohellohelloooooooo RESET convergence count due to threshold met" << std::endl;
        } else { // Otherwise either not using threshold in unlimited repeat case OR using threshold and it's not yet reached
            // Normal operation - output current count
            setOutput("cumulative_convergence_count", cumulative_convergence_count);
        }*/

        /*auto threshold_met = getInput<bool>("threshold_met");
        std::cout << "Successfully got threshold_met input" << std::endl;
        std::cout << "threshold_met.has_value(): " << threshold_met.has_value() << std::endl;
        
        if (threshold_met.has_value()) {
            std::cout << "threshold_met.value(): " << threshold_met.value() << std::endl;
            if (threshold_met.value()) { 
                std::cout << "Threshold was met, resetting count" << std::endl;
                cumulative_convergence_count = 0;  
                setOutput("cumulative_convergence_count", 0);  
            } else {
                std::cout << "Threshold not met, outputting normal count" << std::endl;
                setOutput("cumulative_convergence_count", cumulative_convergence_count);
            }
        } else {
            std::cout << "threshold_met has no value, outputting normal count" << std::endl;
            setOutput("cumulative_convergence_count", cumulative_convergence_count);
        }*/

        // For testing, given we currently only have 2 robots, let's pseudo add 3 as if a third robot came into comms range for once of the robots (id 1)
        // This should cause cbba to run twice for robot 1, and only once for robot 2
        // Prevent duplicates of this id 3, since this node will be executed once per cbba round and there will be multiple rounds to reach convergence
        std::unordered_map<int, std::vector<int>>& world_ping_tracker = _world.getPingTracker();
        std::vector<int>& new_pings = world_ping_tracker[1];
        if (std::find(new_pings.begin(), new_pings.end(), 3) == new_pings.end()) {
            new_pings.push_back(3);  // Mimicking robot 1 hearing a ping from robot 3 that is newly in range
            std::cout << "ADDING 3 TO ROBOT 1 PING TRACKER VECTOR" << std::endl;
            _robot.printWorldPingTracker(world_ping_tracker);
        }
        
        return NodeStatus::SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in CheckConvergence::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList CheckConvergence::providedPorts()
{
    return { InputPort<int>("cumulative_convergence_count_in"),
            OutputPort<int>("cumulative_convergence_count_out") }; //,
             //InputPort<bool>("threshold_met", "Check if threshold was reached")};
}