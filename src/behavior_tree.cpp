#include "behavior_tree.hpp"
#include <iostream> // For std::cout
#include <string>   // For std::string
#include <opencv2/opencv.hpp>
#include "world.hpp"
#include "robot.hpp"
#include "CBBA.hpp"
#include "CBGA.hpp"
#include "planners.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "parser.hpp"
#include "utils.hpp"
#include "greedy.hpp"


using namespace BT;


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

        //std::cout << "[Robot " << _robot.getID() << "] info_available is " << (info_available ? "TRUE" : "FALSE") << std::endl;

        auto do_cbga = getInput<bool>("do_cbga").value_or(false);

        if (info_available) {
            std::string bla = "info_available is TRUE";
            //std::cout << bla << std::endl;
            if (do_cbga) {
                 _robot.resetNumCBGARounds(); // Clear rounds counted from last time CBGA ran
            } else {
                _robot.resetNumCBBARounds(); // Clear rounds counted from last time CBBA ran
            }
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
    return {InputPort<bool>("do_cbga")};
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

        // for testing for broadcast issue
        auto elapsed = _world.getElapsedTime();
        std::string timing_log = "Robot " + std::to_string(_robot.getID()) + " Communicate at time: " + std::to_string(elapsed) + "ms";
        _world.log_info(timing_log);

        std::string blork1 = "Robot " + std::to_string(_robot.getID()) + " STARTING broadcast";
        _world.log_info(blork1);  // Use world.log_info

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

        std::string blork2 = "Robot " + std::to_string(_robot.getID()) + " FINISHED broadcast";  
        _world.log_info(blork2);

        std::string b = "Robot " + std::to_string(_robot.getID()) + "finished broadcasting.";
        _robot.log_info(b);

        //_robot.log_info("in communicate node, before receiveMessages");

        /// testing... ///
        std::unordered_map<int, std::vector<Msg>>& world_msg_tracker = _world.getMessageTracker();
        std::string debug_msg = "Robot " + std::to_string(_robot.getID()) + " accessing tracker at address: " + std::to_string((uintptr_t)&world_msg_tracker);
        _world.log_info(debug_msg);

        _robot.log_info("World message tracker by robot:");
        for (const auto& pair : world_msg_tracker) {
            int robot_id = pair.first;
            const std::vector<Msg>& messages = pair.second;
            
            std::string log_msg = "Robot " + std::to_string(robot_id) + " has messages from: ";
            for (const auto& msg : messages) {
                log_msg += std::to_string(msg.id) + " ";
            }
            log_msg += "(" + std::to_string(messages.size()) + " total)";
            _robot.log_info(log_msg);
        }
        /// testing above... ///

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
        _robot.updateLocations(); // CBGA

        _robot.log_info("Timestamps AFTER change in Communicate::tick in behavior_tree.cpp:");
        utils::logUnorderedMap(_robot.getTimestamps(),_robot);
        std::string plorp = "Robot " + std::to_string(_robot.getID()) + " Communicate returning SUCCESS";
        _world.log_info(plorp);
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

        auto do_cbga = getInput<bool>("do_cbga").value_or(false);

        if (do_cbga) {
            CBGA cbga(_robot, _world, _parser);
            cbga.buildBundle();
        } else {
            CBBA cbba(_robot, _world, _parser);
            cbba.buildBundle();
        }

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
    return {InputPort<bool>("do_cbga")};
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

        auto do_cbga = getInput<bool>("do_cbga").value_or(false);

        int rounds;
        if (do_cbga) {
            CBGA cbga(_robot, _world, _parser);
            //cbga.resolveConflicts();
            _robot.log_info("right before running cbga with test = true");
            cbga.resolveConflicts(true); // for testing
            int& rounds = _robot.getNumCBGARounds();
        } else {
            CBBA cbba(_robot, _world, _parser);
            //cbba.resolveConflicts();
            cbba.resolveConflicts(true); // for testing
            int& rounds = _robot.getNumCBBARounds();
        }
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
    return {InputPort<bool>("do_cbga")};
}

CheckConvergence::CheckConvergence(const std::string& name, const NodeConfig& config, World& world, Robot& robot, JSONParser& parser)
    : ConditionNode(name, config), _world(world), _robot(robot), _parser(parser) {}

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

        auto do_cbga = getInput<bool>("do_cbga").value_or(false);
        std::string bloo;
        if (do_cbga) {
            std::cout << "Checking whether the CBGA has resulted in local convergence and how this has been maintained by number of iterations..." << std::endl;
            int& rounds = _robot.getNumCBGARounds();
            bloo = "CBGA Round " + std::to_string(rounds) + " complete";
        } else {
            std::cout << "Checking whether the CBBA has resulted in local convergence and how this has been maintained by number of iterations..." << std::endl;
            int& rounds = _robot.getNumCBBARounds();
            bloo = "CBBA Round " + std::to_string(rounds) + " complete";
        }
        _robot.log_info(bloo);

        _robot.countConvergedIterations(); // Compare robot beliefs to beliefs at previous iteration (stored in)

        int cumulative_convergence_count = _robot.getConvergenceCount(); // Number of iterations convergence has remained

        _robot.updateBeliefs();


        _robot.log_info("After convergence check at end of CBBA/CBGA round, bundle and path are the following:");
        std::vector<int> bundle = _robot.getBundle();
        _robot.log_info("Bundle:");
        utils::log1DVector(bundle, _robot);
        std::vector<int> path = _robot.getPath();
        _robot.log_info("Path:");
        utils::log1DVector(path, _robot);

        setOutput("cumulative_convergence_count_out", cumulative_convergence_count); // original

        _robot.log_info("RIGHT BEFORE SET TO 3 IF STATEMENT");
        bool test = false;
        if (test and _robot.getID() == 1) { // Only have robot 1 add it otherwise might create race condition
            _robot.log_info("JUST INSIDE SET TO 3 IF STATEMENT (should only happen for robot id 1)");
            // For testing, given we currently only have 2 robots, let's pseudo add 3 as if a third robot came into comms range for once of the robots (id 1)
            // This should cause cbba to run twice for robot 1, and only once for robot 2
            // Prevent duplicates of this id 3, since this node will be executed once per cbba round and there will be multiple rounds to reach convergence
            std::unordered_map<int, std::vector<std::pair<int,double>>>& world_ping_tracker = _world.getPingTracker();
            //std::vector<int>& new_pings = world_ping_tracker[1];
            std::vector<std::pair<int,double>>& new_pings = world_ping_tracker[1]; // Getting ping tracker for robot 1

            auto it = std::find_if(new_pings.begin(), new_pings.end(), 
                       [](const std::pair<int,double>& p) { return p.first == 3; });

            // If didn't find a ping from robot 3 already, add one (with made up timestamp of 1.0)
            if (it == new_pings.end()) {
                new_pings.push_back({3, 1.0});
                std::cout << "ADDING 3 TO ROBOT 1 PING TRACKER VECTOR" << std::endl;
                _robot.printWorldPingTracker(world_ping_tracker);
            }

        }

        // If any changes to bundle, path, winners list, or winning bids list -> update timestamp of last self-update for pinging purposes
        // Commented out because it catches changes made redundantly (e.g., between robot i and k, not just between k and j where k is neighbor of i but j isn't)
        // Causes CBBA to run redundant rounds more often than it probably would catch the external case
        /*if (_robot.foundBeliefUpdate()) {
            double timestamp_now = _robot.getCurrentTime();
            _robot.updateLastSelfUpdateTime(timestamp_now);
        }*/
        

        // if cumulative convergence count is newly at threshold (which will also be checked inside repeat sequence node)
        // then set trigger flag to true (this will only be used in the local task execution subtree to protect against concurrent path changes + execution)
        int convergence_threshold;
        if (do_cbga) {
            CBGA cbga(_robot, _world, _parser);
            convergence_threshold = cbga.getConvergenceThreshold();
        } else {
            CBBA cbba(_robot, _world, _parser);
            convergence_threshold = cbba.getConvergenceThreshold();
        }
        std::string blorb1 = "convergence_threshold: " + std::to_string(convergence_threshold);
        _robot.log_info(blorb1);
        std::string blorb2 = "cumulative_convergence_count: " + std::to_string(cumulative_convergence_count);
        _robot.log_info(blorb2);
        if (cumulative_convergence_count >= convergence_threshold) {
            _robot.setAtConsensus(true);
            _robot.log_info("Setting at_consensus to TRUE (at end of checkConvergence)");
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
            OutputPort<int>("cumulative_convergence_count_out"),
            InputPort<bool>("do_cbga") }; //,
             //InputPort<bool>("threshold_met", "Check if threshold was reached")};
}

GreedyTaskAllocator::GreedyTaskAllocator(const std::string& name, const NodeConfig& config, Robot& r, World& w)
    : StatefulActionNode(name, config), _robot(r), _world(w) {}

NodeStatus GreedyTaskAllocator::onStart()
{

    return BT::NodeStatus::RUNNING;
}

NodeStatus GreedyTaskAllocator::onRunning()
{
    try {

        std::cout << "Building greedy path for robot " << _robot.getID() << "..." << std::endl;
        Greedy greedy2(_robot, _world);
        greedy2.run();
        return NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Greedy::onRunning: " << e.what() << std::endl;
        _robot.log_info("Greedy node FAILURE");
        return NodeStatus::FAILURE;
    }

}

void GreedyTaskAllocator::onHalted()
{
    
}

PortsList GreedyTaskAllocator::providedPorts()
{
    return { };
}

// Below I am re-implementing the path planning logic as single stateful action nodes
// as opposed to sync action nodes that require QueueSize and PopFromQueue (both sync action nodes)
// as well as use waypoint defined here as a threaded action node
// It is better to just call one node in the tree

// Starting with simplest logic, shortest path
FollowShortestPath::FollowShortestPath(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w, ShortestPath& sp)
    : StatefulActionNode(name, config), _robot(r), _world(w), _shortest_path_planner(sp)
    , _current_waypoint_index(0) {}

NodeStatus FollowShortestPath::onStart()
{
    try {
        std::cout << "Planning shortest path for robot " << _robot.getID() << "..." << std::endl;
        
        Pose2D current_pose = _robot.getPose();
        Pose2D goal_pose;
        std::pair<int,int> goal_loc;
        // Check for location input, and if found, convert to pose by adding 0 for theta
        if (getInput<std::pair<int,int>>("goal_loc", goal_loc)) {
            goal_pose = {goal_loc.first, goal_loc.second,0};
        } else {
            _robot.log_info("No input given");
        }

        // for testing updateLocations()
        /*if (_robot.getID() == 1) {
            goal_pose = {10,10,0}; // same as starting location for test 1
        } else if (_robot.getID() == 2) {
            goal_pose = {20,10,0}; // same as starting location for test 1
        } else if (_robot.getID() == 3) {
            goal_pose = {65,10,0}; // starting location will {large, 10,0} so will stop in comms with k but not yet with i 
        }*/

        std::string bla = "Goal pose for shortest path is: " + std::to_string(goal_pose.x) + ", " + std::to_string(goal_pose.y);
        _robot.log_info(bla);
        
        // Init vector of waypoints, the plan
        _waypoints = _shortest_path_planner.plan(current_pose, goal_pose,
                                                _world.getX(), _world.getY());
        
        if (_waypoints.empty()) {
            std::cout << "No path found" << std::endl;
            return NodeStatus::FAILURE;
        }
        
        _current_waypoint_index = 0;
        std::cout << "Planned path with " << _waypoints.size() << " waypoints" << std::endl;
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus FollowShortestPath::onRunning()
{
    try {
        if (_current_waypoint_index >= _waypoints.size()) {
            std::cout << "All waypoints completed!" << std::endl;
            return NodeStatus::SUCCESS;
        }
        
        Pose2D waypoint = _waypoints[_current_waypoint_index];
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Using waypoint " << (_current_waypoint_index + 1) << "/" << _waypoints.size()
                  << ": " << waypoint.x << "/" << waypoint.y << std::endl;
        
        _robot.move(waypoint);
        
        _current_waypoint_index ++;
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void FollowShortestPath::onHalted()
{
    std::cout << "FollowShortestPath halted at waypoint " << _current_waypoint_index
              << "/" << _waypoints.size() << std::endl;
    _current_waypoint_index = 0;
    _waypoints.clear();
}

PortsList FollowShortestPath::providedPorts()
{
    return { InputPort<std::pair<int,int>>("goal_loc") };
}

ExploreA::ExploreA(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus ExploreA::tick()
{
    try {


        if (_robot.ExploreA()) {
            // will need to add logic to actually give output port a value
            // location of task (or start for coverage planner for example)
            std::pair<int,int> start_loc = _robot.getNextStartLocation(); // Location of first task in path (which here is ExploreA)
            std::string bla = "start_loc in ExploreA tick (x, y): " + std::to_string(start_loc.first) + ", " + std::to_string(start_loc.second);
            _robot.log_info(bla);
            setOutput("start_loc", start_loc);
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ExploreA::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList ExploreA::providedPorts()
{
    //return { OutputPort<std::pair<int,int>>("start_loc") }; // deprecated, was used to test shortest path alone
    return {};
}

ExploreB::ExploreB(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus ExploreB::tick()
{
    try {


        if (_robot.ExploreB()) {
            // will need to add logic to actually give output port a value
            // location of task (or start for coverage planner for example)
            std::pair<int,int> start_loc = _robot.getNextStartLocation(); // Location of first task in path (which here is ExploreA)
            std::string bla = "start_loc in ExploreB tick (x, y): " + std::to_string(start_loc.first) + ", " + std::to_string(start_loc.second);
            _robot.log_info(bla);
            setOutput("start_loc", start_loc);
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ExploreB::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList ExploreB::providedPorts()
{
    return {};
}

ExploreC::ExploreC(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus ExploreC::tick()
{
    try {


        if (_robot.ExploreC()) {
            // will need to add logic to actually give output port a value
            // location of task (or start for coverage planner for example)
            std::pair<int,int> start_loc = _robot.getNextStartLocation(); // Location of first task in path (which here is ExploreA)
            std::string bla = "start_loc in ExploreC tick (x, y): " + std::to_string(start_loc.first) + ", " + std::to_string(start_loc.second);
            _robot.log_info(bla);
            setOutput("start_loc", start_loc);
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ExploreC::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList ExploreC::providedPorts()
{
    return {};
}

ExploreD::ExploreD(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus ExploreD::tick()
{
    try {


        if (_robot.ExploreD()) {
            // will need to add logic to actually give output port a value
            // location of task (or start for coverage planner for example)
            std::pair<int,int> start_loc = _robot.getNextStartLocation(); // Location of first task in path (which here is ExploreA)
            std::string bla = "start_loc in ExploreD tick (x, y): " + std::to_string(start_loc.first) + ", " + std::to_string(start_loc.second);
            _robot.log_info(bla);
            setOutput("start_loc", start_loc);
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in ExploreD::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList ExploreD::providedPorts()
{
    return {};
}

FollowCoveragePath::FollowCoveragePath(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w, CoveragePath& cp)
    : StatefulActionNode(name, config), _robot(r), _world(w), _coverage_path_planner(cp)
    , _current_waypoint_index(0) {
    }

NodeStatus FollowCoveragePath::onStart()
{
    try {
        std::cout << "Planning coverage path for robot " << _robot.getID() << "..." << std::endl;

        // Cleaner to not pass info in via trigger condition since it is possible to access here
        // Although it may be redundant, this provides more flexibility to use different planners
        // Otherwise shortest path requires the port to pass a std::pair<int,int> location
        // whereas coverage path requires it to pass an unordered map of string,ints defining the x min/max + y min/max defining the quadrant area

        std::vector<int> task_path = _robot.getPath(); // Order tasks should be executed, by ID
        int current_task_id = task_path[0]; // This is only called if we already determined the task to be executed is coverage
        TaskInfo& current_task = _world.getTaskInfo(current_task_id); // Get task struct from world 
        std::unordered_map<std::string,int> area = current_task.area;
        
        Pose2D current_pose = _robot.getPose();
        // Pose2D goal_pose;
        // std::pair<int,int> goal_loc;
        // Check for location input, and if found, convert to pose by adding 0 for theta
        /*if (getInput<std::pair<int,int>>("area", goal_loc)) {
            goal_pose = {goal_loc.first, goal_loc.second,0};
        } else {
            _robot.log_info("Using default goal pose, so input not found");
            goal_pose = _robot.getGoalPose();
        }*/

        // std::string bla = "Goal pose for coverage path is: " + std::to_string(goal_pose.x) + ", " + std::to_string(goal_pose.y);
        // _robot.log_info(bla);
        
        // Init vector of waypoints, the plan
        _waypoints = _coverage_path_planner.plan(current_pose, area,
                                                _world.getX(), _world.getY());

        // for testing
        std::cout << "Generated waypoints:" << std::endl;
        for (size_t i = 0; i < _waypoints.size(); i++) {
            std::string waypoint_str = "Waypoint " + std::to_string(i) + ": (" + 
                                      std::to_string(_waypoints[i].x) + ", " + 
                                      std::to_string(_waypoints[i].y) + ", " + 
                                      std::to_string(_waypoints[i].theta) + ")";
            _robot.log_info(waypoint_str);
        }
        
        if (_waypoints.empty()) {
            std::cout << "No path found" << std::endl;
            return NodeStatus::FAILURE;
        }
        
        _current_waypoint_index = 0;
        std::cout << "Planned path with " << _waypoints.size() << " waypoints" << std::endl;
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus FollowCoveragePath::onRunning()
{
    try {
        if (_current_waypoint_index >= _waypoints.size()) {
            std::cout << "All waypoints completed!" << std::endl;

            // Movement is done!
            // Remove current first task from path since it has been completed
            _robot.removeCompletedTaskFromPath(); // Removes first task

            return NodeStatus::SUCCESS;
        }
        
        Pose2D waypoint = _waypoints[_current_waypoint_index];
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Using waypoint " << (_current_waypoint_index + 1) << "/" << _waypoints.size()
                  << ": " << waypoint.x << "/" << waypoint.y << std::endl;
        
        _robot.move(waypoint);

        _current_waypoint_index ++;
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void FollowCoveragePath::onHalted()
{
    std::cout << "FollowCoveragePath halted at waypoint " << _current_waypoint_index
              << "/" << _waypoints.size() << std::endl;
    _current_waypoint_index = 0;
    _waypoints.clear();
}

PortsList FollowCoveragePath::providedPorts()
{
    return {};
}