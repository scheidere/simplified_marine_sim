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

//int logger_robot_id = 1;

void saveReward(double time, double reward, const std::string& filename = "reward_data.csv") {
    std::ofstream file(filename, std::ios::app);  // Allow appending
    file << time << "," << reward << "\n";
    file.close();
}

void saveScore(double time, double score, const std::string& filename = "score_data.csv") {
    std::ofstream file(filename, std::ios::app);
    file << time << "," << score << "\n";
    file.close();
}

/*void saveDiscountedReward(double time, double reward, const std::string& filename = "discounted_reward_data.csv") {
    std::ofstream file(filename, std::ios::app);  // Allow appending
    file << time << "," << reward << "\n";
    file.close();
}*/

// moved to robot.cpp
/*void saveDistance(double time, double distance, const std::string& filename = "distance_data.csv") {
    std::ofstream file(filename, std::ios::app);  // Allow appending
    file << time << "," << distance << "\n";
    file.close();
}*/

void taskSuccessProcessing(World& _world, Robot& _robot, int current_task_id, int local_current_task_id) {

    // Current task id is id from path (either main id or subtask id if current robot is in helper mode)
    // Local current task id is always a subtask id, but will match current task id if robot in helper mode

    /*bool do_all = false;
    if (_robot.inHelperMode()) {
        // Helper robot has completed subtask, remove subtask from path, get reward, etc.

    } else {
        // Robot not in helper mode, check if current task is main or subtask and final one in main task subtask list
    }*/

    // // First, catch if robot doing main task and current task is not the final subtask needed to complete that main task
    // if (!_robot.inHelperMode() && !_world.isLastSubtask(current_task_id, local_current_task_id)) {
    //     // In this case, main assigned task not complete, so skip processing
    //     return
    // } else {

    // Reflect task completion if:
    // Option 1: robot in helper mode and this function is being called due to subtask success
    // Option 2: robot in main mode (main task id) and current subtask (local id) is the final required to complete main task
    if (_robot.inHelperMode() || _world.isLastSubtask(current_task_id, local_current_task_id)) {

        std::string plz = "In taskSuccessProcessing before getTaskInfo with task id: " + std::to_string(current_task_id);
        _world.log_info(plz);
        TaskInfo& current_task = _world.getTaskInfo(current_task_id); // Get task struct from world 
        double raw_reward = current_task.reward;
        double current_time = _robot.getCurrentTime();
        // double discount_factor = 0.999;  // Match your CBGA discount
        // double discounted_reward = raw_reward * pow(discount_factor, current_time);

        // Get task allocation score
        double task_score = _robot.getTaskScore(current_task_id);

        std::string rew = "Robot " + std::to_string(_robot.getID()) + 
                  " receives raw reward: " + std::to_string(raw_reward) + 
                  ", task allocation score: " + std::to_string(task_score) + 
                  " at time " + std::to_string(current_time);
        _robot.log_info(rew);

        // Add new reward to cumulative reward for whole team
        _world.updateCumulativeReward(raw_reward);  // Keep raw
        // _world.updateCumulativeDiscountedReward(discounted_reward);
        _world.updateCumulativeScore(task_score); 
        
        double& cumulative_reward = _world.getCumulativeReward();
        // double& cumulative_discounted = _world.getCumulativeDiscountedReward();
        double& cumulative_score = _world.getCumulativeScore(); 
        
        saveReward(current_time, cumulative_reward);
        // saveDiscountedReward(current_time, cumulative_discounted);
        saveScore(current_time, cumulative_score);

        _world.updateTaskCompletionLog(_robot.getID(), current_task_id);

        // Remove current first task from path since it has been completed
        _robot.removeCompletedTaskFromPathAndBundle(); // Removes first task
        // DO WE NEED TO REMOVE FROM BUNDLE AS WELL?? Or does this happen in bundleRemove... seems sketch

        _world.log_info("Paths at task completion:");
        _world.logCurrentTeamAssignment(); // Save current paths of all robots on team

        _world.log_info("Task progress at task completion:");
        _world.logCurrentTeamTaskProgress();

        _world.log_info("Current tasks completed by each robot: ");
        _world.logTaskCompletion();

    }
}

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
        //std::cout << "Checking if new info is available (any pings heard)..." << std::endl;\

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
        // std::string timing_log = "Robot " + std::to_string(_robot.getID()) + " Communicate at time: " + std::to_string(elapsed) + "ms";
        // _world.log_info(timing_log);

        // std::string blork1 = "Robot " + std::to_string(_robot.getID()) + " STARTING broadcast";
        // _world.log_info(blork1);  // Use world.log_info

        auto do_cbga = getInput<bool>("do_cbga").value_or(false);

        // Send messages
        std::string log_msg = "Robot " + std::to_string(_robot.getID()) + " broadcasting message...";
        _robot.log_info(log_msg);

        _robot.log_info("Timestamps BEFORE change in Communicate::tick in behavior_tree.cpp:");
        utils::logUnorderedMap(_robot.getTimestamps(),_robot);

        Message msg(_robot);
        //_robot.log_info("Timestamps test: ");
        //utils::logUnorderedMap(_robot.getTimestamps(), _robot);
        msg.broadcastMessage(_world);
        //std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay 

        // std::string blork2 = "Robot " + std::to_string(_robot.getID()) + " FINISHED broadcast";  
        // _world.log_info(blork2);

        std::string b = "Robot " + std::to_string(_robot.getID()) + " finished broadcasting.";
        _robot.log_info(b);

        _robot.log_info("Broadcasted the following message:");
        std::string bla = "ID: " + std::to_string(_robot.getID());
        _robot.log_info(bla);
        _robot.log_info("Winners: ");
        utils::logUnorderedMap(_robot.getWinners(), _robot);
        _robot.log_info("Winning bids: ");
        utils::logUnorderedMap(_robot.getWinningBids(), _robot);
        _robot.log_info("Winning bids matrix: ");
        utils::log2DVector(_robot.getWinningBidsMatrix(), _robot);
        _robot.log_info("Timestamps: ");
        utils::logUnorderedMap(_robot.getTimestamps(), _robot);
        _robot.log_info("Task progress: ");
        utils::logUnorderedMap(_robot.getTaskProgress(), _robot);
        _robot.log_info("Subtask_failures: ");
        utils::log2DUnorderedMap(_robot.getSubtaskFailures(), _robot);

        //_robot.log_info("in communicate node, before receiveMessages");

        /// testing... ///
        // std::unordered_map<int, std::vector<Msg>>& world_msg_tracker = _world.getMessageTracker();
        // std::string debug_msg = "Robot " + std::to_string(_robot.getID()) + " accessing tracker at address: " + std::to_string((uintptr_t)&world_msg_tracker);
        // _world.log_info(debug_msg);

        // _robot.log_info("World message tracker by robot:");
        // for (const auto& pair : world_msg_tracker) {
        //     int robot_id = pair.first;
        //     const std::vector<Msg>& messages = pair.second;
            
        //     std::string log_msg = "Robot " + std::to_string(robot_id) + " has messages from: ";
        //     for (const auto& msg : messages) {
        //         log_msg += std::to_string(msg.id) + " ";
        //     }
        //     log_msg += "(" + std::to_string(messages.size()) + " total)";
        //     _robot.log_info(log_msg);
        // }
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
            _robot.log_info("Winning bids matrix: ");
            utils::log2DVector(msg.winning_bids_matrix, _robot);
            _robot.log_info("Timestamps: ");
            utils::logUnorderedMap(msg.timestamps,_robot);
            _robot.log_info("Task progress: ");
            utils::logUnorderedMap(msg.task_progress, _robot);
            _robot.log_info("Subtask failures: ");
            utils::logUnorderedMap(msg.subtask_failures, _robot);
            _robot.log_info("Discovered obstacles: ");
            // utils::logMapOfVectors(msg.discovered_obstacles, _robot);
            utils::logMapOf2DVectorOfPoints(msg.discovered_obstacles, _robot);
        }
            

        //_robot.updateTimestamps(); // testing calling this in more specific spots in receiveMessages(), split into two parts
        if (do_cbga) {
            _robot.updateLocations(); // CBGA
        }
        _robot.log_info("task progress b4 update in comms node");
        utils::logUnorderedMap(_robot.getTaskProgress(), _robot);
        _robot.updateTaskProgress(); // CBGA and CBBA
        _robot.log_info("task progress after update in comms node");
        utils::logUnorderedMap(_robot.getTaskProgress(), _robot);

        _robot.log_info("discovered_obstacles b4 update in comms node");
        // utils::logMapOfVectors(_robot.getDiscoveredObstacles(), _robot);
        utils::logMapOf2DVectorOfPoints(_robot.getDiscoveredObstacles(), _robot);
        _robot.updateDiscoveredObstaclesPerNeighbors();
        _robot.log_info("discovered_obstacles after update in comms node");
        // utils::logMapOfVectors(_robot.getDiscoveredObstacles(), _robot);
        utils::logMapOf2DVectorOfPoints(_robot.getDiscoveredObstacles(), _robot);

        if (do_cbga) {
            // Merge subtask failures tracker (taking in info about how other robots have failed and need help)
            _robot.log_info("subtask failures b4 update in comms node");
            utils::logUnorderedMap(_robot.getSubtaskFailures(), _robot);
            _robot.updateSubtaskFailuresPerNeighbors(); // CBGA
            _robot.log_info("subtask failures after update in comms node");
            utils::logUnorderedMap(_robot.getSubtaskFailures(), _robot);
        

            // _robot.log_info("Testing subtask failures update, setting...");
            // _robot.testSubtaskFailuresUpdater();

            // Update doable tasks and subtasks lists per help needed from neighbors
            _robot.updateDoableTasks();
        }

        _world.log_info("Task progress after update via comms:");
        _world.logCurrentTeamTaskProgress();

        _robot.log_info("Timestamps AFTER change in Communicate::tick in behavior_tree.cpp:");
        utils::logUnorderedMap(_robot.getTimestamps(),_robot);
        // std::string plorp = "Robot " + std::to_string(_robot.getID()) + " Communicate returning SUCCESS";
        // _world.log_info(plorp);
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
    return {InputPort<bool>("do_cbga")};
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
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    _robot.setTaskAllocStartTime(start_time);

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

        if (do_cbga) {
            CBGA cbga(_robot, _world, _parser);
            cbga.resolveConflicts();
            // _robot.log_info("right before running cbga with test = true");
            // cbga.resolveConflicts(true); // for testing
            int& rounds = _robot.getNumCBGARounds();
            rounds++;
        } else {
            CBBA cbba(_robot, _world, _parser);
            cbba.resolveConflicts();
            //cbba.resolveConflicts(true); // for testing
            int& rounds = _robot.getNumCBBARounds();
            rounds++;
        }
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

        _robot.countConvergedIterations(do_cbga); // Compare robot beliefs to beliefs at previous iteration (stored in)

        int cumulative_convergence_count = _robot.getConvergenceCount(); // Number of iterations convergence has remained

        _robot.updateBeliefs(do_cbga);


        _robot.log_info("After convergence check at end of CBBA/CBGA round, bundle and path are the following:");
        std::vector<int> bundle = _robot.getBundle();
        _robot.log_info("Bundle:");
        utils::log1DVector(bundle, _robot);
        std::vector<int> path = _robot.getPath();
        _robot.log_info("Path:");
        utils::log1DVector(path, _robot);

        setOutput("cumulative_convergence_count_out", cumulative_convergence_count); // original    

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

            // Log time it took to reach convergence, maintained for the threshold number of rounds
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = end_time - _robot.getTaskAllocStartTime();
            double seconds = std::chrono::duration<double>(duration).count();
            std::string time = "Time to convergence: " + std::to_string(seconds) + " seconds";
            _robot.log_info(time);

            _robot.log_info("At convergence, so updating task progress tracker to prevent redundant allocation of tasks in later CBBA/CBGA rounds...");
            _robot.updateTaskProgressFromAssignment();

            _robot.log_info("Task progress tracker at convergence: ");
            utils::logUnorderedMap(_robot.getTaskProgress(), _robot);

            _world.logNeighbors();

            _world.log_info("Paths at convergence:");
            _world.logCurrentTeamAssignment(); // Save current paths of all robots on team

            //_world.logCurrentTeamTaskProgress(); // Save current task progress vectors

            _world.log_info("Messaging log at convergence:");
            _world.logMessagingLog();
        } else {
            // not at convergence, but at end of round
            _world.log_info("Messaging log at end of round - not at convergence:");
            _world.logMessagingLog();
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

        // Calling this just calls a few world functions and logs them, makes actual greedy timing consistent
        // Not necessary for function, just wanted consistent runtime for recording (perfectionism ftw - send help)
        _world.debugTaskAccess(1, _robot);

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
        std::string strt = "Planning shortest path for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);

        std::vector<int> task_path = _robot.getPath(); 
        int current_task_id = task_path[0];

        // **************************************** //
        // COUNT TASK (that shortest path is navigating to) AS BEGUN, stops CBGA from assigning to another robot when already being executed and therefore not an option anymore
        ////_robot.updateSingleTaskProgress(current_task_id,1); // id, started bool
        // **************************************** //

        _world.log_info("Task progress after single update in action start function:");
        _world.logCurrentTeamTaskProgress();
        
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
            std::string strt = "Completed shortest path for robot " + std::to_string(_robot.getID()) + "...";
            _world.log_info(strt);
            return NodeStatus::SUCCESS;
        }

        std::string strt = "Running shortest path for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);
        
        Pose2D waypoint = _waypoints[_current_waypoint_index];
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Using waypoint " << (_current_waypoint_index + 1) << "/" << _waypoints.size()
                  << ": " << waypoint.x << "/" << waypoint.y << std::endl;
        
        _robot.move(waypoint);

        // do we still want this below? no, it is now is robot.move
        // if (_robot.getID() == 2) { // 1 CHANGED TO 2 for now because CBBA has bug with not assigning anything to robot 1....

        //     //_world.log_info("in robot 1 logging area - shortest path");

        //     // Update cumulative team distance for plotting
        //     _world.updateCumulativeDistance();
        //     double cumulative_team_distance = _world.getCumulativeDistance();

        //     // Save distance at current time
        //     double current_time = _robot.getCurrentTime();
        //     saveDistance(current_time, cumulative_team_distance);
        // }
        
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

// timing bug not here
NodeStatus ExploreA::tick()
{
    try {

        _robot.log_info("in exploreA before check, path: ");
        utils::log1DVector(_robot.getPath(),_robot);

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
        std::string strt = "Planning coverage path for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);

        // testing timing
        _start_time = std::chrono::high_resolution_clock::now();
        _robot.log_info("FollowCoveragePath onStart() beginning");

        // Cleaner to not pass info in via trigger condition since it is possible to access here
        // Although it may be redundant, this provides more flexibility to use different planners
        // Otherwise shortest path requires the port to pass a std::pair<int,int> location
        // whereas coverage path requires it to pass an unordered map of string,ints defining the x min/max + y min/max defining the quadrant area

        std::vector<int> task_path = _robot.getPath(); // Order tasks should be executed, by ID
        int current_task_id = task_path[0]; // This is only called if we already determined the task to be executed is coverage
        ////TaskInfo& current_task = _world.getTaskInfo(current_task_id); // Get task struct from world 

        // **************************************** //
        // COUNT TASK AS BEGUN, stops CBGA from assigning to another robot when already being executed and therefore not an option anymore
        ////_robot.updateSingleTaskProgress(current_task_id,1);
        // **************************************** //

        // _world.log_info("Task progress after single update in action start function:");
        // _world.logCurrentTeamTaskProgress();

        // testing timing to diagnose delay
        auto world_start = std::chrono::high_resolution_clock::now();
        TaskInfo& current_task = _world.getTaskInfo(current_task_id);
        auto world_end = std::chrono::high_resolution_clock::now();
        double world_time = std::chrono::duration<double>(world_end - world_start).count();
        std::cout << "!!!!!!!!!!World access took: " << world_time << "s" << std::endl;

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
        // _waypoints = _coverage_path_planner.plan(current_pose, area,
        //                                         _world.getX(), _world.getY());
        auto planning_start = std::chrono::high_resolution_clock::now();
        _waypoints = _coverage_path_planner.plan(current_pose, area, _world.getX(), _world.getY());
        auto planning_end = std::chrono::high_resolution_clock::now();

        double planning_time = std::chrono::duration<double>(planning_end - planning_start).count();
        std::string b = "Robot " + std::to_string(_robot.getID()) + " path planning took: " + std::to_string(planning_time) + "s";
        _robot.log_info(b);

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

            // Get reward for the coverage path task just completed
            std::vector<int> task_path = _robot.getPath(); // Order tasks should be executed, by ID
            int current_task_id = task_path[0]; // This is only called if we already determined the task to be executed is coverage
            std::string hi = "Current_task_id at end: " + std::to_string(current_task_id);
            _robot.log_info(hi);
            TaskInfo& current_task = _world.getTaskInfo(current_task_id); // Get task struct from world 
            double reward = current_task.reward;

            std::string rew = "Robot " + std::to_string(_robot.getID()) + " receives reward of " + std::to_string(reward) + " for completing " + current_task.name; 
            _robot.log_info(rew);

            // Add new reward to cumulative reward for whole team
            _world.updateCumulativeReward(reward);
            //cumulative_reward += reward; potentially unsafe update

            double& cumulative_reward = _world.getCumulativeReward();

            // Save reward at current time
            double current_time = _robot.getCurrentTime();
            saveReward(current_time, cumulative_reward);

            _world.updateTaskCompletionLog(_robot.getID(), current_task_id);

            // Movement is done!
            // Remove current first task from path since it has been completed
            _robot.removeCompletedTaskFromPath(); // Removes first task

            auto end_time = std::chrono::high_resolution_clock::now();
            double total_start_time = std::chrono::duration<double>(end_time - _start_time).count();
            std::string p = "FollowCoveragePath onStart() total time: " + std::to_string(total_start_time) + "s";
            _robot.log_info(p);

            _world.log_info("Paths at task completion:");
            _world.logCurrentTeamAssignment(); // Save current paths of all robots on team

            _world.log_info("Task progress at task completion:");
            _world.logCurrentTeamTaskProgress();

            _world.log_info("Current tasks completed by each robot: ");
            _world.logTaskCompletion();

            std::string strt = "Completed coverage path for robot " + std::to_string(_robot.getID()) + "...";
            _world.log_info(strt);

            return NodeStatus::SUCCESS;
        }

        std::string strt = "Running coverage path for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);
        
        Pose2D waypoint = _waypoints[_current_waypoint_index];
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Using waypoint " << (_current_waypoint_index + 1) << "/" << _waypoints.size()
                  << ": " << waypoint.x << "/" << waypoint.y << std::endl;
        
        // testing timing
        //_robot.move(waypoint);
        auto move_start = std::chrono::high_resolution_clock::now();
        _robot.move(waypoint);
        auto move_end = std::chrono::high_resolution_clock::now();

        double move_time = std::chrono::duration<double>(move_end - move_start).count();
        std::string g = "Robot " + std::to_string(_robot.getID()) + " waypoint " + std::to_string(_current_waypoint_index) + " move took: " +  std::to_string(move_time) + "s";
        _robot.log_info(g);


        // THIS IS REDUNDANT NOW, save distance moved to robot.move
        // Only log team info once (so only one robot should do it even though it pertains to whole team)
        // if (_robot.getID() == 2) { // 1 CHANGED TO 2 for now because CBBA has bug with not assigning anything to robot 1....
        //     //_world.log_info("in robot 1 logging area - coverage path");

        //     // Save cumulative team distance for plotting
        //     _world.updateCumulativeDistance();
        //     double cumulative_team_distance = _world.getCumulativeDistance();

        //     // Save distance at current time
        //     double current_time = _robot.getCurrentTime();
        //     saveDistance(current_time, cumulative_team_distance);
        // }

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


PathClearingNeeded::PathClearingNeeded(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus PathClearingNeeded::tick()
{
    try {

        if (_robot.PathClearingNeeded()) { //not implemented in robot yet
            std::pair<int,int> start_loc = _robot.getNextStartLocation(); // Location of first task in path (which here is ClearPath)
            std::string bla = "start_loc in PathClearingNeeded tick (x, y): " + std::to_string(start_loc.first) + ", " + std::to_string(start_loc.second);
            _robot.log_info(bla);
            setOutput("start_loc", start_loc);
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in PathClearingNeeded::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList PathClearingNeeded::providedPorts()
{
    return { OutputPort<std::pair<int,int>>("start_loc") };
}


ClearPath::ClearPath(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w)
    : StatefulActionNode(name, config), _robot(r), _world(w) {
    }

NodeStatus ClearPath::onStart()
{
    try {
        std::cout << "Robot " << _robot.getID() << " going to clear path..." << std::endl;
        std::string strt = "Starting clear path for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);
        
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus ClearPath::onRunning()
{
    try {

        // If here, ClearPath is current task because condition node returned true to get here in BT
        // Get location to clear path
        std::vector<int> task_path = _robot.getPath(); // Order tasks should be executed, by ID
        int current_task_id = task_path[0]; // This has to be clear_path to be in onRunning per BT
        // TaskInfo& current_task = _world.getTaskInfo(current_task_id); // Get task struct from world 
        // std::pair<int,int> location = current_task.location; // Get location of this clear path task

        if (_world.fullGroupPresent(current_task_id)) {

            // Entire group is present at task vicinity, so after a short delay we count this as path cleared
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // Get reward for the coverage path task just completed
            std::vector<int> task_path = _robot.getPath(); // Order tasks should be executed, by ID
            int current_task_id = task_path[0]; // This is only called if we already determined the task to be executed is coverage
            std::string hi = "Current_task_id at end: " + std::to_string(current_task_id);
            _robot.log_info(hi);
            TaskInfo& current_task = _world.getTaskInfo(current_task_id); // Get task struct from world 
            double reward = current_task.reward;

            std::string rew = "Robot " + std::to_string(_robot.getID()) + " receives reward of " + std::to_string(reward) + " for completing " + current_task.name; 
            _robot.log_info(rew);

            // Add new reward to cumulative reward for whole team
            _world.updateCumulativeReward(reward);
            //cumulative_reward += reward; potentially unsafe update

            double& cumulative_reward = _world.getCumulativeReward();

            // Save reward at current time
            double current_time = _robot.getCurrentTime();
            saveReward(current_time, cumulative_reward);

            _world.updateTaskCompletionLog(_robot.getID(), current_task_id);

            // Remove current first task from path since it has been completed
            _robot.removeCompletedTaskFromPath(); // Removes first task from path

            _world.log_info("Paths at task completion:");
            _world.logCurrentTeamAssignment(); // Save current paths of all robots on team

            _world.log_info("Task progress at task completion:");
            _world.logCurrentTeamTaskProgress();

            _world.log_info("Current tasks completed by each robot: ");
            _world.logTaskCompletion();

            std::string strt = "Completed clear path for robot " + std::to_string(_robot.getID()) + "...";
            _world.log_info(strt);

            return NodeStatus::SUCCESS;
        }

        std::string strt = "Running clear path for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);

        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void ClearPath::onHalted()
{
    std::cout << "Clear path halted." << std::endl;
}

PortsList ClearPath::providedPorts()
{
    return {};
}

CollectSample::CollectSample(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus CollectSample::tick()
{
    try {

        // if type of first task in path is "Collect" (this covers main task CollectSample and subtasks for helper robots here ExtractSample or LoadSample)

        if (_robot.SampleCollectionNeeded()) {
            std::pair<int,int> start_loc = _robot.getNextStartLocation(); // Location of first task in path (which here is type "collect")
            std::string bla = "start_loc in SampleCollectionNeeded tick (x, y): " + std::to_string(start_loc.first) + ", " + std::to_string(start_loc.second);
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

PortsList CollectSample::providedPorts()
{
    return { OutputPort<std::pair<int,int>>("start_loc") };
}

// ExtractSample::ExtractSample(const std::string& name, const NodeConfig& config,
//                                        Robot& r, World& w)
//     : StatefulActionNode(name, config), _robot(r), _world(w) {
//         // THIS IS A SUB TASK OF A COMBO TASK, 
//     }

// NodeStatus ExtractSample::onStart()
// {
//     try {
//         std::string strt = "Starting extract sample for robot " + std::to_string(_robot.getID()) + "...";
//         _world.log_info(strt);
        
//         return NodeStatus::RUNNING;
        
//     } catch (const std::exception& e) {
//         std::cerr << "Exception: " << e.what() << std::endl;
//         return NodeStatus::FAILURE;
//     }
// }

// NodeStatus ExtractSample::onRunning()
// {
//     try {

//         // If here, ExtractSample is current task because CollectSample condition node and counter sequence logic parent node

//         // Need a way to differentiate between whether the robot running this is helper or helpee
//         // Will change how long it takes to do, to denote handoff occurring (extract and hand off if helpee can still load)
//         // TODO TODO TODO

//         // Get location to extract sample
//          std::vector<int> task_path = _robot.getPath(); // Order tasks should be executed, by ID
//         int current_task_id = task_path[0]; // Should either be main task id (for Collect_Sample) or subtask itself (Extract_Sample)

//         // If current task is main task, i.e., Collect Sample, then no fault has occurred
//         // Task is solo, don't need to wait for group (it's just a group of 1)

//         // if task_info[current_task_id].name == "Collect_Sample" && cooperate = false:
//         //     // Baseline solo run of main task
//         // else if cooperate =


//         //*** Once full failure (at threshold) ***//
//         // Remove assignment of current robot from Collect_Sample and add to Extract_Sample
//         // This gives CBGA info about the failure, making x


//         // Get prerequisite number of failures before full failure accepted for this subtask
//         int failure_threshold = _world.getPrerequisiteFailureThreshold("Extract_Sample");

//         setOutput("failure_threshold", failure_threshold);
//         auto do_cbga = getInput<bool>("cooperate"); // Denotes whether pre (solo task) or post fault (co-op between helper and helpee)


//         if (_world.fullGroupPresent(current_task_id)) {

//             // Don't need to wait for others, since this is a part of a combo task (as opposed to being a co-op task)
//             std::this_thread::sleep_for(std::chrono::milliseconds(50)); // short delay represents extraction operation

//             // Get reward for the extraction task just completed
//             std::vector<int> task_path = _robot.getPath(); // Order tasks should be executed, by ID
//             int current_task_id = task_path[0];

//             TaskInfo& current_task = _world.getSubTaskInfo(current_task_id); // Get task struct from world 
//             double reward = current_task.reward;

//             std::string rew = "Robot " + std::to_string(_robot.getID()) + " receives reward of " + std::to_string(reward) + " for completing " + current_task.name; 
//             _robot.log_info(rew);

//             // Add new reward to cumulative reward for whole team
//             _world.updateCumulativeReward(reward);
//             double& cumulative_reward = _world.getCumulativeReward();

//             // Save reward at current time
//             double current_time = _robot.getCurrentTime();
//             saveReward(current_time, cumulative_reward);

//             _world.updateTaskCompletionLog(_robot.getID(), current_task_id);

//             // Remove current first task from path since it has been completed
//             _robot.removeCompletedTaskFromPath(); // Removes first task from path

//             _world.log_info("Paths at task completion:");
//             _world.logCurrentTeamAssignment(); // Save current paths of all robots on team

//             _world.log_info("Task progress at task completion:");
//             _world.logCurrentTeamTaskProgress();

//             _world.log_info("Current tasks completed by each robot: ");
//             _world.logTaskCompletion();

//             std::string strt = "Completed ExtractSample for robot " + std::to_string(_robot.getID()) + "...";
//             _world.log_info(strt);

//             return NodeStatus::SUCCESS;

//         }

//         std::string strt = "Running extract sample for robot " + std::to_string(_robot.getID()) + "...";
//         _world.log_info(strt);

//         return NodeStatus::RUNNING;


//         if (_world.fullGroupPresent(current_task_id)) {

//             // Entire group is present at task vicinity, so after a short delay we count this as path cleared
//             std::this_thread::sleep_for(std::chrono::milliseconds(50));

//             // Get reward for the coverage path task just completed
//             std::vector<int> task_path = _robot.getPath(); // Order tasks should be executed, by ID
//             int current_task_id = task_path[0]; // This is only called if we already determined the task to be executed is coverage
//             std::string hi = "Current_task_id at end: " + std::to_string(current_task_id);
//             _robot.log_info(hi);
//             TaskInfo& current_task = _world.getTaskInfo(current_task_id); // Get task struct from world 
//             double reward = current_task.reward;

//             std::string rew = "Robot " + std::to_string(_robot.getID()) + " receives reward of " + std::to_string(reward) + " for completing " + current_task.name; 
//             _robot.log_info(rew);

//             // Add new reward to cumulative reward for whole team
//             _world.updateCumulativeReward(reward);
//             //cumulative_reward += reward; potentially unsafe update

//             double& cumulative_reward = _world.getCumulativeReward();

//             // Save reward at current time
//             double current_time = _robot.getCurrentTime();
//             saveReward(current_time, cumulative_reward);

//             _world.updateTaskCompletionLog(_robot.getID(), current_task_id);

//             // Remove current first task from path since it has been completed
//             _robot.removeCompletedTaskFromPath(); // Removes first task from path

//             _world.log_info("Paths at task completion:");
//             _world.logCurrentTeamAssignment(); // Save current paths of all robots on team

//             _world.log_info("Task progress at task completion:");
//             _world.logCurrentTeamTaskProgress();

//             _world.log_info("Current tasks completed by each robot: ");
//             _world.logTaskCompletion();

//             std::string strt = "Completed clear path for robot " + std::to_string(_robot.getID()) + "...";
//             _world.log_info(strt);

//             return NodeStatus::SUCCESS;
//         }

//         std::string strt = "Running extract sample for robot " + std::to_string(_robot.getID()) + "...";
//         _world.log_info(strt);

//         return NodeStatus::RUNNING;
        
//     } catch (const std::exception& e) {
//         std::cerr << "Exception: " << e.what() << std::endl;
//         return NodeStatus::FAILURE;
//     }
// }

// void ExtractSample::onHalted()
// {
//     std::cout << "Extract sample halted." << std::endl;
// }

// PortsList ExtractSample::providedPorts()
// {
//     return {OutputPort<int>("failure_threshold"), // Passed to counter sequence parent node (because of world info access here)
//             InputPort<bool>("do_cbga")}; // Passed from parent counter sequence node (since this is a combo action)
// }

HandleFailures::HandleFailures(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : StatefulActionNode(name, config), _world(world), _robot(robot) {}

NodeStatus HandleFailures::onStart() {

    _robot.log_info("in HandleFailures node start");

    return NodeStatus::RUNNING;
}

NodeStatus HandleFailures::onRunning() {

    try {

        std::cout << "\nRunning HandleFailures (in onRunning)..." << std::endl;
        _robot.log_info("\nRunning HandleFailures (in onRunning)...");
    
        std::string a = "Robot " + std::to_string(_robot.getID()) + " in HandleFailures...";
        _world.log_info(a);

        // std::unordered_map<int,bool> current_subtask_failures = getInput<std::unordered_map<int,bool>>("self_subtask_failures").value();

        std::unordered_map<int,bool> current_subtask_failures;
        auto input = getInput<std::unordered_map<int,bool>>("self_subtask_failures");
        if (input) {
            current_subtask_failures = input.value();
            std::string s = "current_subtask_failures: ";
            _robot.log_info(s);
            utils::logUnorderedMap(current_subtask_failures, _robot);
            _world.log_info(s);
            utils::logUnorderedMapWorld(current_subtask_failures, _world);
        } else {
            // First tick - initialize with empty
            current_subtask_failures = std::unordered_map<int,bool>();
        }

        _robot.log_info("are we here in HandleFailures node");

        // Process input and provide output via HandleFailures robot function
        std::pair<std::pair<int,bool>,std::map<int,int>> scope_and_threshold_info = _robot.HandleFailures(current_subtask_failures);
        std::pair<int,bool> current_task_id_scope = scope_and_threshold_info.first;
        int current_task_id = current_task_id_scope.first;
        bool task_is_main = current_task_id_scope.second; // main vs subtask bool
        std::map<int,int> subtask_failure_thresholds = scope_and_threshold_info.second;

        std::string b = "task_is_main in HandleFailures node: " + std::to_string(task_is_main);
        _robot.log_info(b);
        _world.log_info(b);
        std::string c = "current_task_id in HandleFailures node: " + std::to_string(current_task_id);
        _world.log_info(c);

        // Tell counter sequence whether current task is main or sub
        setOutput("task_is_main", task_is_main);

        // Tell counter sequence what the current task is, by id (will only be used if it is a subtask)
        setOutput("current_task_id", current_task_id);

        // Tell counter sequence how many times it should attempt a failing subtask before counting it as truly failed
        setOutput("subtask_failure_thresholds", subtask_failure_thresholds);
      
        return NodeStatus::RUNNING; // For use with counter sequence, to allow repeated ticking of HandleFailure
    } catch (const std::exception& e) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\nException caught in HandleFailures::tick: " << e.what() << std::endl;
        _robot.log_info("HandleFailures node FAILURE");
        return NodeStatus::FAILURE;
    }
}

void HandleFailures::onHalted() {}

PortsList HandleFailures::providedPorts()
{
    return { InputPort<std::unordered_map<int,bool>>("self_subtask_failures"),
             OutputPort<bool>("task_is_main"),
             OutputPort<int>("current_task_id"),
             OutputPort<std::map<int,int>>("subtask_failure_thresholds")};
}

TaskNeededNow::TaskNeededNow(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus TaskNeededNow::tick()
{
    try {   
        _robot.log_info("in TaskNeededNow");

        if (_robot.TaskNeededNow()) { 
            _robot.log_info("About to return success - task needed now!");
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in TaskNeededNow::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList TaskNeededNow::providedPorts()
{
    return { OutputPort<std::pair<int,int>>("start_loc") };
}

Subtask_1::Subtask_1(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w, ShortestPath& sp)
    : StatefulActionNode(name, config), _robot(r), _world(w), _shortest_path_planner(sp) {
    }

NodeStatus Subtask_1::onStart()
{
    try {
        std::cout << "Robot " << _robot.getID() << " doing subtask 1..." << std::endl;
        std::string strt = "Starting subtask 1 for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);

        _world.log_info("Task progress after single update in action start function:");
        _world.logCurrentTeamTaskProgress();

        // std::vector<int> task_path = _robot.getPath(); 
        // int current_task_id = task_path[0];
        // TaskInfo& current_task = _world.getTaskInfo(current_task_id);
        // std::pair<int,int> goal_loc = current_task.location;
        // Pose2D goal_pose = {goal_loc.first, goal_loc.second,0};

        // Above lines now in this function
        Pose2D goal_pose = _robot.getCurrentGoalPose();

        std::string bla = "Goal loc for subtask 1 is: " + std::to_string(goal_pose.x) + ", " + std::to_string(goal_pose.y);
        _robot.log_info(bla);
        _world.log_info(bla);

        Pose2D current_pose = _robot.getPose();

        // Init vector of waypoints, the plan
        _waypoints = _shortest_path_planner.plan(current_pose, goal_pose,
                                                _world.getX(), _world.getY());
        
        if (_waypoints.empty()) {
            std::cout << "No path found" << std::endl;
            return NodeStatus::FAILURE;
        }
        
        _current_waypoint_index = 0;
        std::string borp = "Planned path with " + std::to_string(_waypoints.size()) + " waypoints";
        _robot.log_info(borp);

        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus Subtask_1::onRunning()
{
    try {

        // If here, Subtask 1 is current task because condition node returned true to get here in BT
        // Just a dummy task for testing counter sequence

        //_world.log_info("in subtask 1 onRunning");

        std::string e = "Robot " + std::to_string(_robot.getID()) + " in onRunning for subtask 1";
        _world.log_info(e);

         // Current_task_id will either be the maintask this subtask is part of (main mode), or the id of this subtask itself (helper mode)
        std::vector<int> path = _robot.getPath();
        int current_task_id = path[0];
        if (_world.isSubtaskID(current_task_id)) {
            std::string plz = "Current_task_id: " + std::to_string(current_task_id);
            _robot.log_info(plz);
            _robot.log_info("in is subtask if statement in subtask 1 onRunning");
            _robot.setHelperMode(true);
        } // otherwise remains default false, which allows potential fault injections

        bool group_present = _world.fullGroupPresent(current_task_id);
        std::string gp = "fullGroupPresent: " + std::to_string(group_present);
        _robot.log_info(gp);

        // Traverse to task location and wait for group, if applicable
        if (_world.fullGroupPresent(current_task_id)) {

            // just for testing
            TaskInfo& task = _world.getTaskInfo(current_task_id);
            std::string msg1 = "GROUP PRESENT! group_size needed: " + std::to_string(task.group_size);
            _robot.log_info(msg1);
            std::string msg2 = "Robot at: " + std::to_string(_robot.getPose().x) + ", " + std::to_string(_robot.getPose().y);
            _robot.log_info(msg2);
            std::string msg3 = "Task location: " + std::to_string(task.location.first) + ", " + std::to_string(task.location.second);
            _robot.log_info(msg3);
            // just for testing

            // Now that we have checked main vs sub logic, need to update current task to this subtask id for fault injection/recovery logic
            std::string name = "Test_Subtask_1";
            int local_current_task_id = _world.getSubtaskID(name);

            std::string strt = "Running subtask 1 for robot " + std::to_string(_robot.getID()) + "...";
            _world.log_info(strt);

            std::string hi = "Current_task_id: " + std::to_string(local_current_task_id);
            _world.log_info(hi);

            // Allow world to inject fault, or not
            bool fault_flag = _world.getFaultInjectionFlag(local_current_task_id);

            std::string hii = "fault_flag: " + std::to_string(fault_flag);
            _world.log_info(hii);

            std::string d = "Robot " + std::to_string(_robot.getID()) + " right before return block for subtask 1";
            _world.log_info(d);

            if (fault_flag && !_robot.inHelperMode()) {
                _robot.log_info("in failure return for subtask 1");
                std::string a = "Robot " + std::to_string(_robot.getID()) + " in failure return for subtask 1";
                _world.log_info(a);
                return NodeStatus::FAILURE;
            } else if (_robot.inHelperMode()) { // For now, we always allow helper robot to successfully help
                _robot.log_info("helper helping with subtask 1 now!");
                // World must detect that helper has helped, and reflect change by reseting fault injection flag from 1 (cause fault) to 0
                _world.updateFaultInjectionTracker(local_current_task_id,0); // Helper resolves fault
                _robot.setHelperMode(false); // No longer a helper for this subtask, because fault resolved now
                std::string hep = "fault recovered with subtask 1, helper succeeds";
                _robot.log_info(hep);
                std::string b = "Robot " + std::to_string(_robot.getID()) + " in helper mode for subtask 1";
                _world.log_info(b);
            }

            taskSuccessProcessing(_world, _robot, current_task_id, local_current_task_id);

            // for obstacle detection testing ONLY
            _robot.log_info("testing log of discovered_obstacles");
            utils::logMapOfVectors(_robot.getDiscoveredObstacles(), _robot);

            std::string herp = "processing done, subtask 1 returning success";
            _robot.log_info(herp);
            std::string c = "Robot " + std::to_string(_robot.getID()) + " at success return for subtask 1";
            _world.log_info(c);
            // If in helper mode or not, permitted to return success here, i.e., no fault
            return NodeStatus::SUCCESS;
        

        }

        std::string strt = "Running subtask 1 for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);

        if (_current_waypoint_index >= _waypoints.size()) {
            std::cout << "All waypoints completed!" << std::endl;
            std::string strt = "Completed shortest path to subask 1 for robot " + std::to_string(_robot.getID()) + "...";
            _world.log_info(strt);
            return NodeStatus::RUNNING;
            // return NodeStatus::SUCCESS; // for testing without group part above
        }
        
        Pose2D waypoint = _waypoints[_current_waypoint_index];
        std::string wp = "Waypoint coords: (" + std::to_string(waypoint.x) + ", " + std::to_string(waypoint.y) + ")";
        _robot.log_info(wp);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Using waypoint " << (_current_waypoint_index + 1) << "/" << _waypoints.size()
                  << ": " << waypoint.x << "/" << waypoint.y << std::endl;


        // Check if next waypoint is traversable for current robot type
        if(_robot.foundObstacle(waypoint)) {
            _robot.log_info("in found obstacle in subtask 1");

            // Clear plan, replan, continue on new plan
            _waypoints.clear(); 
            Pose2D current_pose = _robot.getPose();
            Pose2D goal_pose = _robot.getCurrentGoalPose();
            _waypoints = _shortest_path_planner.plan(current_pose, goal_pose, _world.getX(), _world.getY());

            if (_waypoints.empty()) {
                std::cout << "No path found" << std::endl;
                return NodeStatus::FAILURE;
            }

            _current_waypoint_index = 0;

            return NodeStatus::RUNNING;
        }
        
        _robot.move(waypoint);

        _current_waypoint_index++;
        return NodeStatus::RUNNING;

        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void Subtask_1::onHalted()
{
    std::cout << "Test Subtask 1 halted." << std::endl;
}

PortsList Subtask_1::providedPorts()
{
    return {};
}

Subtask_2::Subtask_2(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w)
    : StatefulActionNode(name, config), _robot(r), _world(w) {
    }

NodeStatus Subtask_2::onStart()
{
    try {
        std::cout << "Robot " << _robot.getID() << " doing subtask 2..." << std::endl;
        std::string strt = "Starting subtask 2 for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);
        
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus Subtask_2::onRunning()
{
    try {

        // If here, Subtask 2 is current task because condition node returned true to get here in BT
        // Just a dummy task for testing counter sequence

        //_world.log_info("in subtask 2 onRunning");

        std::string e = "Robot " + std::to_string(_robot.getID()) + " in onRunning for subtask 2";
        _world.log_info(e);

        // Current_task_id will either be the maintask this subtask is part of (main mode), or the id of this subtask itself (helper mode)
        std::vector<int> path = _robot.getPath();
        int current_task_id = path[0];
        if (_world.isSubtaskID(current_task_id)) {
            std::string plz = "Current_task_id: " + std::to_string(current_task_id);
            _robot.log_info(plz);
            _robot.log_info("in is subtask if statement in subtask 2 onRunning");
            _robot.setHelperMode(true);
        } // otherwise remains default false, which allows potential fault injections

        // Now that we have checked main vs sub logic, need to update current task to this subtask id for fault injection/recovery logic
        std::string name = "Test_Subtask_2";
        int local_current_task_id = _world.getSubtaskID(name);

        std::string strt = "Running subtask 2 for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);

        std::string hi = "Current_task_id: " + std::to_string(local_current_task_id);
        _world.log_info(hi);

        // Allow world to inject fault, or not
        bool fault_flag = _world.getFaultInjectionFlag(local_current_task_id);

        std::string hii = "fault_flag: " + std::to_string(fault_flag);
        _world.log_info(hii);

        // testing helper
        std::string eep = "robot in helper mode?: " + std::to_string(_robot.inHelperMode());
        _robot.log_info(eep);

        std::string d = "Robot " + std::to_string(_robot.getID()) + " right before return block for subtask 2";
        _world.log_info(d);

        if (fault_flag && !_robot.inHelperMode()) {
            _robot.log_info("in failure return for subtask 2");
            std::string a = "Robot " + std::to_string(_robot.getID()) + " in failure return for subtask 2";
            _world.log_info(a);
            return NodeStatus::FAILURE;
        } else if (_robot.inHelperMode()) { // For now, we always allow helper robot to successfully help
            _robot.log_info("helper helping with subtask 2 now!");
            // World must detect that helper has helped, and reflect change by reseting fault injection flag from 1 (cause fault) to 0
            _world.updateFaultInjectionTracker(local_current_task_id,0); // Helper resolves fault
            _robot.setHelperMode(false); // No longer a helper for this subtask, because fault resolved now
            std::string hep = "fault recovered with subtask 2, helper succeeds";
            _robot.log_info(hep);
            std::string b = "Robot " + std::to_string(_robot.getID()) + " in helper mode for subtask 2";
            _world.log_info(b);
        }

        taskSuccessProcessing(_world, _robot, current_task_id, local_current_task_id);

        std::string herp = "processing done, subtask 2 returning success";
        _robot.log_info(herp);
        std::string c = "Robot " + std::to_string(_robot.getID()) + " at success return for subtask 2";
        _world.log_info(c);
        // If in helper mode or not, permitted to return success here, i.e., no fault
        return NodeStatus::SUCCESS;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void Subtask_2::onHalted()
{
    std::cout << "Test Subtask 2 halted." << std::endl;
}

PortsList Subtask_2::providedPorts()
{
    return {};
}

TestShortPath::TestShortPath(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

// timing bug not here
NodeStatus TestShortPath::tick()
{
    try {

        _robot.log_info("in TestShortPath before check");

        std::pair<int,int> task_loc = {150,150}; // Location of first task in path (which here is ExploreA)
        std::string bla = "task_loc in TestShortPath tick (x, y): " + std::to_string(task_loc.first) + ", " + std::to_string(task_loc.second);
        _robot.log_info(bla);
        setOutput("task_loc", task_loc);
        return NodeStatus::SUCCESS;


    } catch (const std::exception& e) {
        std::cerr << "Exception caught in TestShortPath::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList TestShortPath::providedPorts()
{
    return { OutputPort<std::pair<int,int>>("task_loc") }; // deprecated, was used to test shortest path alone
    // return {};
}

// requires fancier action node function to work in just one subtree
/*DoImageArea::DoImageArea(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus DoImageArea::tick()
{
    try {   
        _robot.log_info("in DoImageArea");

        if (_robot.DoImageArea()) { 
            _robot.log_info("About to return success - ImageArea task or action needed now!");
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in DoImageArea::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList DoImageArea::providedPorts()
{
    return { };
}*/

ImageArea::ImageArea(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w, CoveragePath& cp)
    : StatefulActionNode(name, config), _robot(r), _world(w), _coverage_path_planner(cp) {
    // : RepeatableStatefulActionNode(name, config), _robot(r), _world(w), _coverage_path_planner(cp) {

    }

NodeStatus ImageArea::onStart()
{
    try {
        std::cout << "Robot " << _robot.getID() << " doing ImageArea..." << std::endl;
        std::string strt = "Starting ImageArea for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);
        _robot.log_info("Starting ImageArea...");

        _world.log_info("Task progress after single update in action start function:");
        _world.logCurrentTeamTaskProgress();

        std::vector<int> task_path = _robot.getPath(); 
        int current_task_id = task_path[0];
        std::string plz = "In ImageArea onStart before getTaskInfo with task id: " + std::to_string(current_task_id);
        _world.log_info(plz);
        TaskInfo& current_task = _world.getTaskInfo(current_task_id);
        std::unordered_map<std::string,int> area = current_task.area;

        std::string bla = "Area for ImageArea task - xmin: " + std::to_string(area["xmin"]) + 
                          ", xmax: " + std::to_string(area["xmax"]) + 
                          ", ymin: " + std::to_string(area["ymin"]) + 
                          ", ymax: " + std::to_string(area["ymax"]);
        _robot.log_info(bla);
        _world.log_info(bla);

        Pose2D current_pose = _robot.getPose();

        // Init vector of waypoints, the plan
        _waypoints = _coverage_path_planner.plan(current_pose, area,
                                                _world.getX(), _world.getY());
        
        if (_waypoints.empty()) {
            std::cout << "No coverage path found" << std::endl;
            return NodeStatus::FAILURE;
        }
        
        _current_waypoint_index = 0;
        std::string borp = "Planned coverage path with " + std::to_string(_waypoints.size()) + " waypoints";
        _robot.log_info(borp);

        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus ImageArea::onRunning()
{
    try {
        _robot.log_info("in onRunning for ImageArea...");

        std::vector<int> path = _robot.getPath();
        int current_task_id = path[0];
        
        // Check if this is a helper responding to a failure
        if (_world.isSubtaskID(current_task_id)) {
            _robot.log_info("Helper mode - current_task_id is subtask: " + std::to_string(current_task_id));
            _robot.setHelperMode(true);
        }
        
        // Get action ID for fault injection
        std::string plz = "In ImageArea onRunning before getTaskInfo with task id: " + std::to_string(current_task_id);
        _world.log_info(plz);
        TaskInfo& task = _world.getTaskInfo(current_task_id);
        int local_current_task_id;
        if (_robot.inHelperMode()) {
            local_current_task_id = current_task_id;
        } else {
            local_current_task_id = task.subtasks[0];
        }
        
        std::string lctid = "local_current_task_id for fault injection: " + std::to_string(local_current_task_id);
        _robot.log_info(lctid);

        bool fault_flag = _world.getFaultInjectionFlag(local_current_task_id);
        _robot.log_info("fault_flag: " + std::to_string(fault_flag));
        
        if (fault_flag && !_robot.inHelperMode()) {
            // Main robot encounters fault - wait for helper
            _robot.log_info("Fault encountered - waiting for helper");
            return NodeStatus::RUNNING;
        }
        
        if (_robot.inHelperMode()) {
            // Helper clears the fault so original can proceed
            _robot.log_info("Helper clearing fault for original robot");
            _world.updateFaultInjectionTracker(local_current_task_id, 0);
            _robot.setHelperMode(false);
            // Helper ALSO does the coverage task (both robots work together)
        }
        
        // Both robots do coverage (original + helper working together)
        if (_current_waypoint_index < _waypoints.size()) {
            Pose2D waypoint = _waypoints[_current_waypoint_index];
            
            if(_robot.foundObstacle(waypoint)) {
                _waypoints.clear(); 
                std::unordered_map<std::string,int> new_area = _robot.calculateRemainingAreaToCover(task.area);
                _waypoints = _coverage_path_planner.plan(_robot.getPose(), new_area, _world.getX(), _world.getY());
                if (_waypoints.empty()) {
                    return NodeStatus::FAILURE;
                }
                _current_waypoint_index = 0;
                return NodeStatus::RUNNING;
            }
            
            _robot.move(waypoint);
            _current_waypoint_index++;
            return NodeStatus::RUNNING;
        }

        // Coverage complete
        taskSuccessProcessing(_world, _robot, current_task_id, local_current_task_id);
        _robot.log_info("Returning success for completion of ImageArea");
        return NodeStatus::SUCCESS;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void ImageArea::onHalted()
{
    std::cout << "Test ImageArea halted." << std::endl;
}

PortsList ImageArea::providedPorts()
{
    return {};
}

DoImageArea1::DoImageArea1(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus DoImageArea1::tick()
{
    try {   
        _robot.log_info("in DoImageArea1");

        if (_robot.DoImageArea1()) { 
            _robot.log_info("About to return success - ImageArea1 task or action needed now!");
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in DoImageArea1::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList DoImageArea1::providedPorts()
{
    return { };
}

DoImageArea2::DoImageArea2(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus DoImageArea2::tick()
{
    try {   
        _robot.log_info("in DoImageArea2");

        if (_robot.DoImageArea2()) { 
            _robot.log_info("About to return success - ImageArea2 task or action needed now!");
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in DoImageArea2::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList DoImageArea2::providedPorts()
{
    return { };
}

DoImageArea3::DoImageArea3(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus DoImageArea3::tick()
{
    try {   
        _robot.log_info("in DoImageArea3");

        if (_robot.DoImageArea3()) { 
            _robot.log_info("About to return success - ImageArea3 task or action needed now!");
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in DoImageArea3::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList DoImageArea3::providedPorts()
{
    return { };
}

DoImageArea4::DoImageArea4(const std::string& name, const NodeConfig& config, Robot& robot, World& world)
    : ConditionNode(name, config), _robot(robot) {}       

NodeStatus DoImageArea4::tick()
{
    try {   
        _robot.log_info("in DoImageArea4");

        if (_robot.DoImageArea4()) { 
            _robot.log_info("About to return success - ImageArea4 task or action needed now!");
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in DoImageArea4::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList DoImageArea4::providedPorts()
{
    return { };
}

// Can just use ImageArea action node for each subtree
/*ImageArea3::ImageArea3(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w, CoveragePath& cp)
    // : StatefulActionNode(name, config), _robot(r), _world(w), _coverage_path_planner(cp) {
    : StatefulActionNode(name, config), _robot(r), _world(w), _coverage_path_planner(cp) {

    }

NodeStatus ImageArea3::onStart()
{
    try {
        std::cout << "Robot " << _robot.getID() << " doing ImageArea..." << std::endl;
        std::string strt = "Starting ImageArea3 for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);
        _robot.log_info("Starting ImageArea3...");

        _world.log_info("Task progress after single update in action start function:");
        _world.logCurrentTeamTaskProgress();

        std::vector<int> task_path = _robot.getPath(); 
        int current_task_id = task_path[0];
        TaskInfo& current_task = _world.getTaskInfo(current_task_id);
        std::unordered_map<std::string,int> area = current_task.area;

        std::string bla = "Area for ImageArea task - xmin: " + std::to_string(area["xmin"]) + 
                          ", xmax: " + std::to_string(area["xmax"]) + 
                          ", ymin: " + std::to_string(area["ymin"]) + 
                          ", ymax: " + std::to_string(area["ymax"]);
        _robot.log_info(bla);
        _world.log_info(bla);

        Pose2D current_pose = _robot.getPose();

        // Init vector of waypoints, the plan
        _waypoints = _coverage_path_planner.plan(current_pose, area,
                                                _world.getX(), _world.getY());
        
        if (_waypoints.empty()) {
            std::cout << "No coverage path found" << std::endl;
            return NodeStatus::FAILURE;
        }
        
        _current_waypoint_index = 0;
        std::string borp = "Planned coverage path with " + std::to_string(_waypoints.size()) + " waypoints";
        _robot.log_info(borp);


        _robot.log_info("Leaving onStart of ImageArea3");
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus ImageArea3::onRunning()
{
    try {
        _robot.log_info("in onRunning for ImageArea3...");

        std::vector<int> path = _robot.getPath();
        int current_task_id = path[0];
        
        // Check if this is a helper responding to a failure
        if (_world.isSubtaskID(current_task_id)) {
            _robot.log_info("Helper mode - current_task_id is subtask: " + std::to_string(current_task_id));
            _robot.setHelperMode(true);
        }
        
        // Get action ID for fault injection
        TaskInfo& task = _world.getTaskInfo(current_task_id);
        int local_current_task_id;
        if (_robot.inHelperMode()) {
            local_current_task_id = current_task_id;
        } else {
            local_current_task_id = task.subtasks[0];
        }
        
        std::string lctid = "local_current_task_id for fault injection: " + std::to_string(local_current_task_id);
        _robot.log_info(lctid);

        bool fault_flag = _world.getFaultInjectionFlag(local_current_task_id);
        _robot.log_info("fault_flag: " + std::to_string(fault_flag));
        
        if (fault_flag && !_robot.inHelperMode()) {
            // Main robot encounters fault - wait for helper
            _robot.log_info("Fault encountered - waiting for helper");
            return NodeStatus::RUNNING;
        }
        
        if (_robot.inHelperMode()) {
            // Helper clears the fault so original can proceed
            _robot.log_info("Helper clearing fault for original robot");
            _world.updateFaultInjectionTracker(local_current_task_id, 0);
            _robot.setHelperMode(false);
            // Helper ALSO does the coverage task (both robots work together)
        }
        
        // Both robots do coverage (original + helper working together)
        if (_current_waypoint_index < _waypoints.size()) {
            Pose2D waypoint = _waypoints[_current_waypoint_index];
            
            if(_robot.foundObstacle(waypoint)) {
                _waypoints.clear(); 
                std::unordered_map<std::string,int> new_area = _robot.calculateRemainingAreaToCover(task.area);
                _waypoints = _coverage_path_planner.plan(_robot.getPose(), new_area, _world.getX(), _world.getY());
                if (_waypoints.empty()) {
                    return NodeStatus::FAILURE;
                }
                _current_waypoint_index = 0;
                return NodeStatus::RUNNING;
            }
            
            _robot.move(waypoint);
            _current_waypoint_index++;
            return NodeStatus::RUNNING;
        }

        // Coverage complete
        taskSuccessProcessing(_world, _robot, current_task_id, local_current_task_id);
        _robot.log_info("Returning success for completion of ImageArea3");
        return NodeStatus::SUCCESS;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void ImageArea3::onHalted()
{
    std::cout << "Test ImageArea halted." << std::endl;
}

PortsList ImageArea3::providedPorts()
{
    return {};
}

ImageArea4::ImageArea4(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w, CoveragePath& cp)
    // : StatefulActionNode(name, config), _robot(r), _world(w), _coverage_path_planner(cp) {
    : StatefulActionNode(name, config), _robot(r), _world(w), _coverage_path_planner(cp) {

    }

NodeStatus ImageArea4::onStart()
{
    try {
        std::cout << "Robot " << _robot.getID() << " doing ImageArea..." << std::endl;
        std::string strt = "Starting ImageArea4 for robot " + std::to_string(_robot.getID()) + "...";
        _world.log_info(strt);
        _robot.log_info("Starting ImageArea4...");

        std::vector<int> path = _robot.getPath();
        _robot.log_info("Path:");
        utils::log1DVector(path, _robot);

        _world.log_info("Task progress after single update in action start function:");
        _world.logCurrentTeamTaskProgress();

        std::vector<int> task_path = _robot.getPath(); 
        int current_task_id = task_path[0];
        TaskInfo& current_task = _world.getTaskInfo(current_task_id);
        std::unordered_map<std::string,int> area = current_task.area;

        std::string bla = "Area for ImageArea task - xmin: " + std::to_string(area["xmin"]) + 
                          ", xmax: " + std::to_string(area["xmax"]) + 
                          ", ymin: " + std::to_string(area["ymin"]) + 
                          ", ymax: " + std::to_string(area["ymax"]);
        _robot.log_info(bla);
        _world.log_info(bla);

        Pose2D current_pose = _robot.getPose();

        // Init vector of waypoints, the plan
        _waypoints = _coverage_path_planner.plan(current_pose, area,
                                                _world.getX(), _world.getY());
        
        if (_waypoints.empty()) {
            std::cout << "No coverage path found" << std::endl;
            return NodeStatus::FAILURE;
        }
        
        _current_waypoint_index = 0;
        std::string borp = "Planned coverage path with " + std::to_string(_waypoints.size()) + " waypoints";
        _robot.log_info(borp);

        _robot.log_info("Leaving onStart of ImageArea4");
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus ImageArea4::onRunning()
{
    try {
        _robot.log_info("in onRunning for ImageArea4...");

        std::vector<int> path = _robot.getPath();
        int current_task_id = path[0];
        
        // Check if this is a helper responding to a failure
        if (_world.isSubtaskID(current_task_id)) {
            _robot.log_info("Helper mode - current_task_id is subtask: " + std::to_string(current_task_id));
            _robot.setHelperMode(true);
        }
        
        // Get action ID for fault injection
        TaskInfo& task = _world.getTaskInfo(current_task_id);
        int local_current_task_id;
        if (_robot.inHelperMode()) {
            local_current_task_id = current_task_id;
        } else {
            local_current_task_id = task.subtasks[0];
        }
        
        std::string lctid = "local_current_task_id for fault injection: " + std::to_string(local_current_task_id);
        _robot.log_info(lctid);

        bool fault_flag = _world.getFaultInjectionFlag(local_current_task_id);
        _robot.log_info("fault_flag: " + std::to_string(fault_flag));
        
        if (fault_flag && !_robot.inHelperMode()) {
            // Main robot encounters fault - wait for helper
            _robot.log_info("Fault encountered - waiting for helper");
            return NodeStatus::RUNNING;
        }
        
        if (_robot.inHelperMode()) {
            // Helper clears the fault so original can proceed
            _robot.log_info("Helper clearing fault for original robot");
            _world.updateFaultInjectionTracker(local_current_task_id, 0);
            _robot.setHelperMode(false);
            // Helper ALSO does the coverage task (both robots work together)
        }
        
        // Both robots do coverage (original + helper working together)
        if (_current_waypoint_index < _waypoints.size()) {
            Pose2D waypoint = _waypoints[_current_waypoint_index];
            
            if(_robot.foundObstacle(waypoint)) {
                _waypoints.clear(); 
                std::unordered_map<std::string,int> new_area = _robot.calculateRemainingAreaToCover(task.area);
                _waypoints = _coverage_path_planner.plan(_robot.getPose(), new_area, _world.getX(), _world.getY());
                if (_waypoints.empty()) {
                    return NodeStatus::FAILURE;
                }
                _current_waypoint_index = 0;
                return NodeStatus::RUNNING;
            }
            
            _robot.move(waypoint);
            _current_waypoint_index++;
            return NodeStatus::RUNNING;
        }

        // Coverage complete
        taskSuccessProcessing(_world, _robot, current_task_id, local_current_task_id);
        _robot.log_info("Returning success for completion of ImageArea4");
        return NodeStatus::SUCCESS;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void ImageArea4::onHalted()
{
    std::cout << "Test ImageArea halted." << std::endl;
}

PortsList ImageArea4::providedPorts()
{
    return {};
}*/

IsIdle::IsIdle(const std::string& name, const NodeConfig& config, World& world, Robot& robot)
    : ConditionNode(name, config), _world(world), _robot(robot) {}       

NodeStatus IsIdle::tick()
{
    try {

        std::string borp = "In tick for IsIdle";
        _robot.log_info(borp);

        if (_robot.IsIdle()) {
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in IsIdle::tick: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

PortsList IsIdle::providedPorts()
{
    return {};
}

GoHome::GoHome(const std::string& name, const NodeConfig& config,
                                       Robot& r, World& w, ShortestPath& sp)
    : StatefulActionNode(name, config), _robot(r), _world(w), _shortest_path_planner(sp) {
    }

NodeStatus GoHome::onStart()
{
    try {

        std::string yeup = "In onStart for GoHome";
        _robot.log_info(yeup);

        // Above lines now in this function
        Pose2D current_pose = _robot.getPose();
        Pose2D goal_pose = _world.getHomePose();

        // Init vector of waypoints, the plan
        _waypoints = _shortest_path_planner.plan(current_pose, goal_pose,
                                                _world.getX(), _world.getY());
        
        if (_waypoints.empty()) {
            std::cout << "No path home found" << std::endl;
            return NodeStatus::FAILURE;
        }
        
        _current_waypoint_index = 0;
        std::string borp = "Planned path home with " + std::to_string(_waypoints.size()) + " waypoints";
        _robot.log_info(borp);

        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

NodeStatus GoHome::onRunning()
{
    try {
        // Traverse waypoints to home position
        if (_current_waypoint_index >= _waypoints.size()) {
            std::cout << "Robot " << _robot.getID() << " reached home position" << std::endl;
            return NodeStatus::SUCCESS;
        }
        
        Pose2D waypoint = _waypoints[_current_waypoint_index];
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
       
        // Check if next waypoint is traversable
        if(_robot.foundObstacle(waypoint)) {
            // Replan around obstacle
            _waypoints.clear(); 
            Pose2D current_pose = _robot.getPose();
            Pose2D goal_pose = _world.getHomePose();
            _waypoints = _shortest_path_planner.plan(current_pose, goal_pose, _world.getX(), _world.getY());

            if (_waypoints.empty()) {
                std::cout << "No path to home found" << std::endl;
                return NodeStatus::FAILURE;
            }

            _current_waypoint_index = 0;
            return NodeStatus::RUNNING;
        }
        
        _robot.move(waypoint);
        _current_waypoint_index++;
        return NodeStatus::RUNNING;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in GoToHome: " << e.what() << std::endl;
        return NodeStatus::FAILURE;
    }
}

void GoHome::onHalted()
{
    std::cout << "Test GoHome halted." << std::endl;
}

PortsList GoHome::providedPorts()
{
    return {};
}
