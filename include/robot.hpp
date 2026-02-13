#ifndef ROBOT_H
#define ROBOT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "planners.hpp"
#include "world.hpp"
//#include "message.hpp"
#include "structs.hpp"
#include "CBBA.hpp"
#include "parser.hpp"


struct Msg;
//struct Task;
//struct Bundle;

class Robot {
private:
    int id;
    std::string type;
    int task_id;
    Pose2D pose;
    World* world; // Point to single,shared world instance
    JSONParser* parser;
    std::vector<Msg> message_queue;
    cv::Scalar color;
    double battery_level;

    std::vector<int> doable_task_ids; // doable local tasks by id
    std::vector<int> doable_subtask_ids; // doable local subtasks by id

    int max_depth;

    // We initialize bundle if needed at the beginning of CBBA's BuildBundle() (where init path and scores?)
    std::vector<int> bundle; // Assigned tasks (by ID) for this agent, in order of time added
    std::vector<int> path;  // Task execution order for this agent's assigned tasks (by ID)
    std::map<int, double> bids; // Values are bids // changed to map from unordered map for tie priority to lower int task IDs
    //The following three maps are indexed (unordered map keys) by task IDs
    std::unordered_map<int, int> winners; // Values are agent IDs (vector for CBBA)
    std::unordered_map<int, double> winning_bids; // Values are bids obviously (vector for CBBA)
    std::vector<std::vector<double>> winning_bids_matrix; // Values are bids, rows denote task j by idx r = j-1, columns denote agent k by idx c = k-1 (matrix for CBGA, combines winners/winning bids info)
    std::unordered_map<int, double> timestamps; // Time of last info update from each of other agents (does not include own id, because doesn't need to message self)
    std::unordered_map<int,Pose2D> locations; // Last known location of all robots including self (before comms only knows own location)

    std::unordered_map<int,int> task_progress; // task id: 0 if not started, 1 if started // might add 2 for complete at some point

    // The follow are the previous bundle, path, winners and winning_bids, tracked to check for convergence between iterations
    std::vector<int> prev_bundle;
    std::vector<int> prev_path;
    std::unordered_map<int, int> prev_winners; // CBBA
    std::unordered_map<int, double> prev_winning_bids; // CBBA
    std::vector<std::vector<double>> prev_winning_bids_matrix; // CBGA
    int num_converged_iterations;

    int cbba_rounds;
    int cbga_rounds;
    // std::vector<std::pair<int,double>> last_pings; // Pings when last check for new info occurred
    std::vector<std::tuple<int,double,bool>> last_pings; // Pings when last check for new info occurred
    double time_of_last_self_update; // Timestamp at which robot updated its own bundle/path/winners/winning bids (to be passed with id in pings)

    double comms_timeout_threshold;

    bool at_consensus;

    std::ofstream robot_log; // Init file for each robot to log in

    std::chrono::high_resolution_clock::time_point task_alloc_start_time; // will be used by greedy OR CBBA, or CBGA, never at the same time bc different BTs

    double cumulative_distance;

    std::unordered_map<int, std::unordered_map<int, bool>> subtask_failures; // Values True when robot k fails task j 
    std::unordered_map<int, std::unordered_map<int, bool>> prev_subtask_failures; // Used for updating doable tasks when subtasks can newly be considered to aid failing neighboring robots

    bool new_self_subtask_failure;
    bool helper_mode;
    bool reattempt_failing_action; // received from neighbors who may have helped
    bool help_given; // self is helper and has succeeded (set to true in helper mode subtask success)

    // Obstacles list, discovered online, sorted by robot type (type: obstacles found that block that type)
    std::unordered_map<std::string,std::vector<std::vector<cv::Point>>> discovered_obstacles;

    std::unordered_map<int, double> task_scores;

    std::mutex failure_flag_mutex;
    mutable std::mutex subtask_failures_mutex;
    mutable std::mutex task_progress_mutex;

    int consecutive_failure_count;
    int consecutive_waiting_count;

public:
    Robot(World* world, JSONParser* parser, const Pose2D& initial_pose, int robot_id, std::string robot_type, cv::Scalar color); // Is this right with planners?

    int getID() const { return id; }
    std::string getType() const { return type; }
    int getCurrentTaskID() const { return task_id; }
    int getX() const { return pose.x; }
    int getY() const { return pose.y; }
    Pose2D getPose() const { return pose; }
    cv::Scalar getColor() const {return color; }
    std::vector<int>& getDoableTaskIDs() { return doable_task_ids; }
    std::vector<int>& getBundle() { return bundle; }
    std::vector<int>& getPreviousBundle() { return prev_bundle; }
    std::vector<int>& getPath() { return path; }
    std::vector<int>& getPreviousPath() { return prev_path; } 
    //std::unordered_map<int, std::unordered_map<int, bool>>& getSubtaskFailures() { return subtask_failures; } // Used for messaging, moved to .cpp for mutex addition
    std::unordered_map<int, std::unordered_map<int, bool>>& getSubtaskFailures(); // copy instead of reference for use with mutex
    bool getSubtaskFailure(int subtask_id, int agent_id);
    void setSubtaskFailure(int subtask_id, int agent_id, bool failed);
    void updateSubtaskFailuresPerNeighbors(); // Traverse received messages and update subtask failures tracker, deferring to 1's (maining fails), except for self
    void updateSubtaskFailuresPerSelf(std::unordered_map<int,bool> new_self_subtask_failures);
    // void updateWinningBidsMatrixPostFailure();
    void updateWinningBidsMatrixPostResolution();
    void testNewNeighborSubtaskFailures();
    // std::pair<bool,std::vector<int>> newSelfSubtaskFailures();
    // void testNewSelfSubtaskFailures();
    std::pair<bool,std::vector<int>> newNeighborSubtaskFailures();
    void updateDoableTasks();
    void testSubtaskFailuresUpdater();
    std::map<int,double> initBids();  
    void resetBids();
    std::unordered_map<int,int> initWinners();
    std::unordered_map<int,double> initWinningBids(); // CBBA
    std::vector<std::vector<double>> initWinningBidsMatrix(); // CBGA
    std::unordered_map<int,double> initTimestamps();
    std::unordered_map<int,Pose2D> initLocations(); // CBGA
    std::unordered_map<int,int> initTaskProgress(); // CBGA and CBBA (both have taskAlreadyStarted calls now)
    std::unordered_map<int, std::unordered_map<int, bool>> initSubtaskFailures();
    std::pair<int, Pose2D> getMostUpToDateNeighborInfo(int id_k);
    std::map<int, double>& getBids() { return bids; }
    std::unordered_map<int, int>& getWinners() { return winners; }
    std::unordered_map<int, double>& getWinningBids() { return winning_bids; }
    std::vector<std::vector<double>>& getWinningBidsMatrix() { return winning_bids_matrix; }
    std::unordered_map<int, double>& getTimestamps() { return timestamps; }
    // std::unordered_map<int, int>& getTaskProgress() { return task_progress; }
    std::unordered_map<int, int>& getTaskProgress();
    std::unordered_map<int, Pose2D>& getLocations() { return locations; }
    std::unordered_map<int, int>& getPreviousWinners() { return prev_winners; }
    std::unordered_map<int, double>& getPreviouswWinningBids() { return prev_winning_bids; }
    void init(Pose2D initial_pose);
    void saveNewFoundObstacle(std::vector<cv::Point> discovered_obstacle);
    void move(Pose2D waypoint);
    // void printWorldPingTracker(std::unordered_map<int, std::vector<std::pair<int,double>>>& world_ping_tracker);
    void printWorldPingTracker(std::unordered_map<int, std::vector<std::tuple<int,double,bool>>>& world_ping_tracker);
    std::vector<Msg>& getMessageQueue() { return message_queue; }
    //void printMessageQueue(std::vector<Msg>&  message_queue);
    //void printMessage(Msg msg);
    void updateRobotMessageQueue(Msg msg);
    void receiveMessages();
    double getCurrentTime();
    void updateTimestamps();
    void updateLocations(); // CBGA
    bool needRegroup();
    double getBatteryLevel() const { return battery_level; }
    void updateBatteryLevel(double drain_percent);
    bool batteryLow();
    std::string generateLogFilename();
    void log_info(std::string log_msg);
    bool checkIfNewInfoAvailable();
    void countConvergedIterations(bool do_cbga);
    int getConvergenceCount() { return num_converged_iterations; }
    void updateBeliefs(bool do_cbga);
    int& getNumCBBARounds() {return cbba_rounds; }
    int& getNumCBGARounds() {return cbga_rounds; }
    void resetConvergenceCount();
    void resetNumCBBARounds();
    void resetNumCBGARounds();
    void updateLastSelfUpdateTime(double timestamp);
    bool foundBeliefUpdate(bool do_cbga);
    void clearStalePings(); // Get rid of pings in ping tracker that are older than timeout threshold - sender robots are offline or out of range
    bool ExploreA();
    bool ExploreB();
    bool ExploreC();
    bool ExploreD();
    std::pair<int,int> getNextStartLocation(); // Location robot should go to start the current (first) task in the path

    void removeCompletedTaskFromPath();
    void removeCompletedTaskFromPathAndBundle();

    bool& getAtConsensus() { return at_consensus; }
    void setAtConsensus(bool value) { at_consensus = value; }

    std::unordered_map<std::string, int> initTaskGroupFullnessMap();
    std::vector<int> getAssignedAgents(int task_id);
    std::unordered_map<std::string, int> getTaskGroupFullnessbyType(int task_id);

    std::unordered_map<std::string, std::vector<int>> initTaskSubGroupMap();
    std::unordered_map<std::string, std::vector<int>> trackAssignedRobotsbySubGroup(int task_id);

    void setTaskAllocStartTime(const std::chrono::high_resolution_clock::time_point& time) { task_alloc_start_time = time; } 
    std::chrono::high_resolution_clock::time_point getTaskAllocStartTime() const { return task_alloc_start_time; }

    bool PathClearingNeeded();

    void updateSingleTaskProgress(int task_id, int started);
    void updateTaskProgressFromAssignment();
    void updateTaskProgress(); // Traverse received messages and update task progress, deferring to 1's
    bool taskAlreadyStarted(int task_id);

    // Testing splitting updateTimestamps() and calling directly in receiveMessages instead of after in comms BT node function
    void updateTimestampGivenDirectMessage(int id_k);
    void updateRemainingTimestampsIndirectly();

    double& getCumulativeDistance() { return cumulative_distance; }

    bool SampleCollectionNeeded();

    // std::unordered_map<int,int> initFailureThresholdsDict(TaskInfo& current_task_info);
    std::map<int,int> initFailureThresholdsDict(TaskInfo& current_task_info, bool current_task_is_main); 

    bool getCurrentTaskScope(TaskInfo& current_task_info);

    void testCounterSequence();

    std::pair<std::pair<int,bool>,std::map<int,int>> HandleFailures(std::unordered_map<int,bool> new_self_subtask_failures);

    bool TaskNeededNow();

    bool getNewSelfSubtaskFailureFlag() { return new_self_subtask_failure; }

    bool inHelperMode() { return helper_mode; }
    void setHelperMode(bool mode) { helper_mode = mode; }

    bool foundObstacle(Pose2D waypoint); // for call in BT nodes before robot.move(waypoint)

    bool isFoundObstacle(int x, int y); // for call in planner.plan

    Pose2D getCurrentGoalPose();

    std::unordered_map<std::string,std::vector<std::vector<cv::Point>>>& getDiscoveredObstacles() { return discovered_obstacles; } // Used for messaging

    void updateDiscoveredObstacles(std::unordered_map<std::string,std::vector<std::vector<cv::Point>>> neighbor_discovered_obstacles);

    bool polygonsAreEqual(const std::vector<cv::Point>& poly1, const std::vector<cv::Point>& poly2);

    void updateDiscoveredObstaclesPerNeighbors();

    void saveTaskScore(int task_id, double score); // for plotting results
    double getTaskScore(int task_id); // for plotting results

    // bool DoImageArea(); requires fancier action function to work in one subtree alone

    std::unordered_map<std::string,int> calculateRemainingAreaToCover(std::unordered_map<std::string,int>& original_area);

    bool IsIdle();
    bool AwayFromHome();
    bool IsFailingAlone();
    bool isBeingHelped();
    bool IsStuckWaiting(); // for co-op tasks
    bool IsHelper();
    void incrementWaitingCount();
    void resetWaitingCount();
    bool Regrouped();

    bool DoImageArea1();
    bool DoImageArea2();
    bool DoImageArea3();
    bool DoImageArea4();

    bool DoSampleCollection1();
    bool DoSampleCollection2();
    bool DoSampleCollection3();
    bool DoSampleCollection4();

    bool DoClearPath1();
    bool DoClearPath2();

    void setNewSelfSubtaskFailure(bool failure_new);

    int getTaskSubtaskProgress(int task_id);

    // Below was for communicating help was given/received for fault recovery (but counter sequence reattempting inherently is cleaner)
    // bool getReattemptFailingActionFlag() { return reattempt_failing_action; }
    // void setReattemptFailingActionFlag(bool new_reattempt_flag);

    // bool getHelpGivenFlag() { return reattempt_failing_action; }
    // void setHelpGivenFlag(bool new_reattempt_flag);

    //void resurfaceToCharge();

    // Will need to add a function/functions that deal with battery level checking and eval wrt tasks
    // Like how do we know, given a task, how much battery it could take, do we translate distance to battery?
    // If we are given a four corner defined area, then do we calculate distance to traverse the entire thing, and translate that to battery?
};

#endif
 