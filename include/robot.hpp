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
    Planner* planner;
    ShortestPath* shortest_path;
    CoveragePath* coverage_path;
    World* world; // Point to single,shared world instance
    JSONParser* parser;
    std::vector<Msg> message_queue;
    cv::Scalar color;
    double battery_level;

    std::vector<int> doable_task_ids; // doable local tasks by id

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

    // The follow are the previous bundle, path, winners and winning_bids, tracked to check for convergence between iterations
    std::vector<int> prev_bundle;
    std::vector<int> prev_path;
    std::unordered_map<int, int> prev_winners; // CBBA
    std::unordered_map<int, double> prev_winning_bids; // CBBA
    std::vector<std::vector<double>> prev_winning_bids_matrix; // CBGA
    int num_converged_iterations;

    int cbba_rounds;
    int cbga_rounds;
    std::vector<std::pair<int,double>> last_pings; // Pings when last check for new info occurred
    double time_of_last_self_update; // Timestamp at which robot updated its own bundle/path/winners/winning bids (to be passed with id in pings)

    double comms_timeout_threshold;

    bool at_consensus;

    std::ofstream robot_log; // Init file for each robot to log in



public:
    Robot(Planner* planner, ShortestPath* shortest_path, CoveragePath* coverage_path, World* world, JSONParser* parser, const Pose2D& initial_pose, int robot_id, std::string robot_type, cv::Scalar color); // Is this right with planners?

    int getID() const { return id; }
    std::string getType() const { return type; }
    int getCurrentTaskID() const { return task_id; }
    int getX() const { return pose.x; }
    int getY() const { return pose.y; }
    Pose2D getPose() const { return pose; }
    //Pose2D getGoalPose() const { return goal; }
    cv::Scalar getColor() const {return color; }
    std::vector<int>& getDoableTaskIDs() { return doable_task_ids; }
    std::vector<int>& getBundle() { return bundle; }
    std::vector<int>& getPreviousBundle() { return prev_bundle; }
    std::vector<int>& getPath() { return path; }
    std::vector<int>& getPreviousPath() { return prev_path; }
    //std::vector<double>& getScores() { return scores; }
    //std::unordered_map<int,double> initBids();  
    std::map<int,double> initBids();  
    void resetBids();
    std::unordered_map<int,int> initWinners();
    std::unordered_map<int,double> initWinningBids(); // CBBA
    std::vector<std::vector<double>> initWinningBidsMatrix(); // CBGA
    std::unordered_map<int,double> initTimestamps();
    std::unordered_map<int,Pose2D> initLocations(); // CBGA
    std::pair<int, Pose2D> getMostUpToDateNeighborInfo(int id_k);
    //std::unordered_map<int, double>& getBids() { return bids; }
    std::map<int, double>& getBids() { return bids; }
    std::unordered_map<int, int>& getWinners() { return winners; }
    std::unordered_map<int, double>& getWinningBids() { return winning_bids; }
    std::vector<std::vector<double>>& getWinningBidsMatrix() { return winning_bids_matrix; }
    std::unordered_map<int, double>& getTimestamps() { return timestamps; }
    std::unordered_map<int, Pose2D>& getLocations() { return locations; }
    std::unordered_map<int, int>& getPreviousWinners() { return prev_winners; }
    std::unordered_map<int, double>& getPreviouswWinningBids() { return prev_winning_bids; }
    //std::vector<std::vector<int>>& getFeasibleTasks() { return feasible_tasks; }
    void init(Pose2D initial_pose);
    //void printTasksVector();
    void move(Pose2D waypoint);
    //void printWorldPingIDTracker(std::unordered_map<int, std::vector<int>>& world_ping_id_tracker);
    void printWorldPingTracker(std::unordered_map<int, std::vector<std::pair<int,double>>>& world_ping_tracker);
    std::vector<Msg>& getMessageQueue() { return message_queue; }
    //void printMessageQueue(std::vector<Msg>&  message_queue);
    //void printMessage(Msg msg);
    void updateRobotMessageQueue(Msg msg);
    void receiveMessages();
    double getCurrentTime();
    //void receivePings();
    void updateTimestamps();
    void updateLocations(); // CBGA
    bool needRegroup();
    double getBatteryLevel() const { return battery_level; }
    void updateBatteryLevel(double drain_percent);
    bool batteryLow();
    //WinningBids& getWinningBids() { return winning_bids; }
    //WinningAgentIndices& getWinningAgentIndices() { return winning_agent_indices; }
    std::string generateLogFilename();
    void log_info(std::string log_msg);
    bool checkIfNewInfoAvailable();
    void countConvergedIterations();
    int getConvergenceCount() { return num_converged_iterations; }
    void updateBeliefs();
    int& getNumCBBARounds() {return cbba_rounds; }
    int& getNumCBGARounds() {return cbga_rounds; }
    void resetConvergenceCount();
    void resetNumCBBARounds();
    void resetNumCBGARounds();
    void updateLastSelfUpdateTime(double timestamp);
    bool foundBeliefUpdate();
    void clearStalePings(); // Get rid of pings in ping tracker that are older than timeout threshold - sender robots are offline or out of range
    bool ExploreA();
    bool ExploreB();
    bool ExploreC();
    bool ExploreD();
    std::pair<int,int> getNextStartLocation(); // Location robot should go to start the current (first) task in the path

    void removeCompletedTaskFromPath();

    bool getAtConsensus() const { return at_consensus; }
    void setAtConsensus(bool value) { at_consensus = value; }

    std::unordered_map<std::string, int> initTaskGroupFullnessMap();
    std::vector<int> getAssignedAgents(int task_id);
    std::unordered_map<std::string, int> getTaskGroupFullnessbyType(int task_id);

    std::unordered_map<std::string, std::vector<int>> initTaskSubGroupMap();
    std::unordered_map<std::string, std::vector<int>> trackAssignedRobotsbySubGroup(int task_id);

    //void resurfaceToCharge();

    // Will need to add a function/functions that deal with battery level checking and eval wrt tasks
    // Like how do we know, given a task, how much battery it could take, do we translate distance to battery?
    // If we are given a four corner defined area, then do we calculate distance to traverse the entire thing, and translate that to battery?
};

#endif
 