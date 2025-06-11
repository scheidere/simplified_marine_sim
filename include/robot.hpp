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
    Pose2D goal;
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
    /*so bundle will have assigned task IDs in order of time added, 
    scores will have the score for each assigned task in that same order, 
    and the path will have the same IDs but in the order they should be attempted*/
    std::vector<int> bundle; // Assigned tasks for this agent
    std::vector<int> path;  // Task execution order for this agent's assigned tasks
    //std::vector<double> scores; // Scores for this agent's assigned tasks
    //The following three maps are indexed (unordered map keys) by task IDs
    std::map<int, double> bids; // Values are bids // changed to map from unordered for tie priority to lower int task IDs
    std::unordered_map<int, int> winners; // Values are agent IDs
    std::unordered_map<int, double> winning_bids; // Values are bids obviously
    std::unordered_map<int, double> timestamps; // Time of last info update from each of other agents

    // The follow are the previous bundle, path, winners and winning_bids, tracked to check for convergence between iterations
    std::vector<int> prev_bundle;
    std::vector<int> prev_path;
    std::unordered_map<int, int> prev_winners;
    std::unordered_map<int, double> prev_winning_bids;
    int num_converged_iterations;

    int cbba_rounds;
    std::vector<int> last_pings; // Pings when last check for new info occurred

    std::ofstream robot_log; // Init file for each robot to log in



public:
    Robot(Planner* planner, ShortestPath* shortest_path, CoveragePath* coverage_path, World* world, JSONParser* parser, const Pose2D& initial_pose, const Pose2D& goal_pose, int robot_id, std::string robot_type, cv::Scalar color); // Is this right with planners?

    int getID() const { return id; }
    std::string getType() const { return type; }
    int getCurrentTaskID() const { return task_id; }
    int getX() const { return pose.x; }
    int getY() const { return pose.y; }
    Pose2D getPose() const { return pose; }
    Pose2D getGoalPose() const { return goal; }
    cv::Scalar getColor() const {return color; }
    std::vector<int>& getDoableTaskIDs() { return doable_task_ids; }
    std::vector<int>& getBundle() { return bundle; }
    std::vector<int>& getPreviousBundle() { return prev_bundle; }
    std::vector<int>& getPath() { return path; }
    std::vector<int>& getPreviousPath() { return prev_path; }
    //std::vector<double>& getScores() { return scores; }
    //std::unordered_map<int,double> initBids();  
    std::map<int,double> initBids();  
    std::unordered_map<int,int> initWinners();
    std::unordered_map<int,double> initWinningBids();
    std::unordered_map<int,double> initTimestamps();
    //std::unordered_map<int, double>& getBids() { return bids; }
    std::map<int, double>& getBids() { return bids; }
    std::unordered_map<int, int>& getWinners() { return winners; }
    std::unordered_map<int, double>& getWinningBids() { return winning_bids; }
    std::unordered_map<int, double>& getTimestamps() { return timestamps; }
    std::unordered_map<int, int>& getPreviousWinners() { return prev_winners; }
    std::unordered_map<int, double>& getPreviouswWinningBids() { return prev_winning_bids; }
    //std::vector<std::vector<int>>& getFeasibleTasks() { return feasible_tasks; }
    void init(Pose2D initial_pose);
    //void printTasksVector();
    void move(Pose2D waypoint);
    void printWorldPingTracker(std::unordered_map<int, std::vector<int>>& world_ping_tracker);
    std::vector<Msg>& getMessageQueue() { return message_queue; }
    //void printMessageQueue(std::vector<Msg>&  message_queue);
    //void printMessage(Msg msg);
    void updateRobotMessageQueue(Msg msg);
    void receiveMessages();
    double getMessageReceptionTime();
    void receivePings();
    void updateTimestamps();
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

    //void resurfaceToCharge();

    // Will need to add a function/functions that deal with battery level checking and eval wrt tasks
    // Like how do we know, given a task, how much battery it could take, do we translate distance to battery?
    // If we are given a four corner defined area, then do we calculate distance to traverse the entire thing, and translate that to battery?
};

#endif
 