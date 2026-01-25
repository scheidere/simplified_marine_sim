#ifndef WORLD_H
#define WORLD_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "robot.hpp"
#include "message.hpp"
#include "CBBA.hpp"
#include "structs.hpp"
#include <unordered_map>
#include <chrono>


struct Task;
class Robot;
struct Msg;

class World {
protected:
    int X; int Y;
    Distance* distance;
    SensorModel* sensor_model;
    JSONParser* parser;
    cv::Mat image; // For movement
    cv::Mat background_image;  // Permanent background with obstacles
    // std::unordered_map<int, Robot*> robot_tracker; // id, instance
    std::map<int, Robot*> robot_tracker; // id, instance (updated to map from unordered for consistency is messaging runs)
    mutable std::mutex world_mutex; // mutex for MR access to robot tracker (and downstream), mutable for access of consts
    double comms_range;
    std::unordered_map<int,std::vector<Msg>> message_tracker; // receiving robot ID, vector of message structs
    //std::unordered_map<int, std::vector<int>> ping_id_tracker; // receiving robot ID, vector of pinging robot IDs (if in this, are newly in range and haven't messaged yet)
    // std::unordered_map<int, std::vector<std::pair<int,double>>> ping_tracker; // receiving robot ID, vector of pairs: pinging robot ID (sender ID), timestamp of sender's last self-update of bundle/path
    std::unordered_map<int, std::vector<std::tuple<int, double, bool>>> ping_tracker; // receiving robot ID, vector of pairs: pinging robot ID (sender ID), timestamp of sender's last self-update of bundle/path
    std::vector<Pose2D> areaACoords; std::vector<Pose2D> areaBCoords; std::vector<Pose2D> areaCCoords; std::vector<Pose2D> areaDCoords;
    //std::vector<Task> allTasks; // struct stuff, ignoring for now

    int num_agents; // Total number of robots in the world
    std::unordered_map<int,AgentInfo> all_agents_info;
    int num_tasks;  // Number of local tasks that each agent might be able to do depending on type
    std::unordered_map<int,TaskInfo> all_tasks_info;
    std::unordered_map<int,TaskInfo> all_subtasks_info; // not initially available for assignment, each one triggered to be available by a certain fault
    //std::vector<int> agent_indices; // is this really needed
    std::vector<std::string> agent_types;
    std::vector<std::string> task_types;
    //std::unordered_map<std::string, std::vector<int>> all_agent_capabilities; // Denotes which agents can do which tasks by agent type (0: can't; 1: can by self; 2: can co-op; 3: TBD)
    std::unordered_map<std::string, std::unordered_map<std::string,bool>> all_agent_capabilities;

    std::vector<int> agent_ids;
    std::vector<int> subtask_ids;

    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 0, 255),
        cv::Scalar(0, 255, 255),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 0, 255),
        cv::Scalar(255, 255, 255)
    };

    std::chrono::steady_clock::time_point start_time;

    std::unordered_map<int,std::vector<int>> task_completion_log; // robot id : vector of task ids it completed
    std::vector<std::pair<int,int>> task_completion_order; // also for logging but by pair, robot id and task id as completed (more info than above log)

    // Want to check if each robot has heard from all robots at consensus when they all start in comms and reach consensus the first time with no movement
    std::unordered_map<int,std::vector<int>> messaging_log; // robot id : vector of sender ids (cumulative so can have duplicates, just one for each message received)

    std::ofstream world_log; // Init file for world to log to

    double cumulative_distance_traveled; // by the team
    double cumulative_reward_achieved; // by the team

    std::unordered_map<int, bool> fault_injection_tracker; // task id keys, and bool denoting if task fails when attempted by main assigned robot

    std::vector<std::vector<cv::Point>> obstacles; // Main obstacles list

    double signal_path_loss_factor;
    double decay_rate;


public:
    World(int X, int Y, Distance* distance, SensorModel* sensor_model, JSONParser* parser, double comms_range);

    int getX() const { return X; }
    int getY() const { return Y; }

    std::vector<int> getAgentIDs();
    std::vector<int> getSubtaskIDs();

    void logListofTaskIDs(std::unordered_map<int,TaskInfo> task_list);

    std::string generateLogFilename();

    void log_info(std::string log_msg);

    double& getCumulativeDistance() { return cumulative_distance_traveled; }

    double& getCumulativeReward() { return cumulative_reward_achieved; }

    void logCurrentTeamAssignment();

    void logCurrentTeamTaskProgress();

    std::chrono::steady_clock::time_point getStartTime() const;

    double getElapsedTime() const;

    cv::Mat& getImage();

    cv::Mat init();

    void initAllRobots();

    //std::unordered_map<int, Robot*>& getRobotTracker(); //{ return robot_tracker; }
    std::map<int, Robot*>& getRobotTracker(); // switched to map for order consistency in message broadcasting

    std::unordered_map<int,std::vector<Msg>>& getMessageTracker(); //{return message_tracker; }

    std::unordered_map<int,std::vector<Msg>>& getMessageTrackerUnsafe();

    // std::unordered_map<int, std::vector<std::pair<int,double>>>& getPingTracker();
    std::unordered_map<int, std::vector<std::tuple<int,double,bool>>>& getPingTracker();

    int getNumAgents() {return num_agents; }

    int getNumLocalTasks() {return num_tasks; }

    void clear(Pose2D pose);

    void plot();

    void trackRobot(Robot* robot);

    void printTrackedRobots();

    bool inComms(int id1, int id2);

    std::mutex& getWorldMutex() { return world_mutex; }

    void printMessageTracker();

    bool isCollision(int x, int y);

    void defineQuadrants();

    std::vector<Pose2D> getQuadrantCenters();

    void printMessage(Msg msg);

    std::vector<std::string> getAgentTypes() { return agent_types; }

    std::unordered_map<int,AgentInfo> initAllAgentsInfo();

    std::unordered_map<int,AgentInfo>& getAllAgentsInfo() { return all_agents_info; }

    AgentInfo& getAgentInfo(int agent_id);

    std::string& getAgentType(int agent_id);

    int getGroupSize(std::unordered_map<std::string, int> group_info);

    std::unordered_map<int,TaskInfo> initAllTasksInfo();

    std::unordered_map<int, TaskInfo> initAllSubtasksInfo();

    std::unordered_map<int,TaskInfo>& getAllTasksInfo() { return all_tasks_info; }

    std::unordered_map<int,TaskInfo>& getAllSubtasksInfo() { return all_subtasks_info; }

    TaskInfo& getTaskInfo(int task_id);
    //TaskInfo& getTaskInfoUnsafe(int task_id); // mutex scope test, in greedy

    TaskInfo& getSubtaskInfo(int task_id);

    int& getTaskGroupSize(int task_id);

    std::unordered_map<std::string, int>& getTaskGroupInfo(int task_id);

    double& getTaskReward(int task_id);
    //double& getTaskRewardUnsafe(int task_id); // mutex scope test, in greedy

    //std::pair<int,int> getTaskLocation(int task_id, Robot* robot); // robot passed in for TESTING ONLY
    std::pair<int,int> getTaskLocation(int task_id);
    //std::pair<int,int> getTaskLocationUnsafe(int task_id); // mutex scope test, in greedy

    std::pair<int,int> getTaskLocationFromArea(std::unordered_map<std::string, int>& area);

    //std::unordered_map<std::string,std::vector<int>> getAllCapabilities(); // { return all_agent_capabilities; }
    std::unordered_map<std::string, std::unordered_map<std::string,bool>> getAllCapabilities();

    std::vector<int> getRobotSubtaskCapabilities(Robot* robot);

    std::vector<int> getRobotCapabilities(Robot* robot);

    void initMessageTracker();

    void initPingTracker();

    std::vector<int> getNeighborsInComms(int robot_id_i);

    void logNeighbors();

    double getMaxNeighborTimestamp(int robot_id_i, int out_of_range_robot_id_k);

    bool hasTaskInfo(int task_id);

    void debugTaskAccess(int task_id, Robot& robot); // called before greedy class in BT greedy node so timing is consistent

    bool fullGroupPresent(int current_task_id);

    std::unordered_map<int,std::vector<int>>& getTaskCompletionLog() { return task_completion_log; }

    void updateTaskCompletionLog(int robot_id, int completed_task_id);

    void logTaskCompletion();

    void updateCumulativeReward(double reward);

    void updateCumulativeDistance();

    void updateMessagingLog(int robot_id, int msg_id);

    void logMessagingLog();

    int getPrerequisiteFailureThreshold(std::string subtask_name);

    std::unordered_map<int,bool> initFaultInjectionTracker();

    bool getFaultInjectionFlag(int task_id);

    void updateFaultInjectionTracker(int task_id, bool fail_flag); // fail flag 1 if injection defaults failure, 0 when helper has helped do it successfully

    int getSubtaskID(std::string name);

    bool isSubtaskID(int task_id);

    bool isLastSubtask(int current_task_id, int local_current_task_id);

    void initializeBackground();

    void addObstacle(std::vector<cv::Point> polygon); // to list, not plotting yet

    void getObstacles();

    bool isObstacle(int x, int y);

    double initSignalPathLossFactor();

    double getSignalStrength(); // not used now

    double initDecayRate();
};

#endif
