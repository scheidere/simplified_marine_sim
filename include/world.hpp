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
    cv::Mat image;
    std::unordered_map<int, Robot*> robot_tracker; // id, instance
    mutable std::mutex world_mutex; // mutex for MR access to robot tracker (and downstream), mutable for access of consts
    double comms_range;
    std::unordered_map<int,std::vector<Msg>> message_tracker; // receiving robot ID, vector of message structs
    std::unordered_map<int, std::vector<int>> ping_tracker; // receiving robot ID, vector of pinging robot IDs (if in this, are newly in range and haven't messaged yet)
    std::vector<Pose2D> areaACoords; std::vector<Pose2D> areaBCoords; std::vector<Pose2D> areaCCoords; std::vector<Pose2D> areaDCoords;
    //std::vector<Task> allTasks; // struct stuff, ignoring for now

    int num_agents; // Total number of robots in the world
    std::unordered_map<int,AgentInfo> all_agents_info;
    int num_tasks;  // Number of local tasks that each agent might be able to do depending on type
    std::unordered_map<int,TaskInfo> all_tasks_info;
    std::vector<int> agent_indices; // is this really needed
    std::vector<std::string> agent_types;
    std::vector<std::string> task_types;
    std::unordered_map<std::string, std::vector<int>> all_agent_capabilities; // Denotes which agents can do which tasks by agent type (0: can't; 1: can by self; 2: can co-op; 3: TBD)

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



public:
    World(int X, int Y, Distance* distance, SensorModel* sensor_model, JSONParser* parser, double comms_range);

    int getX() const { return X; }
    int getY() const { return Y; }

    std::chrono::steady_clock::time_point getStartTime() const;

    double getElapsedTime() const;

    cv::Mat& getImage();

    cv::Mat init();

    void initAllRobots();

    std::unordered_map<int, Robot*>& getRobotTracker(); //{ return robot_tracker; }

    std::unordered_map<int,std::vector<Msg>>& getMessageTracker(); //{return message_tracker; }

    std::unordered_map<int, std::vector<int>>& getPingTracker();

    int getNumAgents() {return num_agents; }

    int getNumLocalTasks() {return num_tasks; }

    //void initAllTasks();

    //std::vector<Task>& getAllTasks();

    //int getTaskIndex(Task task); // std::pair<Task, int>

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

    //std::vector<AgentInfo> getAgents(); //AgentInfo is a struct

    std::unordered_map<int,AgentInfo> initAllAgentsInfo();

    std::unordered_map<int,AgentInfo>& getAllAgentsInfo() { return all_agents_info; }

    std::unordered_map<int,TaskInfo> initAllTasksInfo();

    std::unordered_map<int,TaskInfo>& getAllTasksInfo() { return all_tasks_info; }

    TaskInfo& getTaskInfo(int task_id);

    double& getTaskReward(int task_id);

    std::pair<int,int> getTaskLocation(int task_id, Robot* robot); // robot passed in for TESTING ONLY

    std::pair<int,int> getTaskLocationFromArea(std::unordered_map<std::string, int>& area);

    std::unordered_map<std::string,std::vector<int>> getAllCapabilities(); // { return all_agent_capabilities; }

    std::vector<int> getRobotCapabilities(Robot* robot);

    void initMessageTracker();

    void initPingTracker();

    std::vector<int> getNeighborsInComms(int robot_id_i);

    double getMaxNeighborTimestamp(int robot_id_i, int out_of_range_robot_id_k);

};

#endif
