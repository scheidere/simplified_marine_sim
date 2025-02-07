#ifndef CBBA_H
#define CBBA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "planners.hpp"
#include "distance.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "structs.hpp"


struct NewWinIndicator;
struct NewWinIndicator { // h_i
    std::vector<bool> win_indicator;
    int numTasks;

    NewWinIndicator(int numTasks) : numTasks(numTasks), win_indicator(init()) {}

    std::vector<bool> init() {
        // Get the list of all tasks
        //std::vector<bool> allTasks = world->getAllTasks();
        //int n = allTasks.size(); // Get number of tasks to set indices, once and for all

        // Initialize the vector with zero for each task
        std::vector<bool> win_indicator(numTasks, true);
        return win_indicator;
    }
};


class CBBA {
private:
    int num_agents; // Number of agents (including all types)
    int num_tasks;  // Number of local tasks that each agent might be able to do depending on type
    int max_depth; // Maximum number of tasks an agent can hold in its bundle

    // Time-related stuff?
    // time window flag
    // time duration flag (all durations > 0)
    // time interval list [earliest start, latest end] if tasks are time dependent

    // General agent/task info
    std::vector<int> agent_indices; // is this really needed
    std::vector<std::string> agent_types;
    //std::vector<robot class instance> agents; tbd
    std::vector<std::string> task_types;
    //std::vector<task class or struct instance> tasks; tbd pick either class or struct to represent each task
    std::vector<std::vector<int>> capabilities; // Denotes which agents can do which tasks (0: can't; 1: can by self; 2: can co-op; 3: TBD)

    // In the following 2D vectors, one row for each agent and columns are for the task-related info
    std::vector<std::vector<int>> bundle;  // Assigned tasks for all agents
    std::vector<std::vector<int>> path;    // Task execution order for all agents
    std::vector<std::vector<double>> execution_times; // Execution times for each agent's tasks
    std::vector<std::vector<double>> scores; // Scores for each agent's tasks

    // Spatial limits X Y Z of world

    // Auction info
    std::vector<std::vector<double>> bids;
    std::vector<std::vector<int>> winners; // by index
    std::vector<std::vector<double>> winning_bids;

    // Will need to pass world class in






public:
    CBBA();

    //double createBid(Robot * robot, Task& task); // I don't think this is explicitly needed

    double calculatePathUtility(Robot& robot, Path path);

    void buildBundle(World& world, Robot& robot);

    //std::tuple<const Task&, int, int, double> findTaskForMaxScoreImprovement(Robot* robot, std::vector<Task>& allTasks, Bundle& b_i, Path& p_i, NewWinIndicator& h_i, WinningBids& y_i);
    std::tuple<Task, int, double> findTaskForMaxScoreImprovement(World& world, Robot& robot, std::vector<Task>& allTasks, Bundle& b_i, Path& p_i, NewWinIndicator& h_i, WinningBids& y_i);

    std::tuple<double, int> calculateMaxScoreImprovement(Robot& robot, Path path, double path_score_before, Task task);

    bool TaskInBundle(Bundle& bundle, Task& task);

    void printBundle();

    void obtainConsensus();


};

#endif
