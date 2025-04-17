#ifndef CBBA_H
#define CBBA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <cmath>
#include <limits>
#include "message.hpp"
#include "planners.hpp"
#include "distance.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "structs.hpp"
#include "parser.hpp"


struct Msg;


class CBBA {
private:

    Robot& robot; // Reference to specific robot on team
    JSONParser& parser;

    //int num_agents; // Number of agents (including all types)
    //int num_tasks;  // Number of local tasks that each agent might be able to do depending on type
    int max_depth; // Maximum number of tasks an agent can hold in its bundle

    const double epsilon = std::numeric_limits<double>::epsilon();

    // Time-related stuff?
    // time window flag
    // time duration flag (all durations > 0)
    // time interval list [earliest start, latest end] if tasks are time dependent
    // Execution times vector commented out below

    // General agent/task info
    //std::vector<int> agent_indices; // is this really needed
    //std::vector<std::string> agent_types;
    //std::vector<robot class instance> agents; tbd
    //std::vector<std::string> task_types;
    //std::vector<task class or struct instance> tasks; tbd pick either class or struct to represent each task
    //std::vector<std::vector<int>> capabilities; // Denotes which agents can do which tasks (0: can't; 1: can by self; 2: can co-op; 3: TBD)

    // In the following 2D vectors, one row for each agent and columns are for the task-related info (up to max_depth num of tasks)
    //std::vector<int> bundle;  // Assigned tasks for all agents
    //std::vector<int> path;    // Task execution order for all agents
    //std::vector<double> execution_times; // Execution times for each agent's tasks
    //std::vector<double> scores; // Scores for each agent's tasks

    // Auction info
    /*std::vector<double> bids;
    std::vector<int> winners; // by index
    std::vector<double> winning_bids;*/


public:
    CBBA(Robot& robot, JSONParser& parser);

    void init();

    //double createBid(Robot * robot, Task& task); // I don't think this is explicitly needed

    //double calculatePathUtility(Robot& robot, Path path);

    int getTaskIndex(int task_id, std::vector<int>& vec);

    void testGetTaskIndex();

    void removeGaps(std::vector<int>& vec);

    void testRemoveGaps();

    bool isFeasible(int task_id, bool do_test_3 = false);

    //void bundleRemove();
    void bundleRemove(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        std::unordered_map<int, int>& winners, 
                        std::unordered_map<int, double>& winning_bids,
                        bool do_test_3 = false);

    void testBundleRemove();

    void buildBundle();

    std::unordered_map<int,int> initLocalWinIndicatorH();

    double getPathScore(std::vector<int> path);

    std::vector<int> addTaskToPath(int task_id, std::vector<int> test_path, int position_n);

    std::vector<int> shiftTaskInPath(int i, std::vector<int> path);

    int findFirstEmptyElement(std::vector<int> path);

    std::unordered_map<int, std::vector<int>> getTestPaths(int new_task_id);

    void updatePath(std::vector<int>& path, int n, int new_task_id);

    std::pair<double,int> computeBid(int task_id);

    //int getBestTaskID(const std::unordered_map<int, double>& bids, const std::unordered_map<int, int>& h);
    int getBestTaskID(const std::map<int, double>& bids, const std::unordered_map<int, int>& h);

    void addTaskToBundleEnd(std::vector<int>& bundle, int task_id);

    int getBundleOrPathSize(const std::vector<int>& vec);

    void bundleAdd(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        std::vector<double>& scores,
                        std::map<int, double>& bids,
                        std::unordered_map<int, int>& winners, 
                        std::unordered_map<int, double>& winning_bids);

    void update(int j, std::unordered_map<int, int>& winners_i, std::unordered_map<int, int> winners_k,
    std::unordered_map<int, double>& winning_bids_i, std::unordered_map<int, double> winning_bids_k); 

    void reset(int j, std::unordered_map<int, int>& winners_i, std::unordered_map<int, double>& winning_bids_i);
    void resolveConflicts(bool do_test = false);

    void testResolveConflicts(int id_i, std::vector<Msg>& message_queue, 
                                std::unordered_map<int, int>& winners_i, 
                                std::unordered_map<int, double>& winning_bids_i, 
                                std::unordered_map<int,double>& timestamps_i,
                                int id_k, std::unordered_map<int, int>& winners_k, 
                                std::unordered_map<int, double>& winning_bids_k, 
                                std::unordered_map<int,double>& timestamps_k);

    //std::tuple<const Task&, int, int, double> findTaskForMaxScoreImprovement(Robot* robot, std::vector<Task>& allTasks, Bundle& b_i, Path& p_i, NewWinIndicator& h_i, WinningBids& y_i);
    //std::tuple<Task, int, double> findTaskForMaxScoreImprovement(World& world, Robot& robot, std::vector<Task>& allTasks, Bundle& b_i, Path& p_i, NewWinIndicator& h_i, WinningBids& y_i);

    //std::tuple<double, int> calculateMaxScoreImprovement(Robot& robot, Path path, double path_score_before, Task task);

    //bool TaskInBundle(Bundle& bundle, Task& task);

    //void printBundle();

    //void obtainConsensus();

    template <typename T>
    void print1DVector(const std::vector<T>& vec);

    template <typename T>
    void print2DVector(const std::vector<std::vector<T>>& vec);


};

#endif
