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
    int temp;

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
