#ifndef CBGA_H
#define CBGA_H

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


class CBGA {
private:

    Robot& robot; // Reference to specific robot on team
    World& world;
    JSONParser& parser;

    int max_depth; // Maximum number of tasks an agent can hold in its bundle
    int convergence_threshold;

    const double epsilon = std::numeric_limits<double>::epsilon();



public:
    CBGA(Robot& robot, World& world, JSONParser& parser);

    void init();

    int getConvergenceThreshold() const { return convergence_threshold; }

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

    double getDistanceAlongPathToTask(std::vector<int> path, int task_id);

    double getPathScore(std::vector<int> path, bool do_test = false);

    std::vector<int> addTaskToPath(int task_id, std::vector<int> test_path, int position_n);

    std::vector<int> shiftTaskInPath(int i, std::vector<int> path);

    int findFirstEmptyElement(std::vector<int> path);

    std::unordered_map<int, std::vector<int>> getTestPaths(int new_task_id);

    void updatePath(std::vector<int>& path, int n, int new_task_id);

    std::pair<double,int> computeBid(int task_id);

    //int getBestTaskID(const std::unordered_map<int, double>& bids, const std::unordered_map<int, int>& h);
    int getBestTaskID(const std::map<int, double>& bids, const std::unordered_map<int, int>& h, std::vector<int>& bundle);

    void addTaskToBundleEnd(std::vector<int>& bundle, int task_id);

    int getBundleOrPathSize(const std::vector<int>& vec);

    void bundleAdd(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        //std::vector<double>& scores,
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

    template <typename T>
    void print1DVector(const std::vector<T>& vec);

    template <typename T>
    void print2DVector(const std::vector<std::vector<T>>& vec);


};

#endif
