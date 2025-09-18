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

    void bundleRemove(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        std::vector<std::vector<double>>& winning_bids_matrix,
                        bool do_test_3 = false);

    void testBundleRemove();

    void testBundleAdd(std::map<int, double>& bids);

    void buildBundle();

    std::unordered_map<int,int> initLocalWinIndicatorH();

    std::pair<double, std::unordered_map<int,Pose2D>> getFurthestPossibleDistanceInGroup(int task_id, std::unordered_map<int,Pose2D> prev_locations);

    double getDistanceAlongPathToTask(std::vector<int> path, int task_id);

    double getPathScore(std::vector<int> path, bool do_test = false);

    std::vector<int> addTaskToPath(int task_id, std::vector<int> test_path, int position_n);

    std::vector<int> shiftTaskInPath(int i, std::vector<int> path);

    int findFirstEmptyElement(std::vector<int> path);

    std::unordered_map<int, std::vector<int>> getTestPaths(int new_task_id);

    void updatePath(std::vector<int>& path, int n, int new_task_id);

    std::pair<double,int> computeBid(int task_id);

    int getBestTaskID(const std::map<int, double>& bids, const std::unordered_map<int, int>& h, std::vector<int>& bundle);

    void addTaskToBundleEnd(std::vector<int>& bundle, int task_id);

    int getBundleOrPathSize(const std::vector<int>& vec);

    double getSoloWinningBid(std::vector<std::vector<double>>& winning_bids_matrix, int task_id);

    bool isGroupEffectivelyFull(std::unordered_map<std::string, std::vector<int>> task_sub_group_assignments, std::unordered_map<std::string, int>& task_group_max_size, std::string current_robot_type);

    std::vector<int> getRelevantAssignedIDs(std::unordered_map<std::string, std::vector<int>> task_sub_group_assignments, std::string current_robot_type);

    void bundleAdd(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        std::map<int, double>& bids,
                        std::vector<std::vector<double>>& winning_bids_matrix);

    void update(int j, std::vector<std::vector<double>>& winning_bids_matrix_i, std::vector<std::vector<double>>& winning_bids_matrix_k);

    void reset(int j, std::vector<std::vector<double>>& winning_bids_matrix_i);

    void resolveSoloConflicts(int j, int id_i, int id_k, int winner_ij, int winner_kj, double winning_bid_ij, double winning_bid_kj,
                                    //std::unordered_map<int, double>& timestamps_i, std::unordered_map<int, double>& timestamps_k,
                                    const std::function<double(int)>& ts_i_m, const std::function<double(int)>& ts_k_m,
                                    std::vector<std::vector<double>>& winning_bids_matrix_i, std::vector<std::vector<double>>& winning_bids_matrix_k);

    
    std::pair<int, double> getMinExistingWinningBid(std::vector<double> winning_bids_ij, int num_agents);

    void resolveGroupConflicts(int task_idx, int agent_idx_m,  
                                    std::vector<std::vector<double>>& winning_bids_matrix_i, std::vector<std::vector<double>>& winning_bids_matrix_k,
                                    bool full_wrt_current_robot_type, int num_agents);

    void resolveConflicts(bool do_test = false);

    void testResolveConflicts(int id_i, std::vector<Msg>& message_queue, 
                                std::vector<std::vector<double>>& winning_bids_matrix_i, 
                                std::unordered_map<int,double>& timestamps_i,
                                int id_k, std::vector<std::vector<double>>& winning_bids_matrix_k, 
                                std::unordered_map<int,double>& timestamps_k);

    template <typename T>
    void print1DVector(const std::vector<T>& vec);

    template <typename T>
    void print2DVector(const std::vector<std::vector<T>>& vec);


};

#endif
