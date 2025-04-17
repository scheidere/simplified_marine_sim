#include <cstdio>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "world.hpp"
#include "robot.hpp"
#include "CBBA.hpp"
#include "parser.hpp"
#include <utils.hpp>

class Robot;

CBBA::CBBA(Robot& r, JSONParser& p) : robot(r), parser(p) {
    init();
}

void CBBA::init() {

    max_depth = parser.getMaxDepth(); // Currently, this is only thing CBBA parses directly (the rest robot should have)
    //std::cout << max_depth << std::endl;

}

template <typename T>
void CBBA::print1DVector(const std::vector<T>& vec) {
    for (const auto& elem : vec) {
        std::cout << elem << " ";  // Print each element
    }
    std::cout << std::endl;  // Newline after
    
}

template <typename T>
void CBBA::print2DVector(const std::vector<std::vector<T>>& vec) {
    for (const auto& row : vec) {
        for (const auto& elem : row) {
            std::cout << elem << " ";  // Print each element
        }
        std::cout << std::endl;  // Newline after each row
    }
}

/*double CBBA::calculatePathUtility(Robot& robot, Path path) {
    try {
        double distance_weight = 1;
        double priority_weight = 1;
        double battery_weight;
        double battery_level = robot.getBatteryLevel();

        if (battery_level > 0.5) {
            battery_weight = 0.9;
        } else if (battery_level > 0.1) {
            battery_weight = 0.5;
        } else {
            battery_weight = 0.1;
        }

        double distance = 0;
        double task_priority_sum = 0;
        Pose2D prev_location = robot.getPose();

        for (const auto& task : path.tasks) {
            Pose2D new_location = task.location;
            double delta = Distance::getEuclideanDistance(prev_location.x, prev_location.y, new_location.x, new_location.y);
            distance += delta;
            prev_location = new_location;
            task_priority_sum += task.priority;
        }

        return distance_weight * (1.0 / distance) + priority_weight * task_priority_sum + battery_weight * battery_level;
    } catch (const std::exception& e) {
        std::cerr << "Error in calculatePathUtility: " << e.what() << std::endl;
        return 0; // Return a default value or handle the error appropriately
    }
}

std::tuple<double, int> CBBA::calculateMaxScoreImprovement(Robot& robot, Path path, double path_score_before, Task task) {
    try {
        Path test_path = path;
        double max_score_improvement = 0;
        int max_n = -1;

        for (int n = 0; n <= path.tasks.size(); n++) {
            test_path.addTask(task, n);
            double path_score_after = calculatePathUtility(robot, test_path);
            double delta_c_ij = path_score_after - path_score_before;
            if (delta_c_ij > max_score_improvement) {
                max_score_improvement = delta_c_ij;
                max_n = n;
            }
        }

        return std::make_tuple(max_score_improvement, max_n);
    } catch (const std::exception& e) {
        std::cerr << "Error in calculateMaxScoreImprovement: " << e.what() << std::endl;
        return std::make_tuple(0.0, -1); // Return default values or handle the error appropriately
    }
}

bool CBBA::TaskInBundle(Bundle& bundle, Task& task) {
    try {
        if (bundle.tasks.size() > 0) {
            for (const auto& task_in_bundle : bundle.tasks) {
                if (task.id == task_in_bundle.id) {
                    return true;
                }
            }
        }
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error in TaskInBundle: " << e.what() << std::endl;
        return false; // Return a default value or handle the error appropriately
    }
}

std::tuple<Task, int, double> CBBA::findTaskForMaxScoreImprovement(World& world, Robot& robot, std::vector<Task>& allTasks, Bundle& b_i, Path& p_i, NewWinIndicator& h_i, WinningBids& y_i) {
    try {
        Pose2D placeholder{0, 0, 0};
        Task best_task_J(0, "Placeholder task", placeholder, 0, 0, 0);
        int best_index_n = -1;
        double path_score_before = calculatePathUtility(robot, p_i);
        double overall_max_score_improvement = 0;

        for (auto& task_j : allTasks) {
            if (!TaskInBundle(b_i, task_j)) {
                int j = world.getTaskIndex(task_j);
                auto [max_score_improvement, n] = calculateMaxScoreImprovement(robot, p_i, path_score_before, task_j);
                h_i.win_indicator[j] = max_score_improvement > y_i.winning_bids[j];
                if (max_score_improvement > overall_max_score_improvement) {
                    overall_max_score_improvement = max_score_improvement;
                    best_task_J = task_j;
                    best_index_n = n;
                }
            }
        }

        return std::make_tuple(best_task_J, best_index_n, overall_max_score_improvement);
    } catch (const std::exception& e) {
        std::cerr << "Error in findTaskForMaxScoreImprovement: " << e.what() << std::endl;
        Pose2D placeholder{0, 0, 0};
        Task default_task(0, "Error Task", placeholder, 0, 0, 0);
        return std::make_tuple(default_task, -1, 0.0); // Return default values or handle the error appropriately
    }
}*/

void CBBA::testGetTaskIndex() {

    // Passed (idx of 2 is 1)

    std::vector<int> testv = {1,2,3};
    utils::log1DVector(testv, robot);
    int idx = getTaskIndex(2, testv);
    std::string bla = "Task index of 2 in the above vector is: " + std::to_string(idx);
    robot.log_info(bla);

}


void CBBA::buildBundle() {
    try {
        std::cout << "in CBBA::buildBundle..." << std::endl;

        // Let's test the individual functions used in bundleRemove()
        //testGetTaskIndex(); // Passed
        //testRemovePathGaps();
        //testBundleRemove();

        std::vector<int>& bundle = robot.getBundle();
        std::vector<int>& path = robot.getPath();
        std::unordered_map<int, int>& winners = robot.getWinners();
        std::unordered_map<int, double>& winning_bids = robot.getWinningBids();

        bundleRemove(bundle, path, winners, winning_bids); // Update bundle if new info reveals mistakes

        std::vector<double>& scores = robot.getScores();
        std::map<int, double>& bids = robot.getBids();

        bundleAdd(bundle, path, scores, bids, winners, winning_bids); // Populate bundle if any empty slots

        std::string log_msg = "Building bundle for robot " + std::to_string(robot.getID()) + "...";
        robot.log_info(log_msg);
        std::string b = "Bundle: ";
        robot.log_info(b);
        utils::log1DVector(robot.getBundle(), robot);
        std::string p = "Path: ";
        robot.log_info(p);
        utils::log1DVector(robot.getPath(), robot);

        /*
        Notes:
        1. Path order logic for ties: Since getTestPaths() inserts in a certain way (the new task ID first at left, then each between element, then the end/right),
        the lowest task ID won't always be prioritized to the left of an equally performing higher task ID in the path.
        For example, the path may be 2 3 1 (and 1 and 3 affect the path the same way) instead of 2 1 3... 
        In this example, these paths perform the same, and during bundle building, task 2 is chosen, then task 1 (path is 2 1 -1 at this point), then task 3...
        When task 3 is added, it first looks at test paths 3 2 1, then 2 3 1, then 2 1 3. Since the final two have the same score, 
        the first test path to win is chosen: 2 3 1.
        */

        /*std::cout << "at end of CBBA::buildBundle..." << std::endl;*/
    } catch (const std::exception& e) {
        std::cerr << "Error in buildBundle: " << e.what() << std::endl;
    }
}

int CBBA::getTaskIndex(int task_id, std::vector<int>& vec) {
    // Note, this is index as in location in the bundle or path (or other relevant vector)
    // This is not task ID

    auto it = std::find(vec.begin(), vec.end(), task_id);
    if (it != vec.end()) {
        int index = std::distance(vec.begin(), it);
        //std::cout << "The index of " << task_id << " is: " << index << std::endl;
        return index;
    }

    return -1; // Valid int but invalid index to denote error: task_id not found
}

void CBBA::removeGaps(std::vector<int>& vec) {
    // Remove gaps (-1) in the given vector so they only appear at the end

    int write_idx = 0;  // Position to write the first non-empty value

    // Move non-empty (-1) values to start
    for (int read_idx = 0; read_idx < vec.size(); ++read_idx) {
        //std::string bla = "write_idx: " + std::to_string(write_idx) + " read_idx: " + std::to_string(read_idx);
        //robot.log_info(bla);
        //utils::log1DVector(path,robot);
        if (vec[read_idx] != -1) {
            vec[write_idx] = vec[read_idx];
            write_idx++;
        }
    }

    // Fill the remaining elements with -1 (more efficient than writing each in loop)
    std::fill(vec.begin() + write_idx, vec.end(), -1);
}

void CBBA::testRemoveGaps() {

    // In progress 
    // Passed (result should be {3,-1,-1} for {-1,-1,3})
    // Passed (should be {2,3,-1} for {2,-1,3})
    // Passed (should be {2,3,-1} for {-1,2,3})

    //std::vector<int> test = {-1, -1, 3};  // Example test vector
    //std::vector<int> test = {2, -1, 3};  // Example test vector
    std::vector<int> test = {-1, 2, 3};  // Example test vector
    std::vector<int>& path = robot.getPath();  

    path = test;
    std::vector<int>& path2 = robot.getPath(); // see if assigning test worked
    robot.log_info("Path before removing gaps: ");
    utils::log1DVector(path2, robot);
    removeGaps(path2);
    robot.log_info("Path after removing gaps: ");
    std::vector<int>& path3 = robot.getPath();
    utils::log1DVector(path3, robot);
}

bool CBBA::isFeasible(int task_id, bool do_test_3) {

    if (do_test_3) {
        // Without changes it's {1,2,3,4} for the other tests
        // Let's make it 3,4 so 1 and 2 must be removed from bundles
        std::vector<int> test_doable_task_ids = {3,4};
        return std::find(test_doable_task_ids.begin(), test_doable_task_ids.end(), task_id) != test_doable_task_ids.end();
    }

    std::vector<int> doable_task_ids = robot.getDoableTaskIDs(); // We will remove tasks from this elsewhere when they are found to become infeasible
    return std::find(doable_task_ids.begin(), doable_task_ids.end(), task_id) != doable_task_ids.end();
}

//void CBBA::bundleRemove() {
void CBBA::bundleRemove(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        std::unordered_map<int, int>& winners, 
                        std::unordered_map<int, double>& winning_bids,
                        bool do_test_3) {

    robot.log_info("In bundleRemove...");
    utils::log1DVector(bundle,robot);
    utils::log1DVector(path,robot);
    utils::logUnorderedMap(winners,robot);
    utils::logUnorderedMap(winning_bids,robot);

    for (auto it = bundle.begin(); it != bundle.end(); ++it) {
        int task_id = *it;

        // Skip if task_id is -1 because empty
        if (task_id == -1) {
            continue;
        }

        bool new_winner_found = (winners[task_id] != robot.getID());

        robot.log_info("New winner found: " + std::to_string(new_winner_found) + " winners[task_id]: " + std::to_string(winners[task_id]));

        if (new_winner_found || !isFeasible(task_id, do_test_3)) {
            *it = -1;  // Remove task id from bundle and replace with -1 to denote empty

            // Find in path and remove
            auto p_idx = getTaskIndex(task_id, path);
            path[p_idx] = -1;

            // Update winners and winning_bids vectors if not already
            if (!new_winner_found) {
                robot.log_info("No new winner found, reflecting change via removing current robot id as winner for removed task...");
                winners[task_id] = -1;
                winning_bids[task_id] = 0.0;
            }

            // Log winners after modification
            robot.log_info("After modification, winners: ");
            for (const auto& winner : winners) {
                robot.log_info("task_id: " + std::to_string(winner.first) + " winner: " + std::to_string(winner.second));
            }
        }
    }

    removeGaps(bundle);
    removeGaps(path);

    robot.log_info("Final results for bundleRemove (bundle, path, winners, winning_bids): ");
    utils::log1DVector(bundle,robot);
    utils::log1DVector(path,robot);
    utils::logUnorderedMap(winners,robot);
    utils::logUnorderedMap(winning_bids,robot);



}

void CBBA::testBundleRemove() {

    // Test 1: Passes with the below (robot 2 ends up with empty bundle and path, robot 1 has no changes)
    /*std::vector<int> bundle = {1,2,3};
    std::vector<int> path = {1,2,3};
    std::unordered_map<int, int> winners = {
    {1, 1},
    {2, 1}, 
    {3, 1}  
    };
    std::unordered_map<int, double> winning_bids = {
    {1, 10.5},
    {2, 12.0},
    {3, 9.8}
    };*/

    // Test 2: Passed (robot 2 ends with 1,-1,-1, robot 1 ends with 2,-1,-1)
    /*std::vector<int> bundle = {1,2,-1};
    std::vector<int> path = {2,1,-1};
    std::unordered_map<int, int> winners = {
    {1, 2},
    {2, 1}, 
    {3, 1}  // not relevant for this test
    };
    std::unordered_map<int, double> winning_bids = { // don't think this matters
    {1, 10.5},
    {2, 12.0},
    {3, 9.8}
    };*/

    //bundleRemove(bundle,path,winners,winning_bids); // Call for test 1 or test 2

    // In progress, test case where no new winner found but found task in bundle that !isFeasible (so should see changes to winners/winning_bids)
    //do this and remove prints from bundleRemove
    //loop thru doable tasks to test isFeasible with true for test 3 (should be 0,0,1,1)

    // isFeasible do_test_3 = true test - passed
    /*std::vector<int> doable_task_ids = robot.getDoableTaskIDs();
    for (auto i : doable_task_ids) {
        bool is_feas = isFeasible(i,true);
        std::string bla = "Originally doable task " + std::to_string(i) + " has " + std::to_string(is_feas) + " feasibility";
        robot.log_info(bla);

    }*/

    // Test 3: Passed (focusing on robot 1 for this test, expect bundle to be 3,-1,-1 at end)
    // Must focus on 1 robot only because the winners list can't have both robots win the same tasks 
    // In reality/simulations later, the vectors will be local, i.e., different instances
    // Note robot 2 will just end up with empty bundle (this is because of winners being all robot 1)
    std::vector<int> bundle = {2,1,3};
    std::vector<int> path = {3,2,1};
    std::unordered_map<int, int> winners = {
    {1, 1},
    {2, 1}, 
    {3, 1}
    };
    std::unordered_map<int, double> winning_bids = { // don't think this matters
    {1, 10.5},
    {2, 12.0},
    {3, 9.8}
    };

    bundleRemove(bundle,path,winners,winning_bids, true); // Call for test 3

}

std::unordered_map<int,int> CBBA::initLocalWinIndicatorH() {

    std::unordered_map<int,int> local_win_indicator_h;

    for (auto pair : robot.getBids()) { // traverse doable tasks not in bundle as tracked by bids map
        int task_id = pair.first;
        local_win_indicator_h[task_id] = 0; // Initialize each key with 0
    }

    return local_win_indicator_h;
}

double CBBA::getPathScore(std::vector<int> path) {
    double score = 0.0;
    
    for (int i = 0; i < getBundleOrPathSize(path); i++) {
        if (path[i] == 2) {
            score += (i == 0) ? 3.0 : 2.0; // val if true: val if false
        } else {
            score += 1.0; // Assign base reward for other tasks
        }
    }

    return score;
}

std::vector<int> CBBA::addTaskToPath(int task_id, std::vector<int> test_path, int position_n) {
 
    // Pass in path that has a valid empty slot (assume that position n is valid)

    //robot.log_info("in addTaskToPath");

    //utils::log1DVector(test_path, robot);

    test_path[position_n] = task_id;

    //utils::log1DVector(test_path, robot);

    //robot.log_info("end addTaskToPath");

    return test_path;
}

std::vector<int> CBBA::shiftTaskInPath(int i, std::vector<int> path) {

    // TODO: double check catches all cases!

    //robot.log_info("in shiftTaskInPath");

    // Shift task at i-1 to i (so shifting -1 from i to i-1 as well)
    std::vector<int> shifted = path;

    //utils::log1DVector(shifted, robot);

    shifted[i] = path[i-1]; shifted[i-1] = path[i];

    //utils::log1DVector(shifted, robot);

    //robot.log_info("end shiftTaskInPath");

    return shifted;
}

int CBBA::findFirstEmptyElement(std::vector<int> path) {

    for (int i=0; i<path.size(); i++) {
        if (path[i]==-1) {
            return i;
        }
    }

    return -1; // Path is full (should not get here because bundle would be full and stop it)
}

std::unordered_map<int, std::vector<int>> CBBA::getTestPaths(int new_task_id) {

    // Returns unordered map where keys denote index where new task inserted in new path to be tested
    // New test path is value of each key (with the new task id at n)

    //robot.log_info("in getTestPaths:");

    std::unordered_map<int, std::vector<int>> test_paths;

    std::vector<int> path = robot.getPath(); // Makes a copy
    //robot.log_info("Path before changes:");
    //utils::log1DVector(path, robot);

    int e = findFirstEmptyElement(path); // Find index of first empty element (-1)
    //std::string str1 = "first empty element e: " + std::to_string(e);
    //robot.log_info(str1);
    if (e==-1) {
        std::string log_msg = "The path is actually full so there can be no test paths (e=-1)";
        robot.log_info(log_msg);
    }

    // First, insert at e (the first empty element index) - no shifting of other tasks in path (if any) required
    std::vector<int> first_test_path = addTaskToPath(new_task_id,path,e);
    test_paths[e] = first_test_path;
    //robot.log_info("Add task at 'end' aka rightmost empty slot:");
    //utils::log1DVector(first_test_path, robot);

    //robot.log_info("Shifting and adding new task between (while maintaining existing overarching order)...");
    // Next, if any, shift tasks one to the right, one by one (inserting the new task where shifted task was)
    std::vector<int> shifted_path = path; // Init before any shifts
    for (int i=e; i>0; i--) {
        // Get new copy?
        shifted_path = shiftTaskInPath(i, shifted_path); // Shift existing task from i-1 to i
        std::vector<int> new_test_path = addTaskToPath(new_task_id, shifted_path,i-1); // Insert new task id at i-1
        test_paths[i-1] = new_test_path; // Save it based on where new task is now
        //utils::log1DVector(new_test_path, robot);
    }

    /*robot.log_info("Checking test_paths");
    for (auto& [n, test_path] : test_paths) {
        std::string msg = "n: " + std::to_string(n);
        robot.log_info(msg);
        utils::log1DVector(test_path,robot);
    }*/

    robot.log_info("end getTestPaths");

    return test_paths;


}

std::pair<double, int> CBBA::computeBid(int task_id) {
    // Calculate maximum score improvement (will use as bid)
    double max_score_improvement = 0.0;
    double marginal_score_improvement_c = 0.0;
    int best_position_n = -1;

    robot.log_info("in computeBid");

    // Current path
    std::vector<int> path = robot.getPath();

    // Get score for current path (without adding new task)
    //robot.log_info("cuuureeennnttt path score call");
    double current_path_score = getPathScore(path);
    //robot.log_info("end current_path_score");

    std::unordered_map<int, std::vector<int>> test_paths = getTestPaths(task_id);

    //robot.log_info("Checking loop in computeBid...");
    for (auto& [n,test_path] : test_paths) {
        std::string str2 = "n: " + std::to_string(n) + ", test_path:";
        robot.log_info(str2);
        utils::log1DVector(test_path,robot);
        //robot.log_info("why no new path score print");
        double new_path_score = getPathScore(test_path);
        std::string str3 = "new path score: " + std::to_string(new_path_score);
        robot.log_info(str3);
        marginal_score_improvement_c = new_path_score - current_path_score;
        std::string bleu = "marginal_score_improvement_c: " + std::to_string(marginal_score_improvement_c);
        robot.log_info(bleu);
        std::string bleu2 = "max_score_improvement: " + std::to_string(max_score_improvement);
        robot.log_info(bleu2);
        if (marginal_score_improvement_c > max_score_improvement) {
            robot.log_info("Found new max max_score_improvement");
            max_score_improvement = marginal_score_improvement_c;
            best_position_n = n;
        }
    }

    if (best_position_n != 0) {
        robot.log_info("FOUND: best position n is something other than 0 in computeBid");
    }

    // return max marginal score improvement for that task and the n describing where it must be in the path to achieve this score
    return std::make_pair(max_score_improvement, best_position_n);

    //return 1.0; // For now just a num to test
}

//int CBBA::getBestTaskID(const std::unordered_map<int, double>& bids, const std::unordered_map<int, int>& h) {
int CBBA::getBestTaskID(const std::map<int, double>& bids, const std::unordered_map<int, int>& h) {
    // Get the task id for the task that has the max new winning bid, J
    // h is local win indicator with elements 0 or 1

    //robot.log_info("in getBestTaskID");
    //robot.log_info("indicator h:");
    //utils::logUnorderedMap(h, robot);

    double max_product = 0.0;
    int ID_of_max = -1;

    for (auto [task_id,bid] : bids) {
    //for (int i: i<bids.size(); i++) { // to traverse by increasing ID number (actually going to sort the unordered map when created)
        //auto [task_id,bid] = [i]
        double product = bid*h.at(task_id); //.at(task_id) may be better because [task_id] defaults to 0 if task_id not a key
        if (product - max_product > epsilon) { // product is more than epsilon bigger than previous max
            max_product = product;
            ID_of_max = task_id;
        }
    }

    if (ID_of_max==-1) { // no max found (likely all 0 in h indicating all tied)
        ID_of_max = bids.begin()->first;// Pick lowest task id (as a default)
    }


    //robot.log_info("end getBestTaskID");
    return ID_of_max;

}

void CBBA::addTaskToBundleEnd(std::vector<int>& bundle, int task_id) {
    auto it = std::find(bundle.begin(), bundle.end(), -1); // Find first occurrence of -1
    if (it != bundle.end()) {
        *it = task_id; // Replace -1 with the new task ID
    }
}

int CBBA::getBundleOrPathSize(const std::vector<int>& vec) {
    return std::count_if(vec.begin(), vec.end(), [](int task_id) {
        return task_id != -1;
    });
}

void CBBA::updatePath(std::vector<int>& path, int n, int new_task_id) {

    if (path[n]==-1) { // for first task in path or ones added after that cleanly based on n
        // Aleady empty so just assign new task to n position in path
        path[n] = new_task_id;
    } else { // shift existing task in path to prioritize new task over the one tied in that location
        for (int i = path.size() - 1; i > n; --i) {
            path[i] = path[i - 1];  // Move each task one position to the right
        }
        //path[n+1] = path[n]; // does this work?
        path[n] = new_task_id;

    }
}

void CBBA::bundleAdd(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        std::vector<double>& scores,
                        std::map<int, double>& bids,
                        std::unordered_map<int, int>& winners, 
                        std::unordered_map<int, double>& winning_bids) {
    try {

        
        int size = getBundleOrPathSize(bundle);

        std::vector<int> test = robot.getDoableTaskIDs();

        // Check if bundle is full (i.e., at max_depth)
        while (getBundleOrPathSize(bundle) < max_depth) {

            robot.log_info("++++ While loop start ++++");
            robot.log_info("Bundle:");
            utils::log1DVector(bundle, robot);
            robot.log_info("Path:");
            utils::log1DVector(path, robot);
            robot.log_info("Bids:");
            utils::logMap(bids, robot); // bids is map, not unordered
            robot.log_info("++++  ++  ++++");



            std::unordered_map<int, int> local_win_indicator_h = initLocalWinIndicatorH();
            std::unordered_map<int,int> best_position_n_tracker; // keys are task id's and values are best index n in path
            for (auto task_id : robot.getDoableTaskIDs()) {
                if ( std::find(bundle.begin(), bundle.end(), task_id) == bundle.end() ) { // for each doable task not in bundle already
                    //std::string str1 = "Looking at the following doable task not in bundle:" + std::to_string(task_id);
                    std::string str1 = "-----Potential next task: " + std::to_string(task_id);
                    robot.log_info(str1);
                    std::pair<double,int> bid_and_best_path_position = computeBid(task_id);
                    bids[task_id] = bid_and_best_path_position.first; best_position_n_tracker[task_id] = bid_and_best_path_position.second;
                    //robot.log_info("After computeBid update, bids: ");
                    //utils::logMap(bids, robot);
                    robot.log_info("and best position tracker: ");
                    utils::logUnorderedMap(best_position_n_tracker, robot);
                    if ( bids[task_id] - winning_bids[task_id] > epsilon ) {
                        // Found better bid so track in local win indicator h
                        std::string log_msg1 = "For task " + std::to_string(task_id) + " bid (" + std::to_string(bids[task_id]) + ") > winning bid(" + std::to_string(winning_bids[task_id]) + ")";
                        robot.log_info(log_msg1);
                        local_win_indicator_h[task_id] = 1;

                    } else {
                        std::string log_msg1 = "For task " + std::to_string(task_id) + "bid (" + std::to_string(bids[task_id]) + ") NOT > winning bid(" + std::to_string(winning_bids[task_id]) + ")";
                        robot.log_info(log_msg1);
                    }


                     /*else if ( bids[task_id] - winning_bids[task_id] <= epsilon ) {
                        // Found equally good bid, do not replace it
                        std::string log_msg6 = "For task " + std::to_string(task_id) + "bid (" + std::to_string(bids[task_id]) + ") ~= winning bid(" + std::to_string(winning_bids[task_id]) + ")";
                        robot.log_info(log_msg6);
                    }*/
                }
            }
            // Get the task id for the task that has the max new winning bid, J
            int J = getBestTaskID(bids, local_win_indicator_h);
            std::string blob = "Best task ID is " + std::to_string(J); 
            robot.log_info(blob);
            int n = best_position_n_tracker[J];
            std::string loggy = "Best position in path is: " + std::to_string(n);
            robot.log_info(loggy);
            //robot.log_info("before bundle change ===============================");
            //utils::log1DVector(bundle, robot);
            addTaskToBundleEnd(bundle, J); // Insert J at end of bundle
            //robot.log_info("after bundle change ===============================");
            //utils::log1DVector(bundle, robot);
            //robot.log_info("=====");
            updatePath(path,n,J);
            //path[n] = J; // Insert J at n in path
            winning_bids[J] = bids[J]; // Add bid associated with J to winning bid list
            winners[J] = robot.getID(); // Add current agent ID since it just won
            bids.erase(J); // Remove task now in bundle from future consideration

        }
    
        //std::string log_msg2 = "Bundle is this after bundleAdd:";
        //robot.log_info(log_msg2);
        //utils::log1DVector(bundle, robot);

    } catch (const std::exception& e) {
        std::cerr << "Error in bundleAdd: " << e.what() << std::endl;
    };
}

void CBBA::update(int j, std::unordered_map<int, int>& winners_i, std::unordered_map<int, int> winners_k,
    std::unordered_map<int, double>& winning_bids_i, std::unordered_map<int, double> winning_bids_k) {

    // Precedence given to k, which is determined to have more recent info than i
    // Winners update z_ij = z_kj and winning bids update y_ij = y_kj
    winners_i[j] = winners_k[j]; winning_bids_i[j] = winning_bids_k[j];

}

void CBBA::reset(int j, std::unordered_map<int, int>& winners_i, std::unordered_map<int, double>& winning_bids_i) {

    winners_i[j] = -1; winning_bids_i[j] = 0.0;
}

void CBBA::resolveConflicts(bool do_test) {

    // Given communication between at least two agents, check for conflicts in shared info
    // Option 1: Update agent i's y (winning bids) and z (winners) with agent k's
    // Option 2: Reset agent i's y and z's (to empty/0)
    // Option 3: Leave aka no changes
    // Following table 1
    // ACTION RULE FOR AGENT i (receiver, current) BASED ON COMMUNICATION WITH AGENT k (sender, other) REGARDING TASK j

    // Current robot i
    int id_i = robot.getID(); 
    std::vector<Msg> message_queue_i = robot.getMessageQueue();
    std::unordered_map<int, int> winners_i = robot.getWinners();
    std::unordered_map<int, double> winning_bids_i = robot.getWinningBids();
    std::unordered_map<int,double> timestamps_i = robot.getTimestamps();

    for (auto& msg : message_queue_i) {
        int id_k = msg.id;
        std::unordered_map<int, int> winners_k = msg.winners;
        std::unordered_map<int, double> winning_bids_k = msg.winning_bids;
        std::unordered_map<int, double> timestamps_k = msg.timestamps;
        std::string bla = "Message from robot " + std::to_string(id_k) + ":";
        robot.log_info(bla);
        robot.log_info("Winners: ");
        utils::logUnorderedMap(winners_k,robot);
        robot.log_info("Winning bids: ");
        utils::logUnorderedMap(winning_bids_k,robot);
        robot.log_info("Timestamps: ");
        utils::logUnorderedMap(timestamps_k,robot);

        if (do_test) {
            // This will change parts of these vectors/unordered maps to test different parts of the conflict resolution logic
            testResolveConflicts(id_i, message_queue_i, winners_i, winning_bids_i, timestamps_i,
                                id_k, winners_k, winning_bids_k, timestamps_k);
        }

        for (auto& [j, winner_ij] : winners_i) { // Looping all tasks j
            if (winner_ij == winners_k[j]) { // CASE 1: If i and k agree about winner of task j
                if (winner_ij == id_k) { // If winner is k
                    update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                }
                else if (winner_ij != -1 and winner_ij != id_i) { // If winner is another, m (not i or k)
                    int m = winner_ij;
                    if (timestamps_k[m] > timestamps_i[m]) {
                        update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                    }
                }
                // If winner is i or unsure, leave (no change)


            } else if (winners_k[j] == id_k) { // CASE 2: They don't agree and k thinks it is the winner of task j
                if (winner_ij == id_i && winning_bids_k[j] > winning_bids_i[j]) { // If i thinks i
                    update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                } else if (winner_ij == -1) { // If i is unsure
                    update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                } else { // If i thinks m
                    int m = winner_ij;
                    if (timestamps_k[m] > timestamps_i[m] || winning_bids_k[j] > winning_bids_i[j]) {
                        update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                    } 

                }
                
            } else if (winners_k[j] == id_i) { // CASE 3: They don't agree and k thinks i should be the winner of task j
                if (winner_ij == id_k) { // If i thinks k
                    reset(j, winners_i, winning_bids_i);
                } else if (winner_ij != -1) { // If i thinks m
                    int m = winner_ij;
                    if (timestamps_k[m] > timestamps_i[m]) {
                        reset(j, winners_i, winning_bids_i);
                    }
                }
                // If i is unsure, leave (no change)

            } else if (winners_k[j] != -1) { // CASE 4: They don't agree and k thinks another agent m (not i or k) wins task j
                // Only other option is unsure aka -1, so if not -1, then must be another id m

                if (do_test) {robot.log_info("in else for k thinks m");}

                int m = winners_k[j];

                if (winner_ij == id_i) { // If i thinks i
                    if (timestamps_k[m] > timestamps_i[m] && winning_bids_k[j] > winning_bids_i[j]) { 
                        update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);   
                    } 
                } else if (winner_ij == id_k) { // If i thinks k
                    if (timestamps_k[m] > timestamps_i[m]) {
                        update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                    } else {
                        reset(j, winners_i, winning_bids_i);
                    }
                } else if (winner_ij == -1) { // If i is unsure
                    if (timestamps_k[m] > timestamps_i[m]) {
                        update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                    }

                } else { // If i thinks another unique robot n (not i k or m)
                    int n = winner_ij;
                    if (timestamps_k[m] > timestamps_i[m] && ( timestamps_k[n] > timestamps_i[n] || winning_bids_k[j] > winning_bids_i[j])) {
                        update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                    } else if (timestamps_k[n] > timestamps_i[n] && timestamps_i[m] > timestamps_k[m]) {
                        reset(j, winners_i, winning_bids_i);
                    }
                }

            } else if (winners_k[j] == -1) { // CASE 5: They don't agree and k is unsure

                if (do_test) {robot.log_info("in else for k is unsure");}

                if (winner_ij == id_k) { // If i thinks k
                    update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                } else if (winner_ij != id_i && winner_ij != -1) { // If i thinks m
                    int m = winner_ij;
                    if (timestamps_k[m] > timestamps_i[m]) {
                        update(j,winners_i,winners_k,winning_bids_i,winning_bids_k);
                    }
                }
                // If i thinks i, leave (no change)

            }
        }

        if (do_test) {
            robot.log_info("Current robot's winners and winning_bids after resolveConflicts...");
            utils::logUnorderedMap(winners_i, robot);
            utils::logUnorderedMap(winning_bids_i, robot);

        }


    }

}

// inputs: robot i winners, winning_bids, message_queue (to get msg's from each k), timestamps
void CBBA::testResolveConflicts(int id_i, std::vector<Msg>& message_queue, std::unordered_map<int, int>& winners_i, 
    std::unordered_map<int, double>& winning_bids_i, std::unordered_map<int,double>& timestamps_i,
    int id_k, std::unordered_map<int, int>& winners_k, 
    std::unordered_map<int, double>& winning_bids_k, std::unordered_map<int,double>& timestamps_k) {

    robot.log_info("Before testResolveConflicts changes anything for testing...");
    std::string bla = "Current robot " + std::to_string(id_i) + " (winners, winning_bids, timestamps):";
    robot.log_info(bla);
    utils::logUnorderedMap(winners_i, robot);
    utils::logUnorderedMap(winning_bids_i, robot);
    utils::logUnorderedMap(timestamps_i, robot);
    std::string bla1 = "Other robot " + std::to_string(id_k) + " (winners, winning_bids, timestamps):";
    robot.log_info(bla1);
    utils::logUnorderedMap(winners_k, robot);            
    utils::logUnorderedMap(winning_bids_k, robot);
    utils::logUnorderedMap(timestamps_k, robot);
 

    // ***** TEST 1: If robot i and robot k agree on winner of some task j
    int task_j = 1; // This is random

    //  // Test 1.A 
    // They agree on k winning for task j (this should result in update of winners and winning bids for robot i) - PASSED
    /*winners_i[task_j] = id_k;
    winners_k[task_j] = id_k;
    winning_bids_k[task_j] = 10;*/ // Making it different so we see an update if it happens (as it should)
    
    //  // Test 1.B
    // They agree on i winning for task j - Should result in no change - PASSED
    /*winners_i[task_j] = id_i;
    winners_k[task_j] = id_i;
    winning_bids_k[task_j] = 10;*/

    //  // Test 1.C
    // They are both unsure who should win task j - Should result in no change - PASSED
    /*winners_i[task_j] = -1;
    winners_k[task_j] = -1;
    winning_bids_k[task_j] = 10;*/

    //  // Test 1.D
    // They agree on another robot m winning for task j (not i or k) (result depends on timestamp - see 2 cases below)
    // Let's choose m = 3 (technically testing with just 2 robots for simplicity so manufacturing this)
    /*int id_m = 3;
    winners_i[task_j] = id_m;
    winners_k[task_j] = id_m;
    winning_bids_k[task_j] = 10;*/

    // 1.D.1 - Should result in update because km timestamp more recent - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;*/

    // 1.D.2 - Should result in no change because im timestamp more recent - PASSED
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;*/

    // ***** Test 2: If robot i and robot k don't agree and k thinks k is winner of task j
    //winners_k[task_j] = id_k; // UNCOMMENT part 1/3

    //  // Test 2.A
    // i thinks i and k thinks k - Result depends on winning bids
    //winners_i[task_j] = id_i;

    // 2.A.1 - Should result in update because k winning bid higher than i winning bid - PASSED
    /*winning_bids_i[task_j] = 5;
    winning_bids_k[task_j] = 10;*/

    // 2.A.2 - Should result in no change because i winning bid higher than k winning bid - PASSED
    /*winning_bids_i[task_j] = 10;
    winning_bids_k[task_j] = 5;*/

    //  // Test 2.B
    // i is unsure - Should result in update (winners, winning bids) in every case
    /*winners_i[task_j] = -1;
    winning_bids_i[task_j] = -1; // Here just to observe update
    winning_bids_k[task_j] = 5;*/ // Here just to observe update

    //  // Test 2.C 
    // UNCOMMENT part 2/3 (example)
    // i thinks another robot m
    /*int id_m = 3;
    winners_i[task_j] = id_m;*/

    // 2.C.1 - Should result in update because k has higher timestamp from m (even though lower bid for j) - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;
    winning_bids_i[task_j] = 10;
    winning_bids_k[task_j] = 5;*/

    // 2.C.2 - Should result in update because k has higher timestamp from m and k has higher bid for j - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;
    winning_bids_i[task_j] = 5;
    winning_bids_k[task_j] = 10;*/

    // 2.C.3 - Should result in no change because i has higher timestamp and bid - PASSED
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;
    winning_bids_i[task_j] = 10;
    winning_bids_k[task_j] = 5;*/

    // 2.C.4 - Should result in update because k has higher bid for j (even though lower timestamp) - PASSED
    // UNCOMMENT part 3/3 (example)
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;
    winning_bids_i[task_j] = 5;
    winning_bids_k[task_j] = 10;*/

    // ***** Test 3: If robot i and robot k don't agree and k thinks i is winner of task j
    //winners_k[task_j] = id_i; // UNCOMMENT part 1/3

    //  // Test 3.A
    // i thinks k - Should result in reset - PASSED
    //winners_i[task_j] = id_k;

    //  // Test 3.B
    // i thinks another robot m;
    // UNCOMMENT part 2/3 (example)
    /*int id_m = 3;
    winners_i[task_j] = id_m;*/

    // 3.B.1 - Should not change because i has more recent info from m - PASSED
    // UNCOMMENT part 3/3 (example)
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;*/

    // 3.B.2 - Should reset because k has more recent info from m - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;*/

    //  // Test 3.C
    // i is unsure - Should result in no change - PASSED
    //winners_i[task_j] = -1;

    // ***** Test 4: If robot i and robot k don't agree and k thinks another robot m is winner of task j
    // UNCOMMENT part 1/3
    /*int id_m = 3;
    winners_k[task_j] = id_m;*/

    //  // Test 4.A
    // i thinks i
    //winners_i[task_j] = id_i; // UNCOMMENT part 2/3 (example)

    // 4.A.1 - Should result in no change - PASSED
    // UNCOMMENT part 3/3 (example)
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;
    winning_bids_i[task_j] = 5;
    winning_bids_k[task_j] = 10;*/


    // 4.A.2 - Should result in update - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;
    winning_bids_i[task_j] = 5;
    winning_bids_k[task_j] = 10;*/

    // 4.A.3 - Should result in no change - PASSED
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;
    winning_bids_i[task_j] = 10;
    winning_bids_k[task_j] = 5;*/


    // 4.A.4 - Should result in no change - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;
    winning_bids_i[task_j] = 10;
    winning_bids_k[task_j] = 5;*/


    //  // Test 4.B
    // i thinks k
    //winners_i[task_j] = id_k;

    // 4.B.1 - Should result in reset - PASSED
    // UNCOMMENT part 3/3 (example)
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;*/

    // 4.B.2 - Should result in update - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;*/

    //  // Test 4.C
    // i is unsure
    //winners_i[task_j] = -1;

    // 4.C.1 - Should result in no change - PASSED
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;*/

    // 4.C.2 - Should result in update - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;*/

    //  // Test 4.D
    // is thinks another robot n
    /*int id_n = 4;
    winners_i[task_j] = id_n;*/

    // 4.D.1 - Should update since km and kn timestamp win over im and in respectively (k wins both timestamps) - PASSED
    /*timestamps_i[id_m] = 2; 
    timestamps_k[id_m] = 3; // k wins this
    timestamps_i[id_n] = 2; 
    timestamps_k[id_n] = 3; // and either k wins this
    winning_bids_i[task_j] = 10;
    winning_bids_k[task_j] = 5;*/ // or k wins this (but doesn't in this example)


    // 4.D.2 - Should update since timestamp km > im and winning bid kj > ij (k wins m timestamp and winning bids) - PASSED
    /*timestamps_i[id_m] = 2; 
    timestamps_k[id_m] = 3; // k wins this
    timestamps_i[id_n] = 3; 
    timestamps_k[id_n] = 2; // and either k wins this (but doesn't in this example)
    winning_bids_i[task_j] = 5;
    winning_bids_k[task_j] = 10;*/ // or k wins this

    // 4.D.3 - Should reset because i wins timestamp for m (but thinks n), and k wins timestamp for n (but thinks m) - PASSED
    /*timestamps_i[id_m] = 3; // i wins this 
    timestamps_k[id_m] = 2;
    timestamps_i[id_n] = 2; 
    timestamps_k[id_n] = 3; // and k wins this
    // winning bids should be irrelevant so making them same
    winning_bids_i[task_j] = 10;
    winning_bids_k[task_j] = 10;*/

    // ***** Test 5: If robot i and robot k don't agree and k is unsure who should win task j
    // UNCOMMENT part 1/3
    winners_k[task_j] = -1;

    //  // Test 5.A
    // i thinks i - Should be no change - PASSED
    //winners_i[task_j] = id_i;

    //  // Test 5.B
    // i thinks k - Should update - PASSED
    /*winners_i[task_j] = id_k;
    winning_bids_i[task_j] = 5;
    winning_bids_k[task_j] = 10;*/


    //  // Test 5.C
    // i thinks m
    int id_m = 3;
    winners_i[task_j] = id_m; // UNCOMMENT part 2/3 (example)

    // 5.C.1 - Should result in no change because i more confident and has more recent info from m - PASSED
    // UNCOMMENT part 3/3 (example)
    timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;

    // 5.C.2 - Should result in update since k has more recent info - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;*/
    

    robot.log_info("After testResolveConflicts changes things for testing...");
    std::string bla2 = "Current robot " + std::to_string(id_i) + " (winners, winning_bids, timestamps):";
    robot.log_info(bla2);
    utils::logUnorderedMap(winners_i, robot);
    utils::logUnorderedMap(winning_bids_i, robot);
    utils::logUnorderedMap(timestamps_i, robot);
    std::string bla3 = "Other robot " + std::to_string(id_k) + " (winners, winning_bids, timestamps):";
    robot.log_info(bla3);
    utils::logUnorderedMap(winners_k, robot);            
    utils::logUnorderedMap(winning_bids_k, robot);
    utils::logUnorderedMap(timestamps_k, robot);

}

/*void CBBA::printBundle() {
    try {
        // Implement printBundle logic here
    } catch (const std::exception& e) {
        std::cerr << "Error in printBundle: " << e.what() << std::endl;
    }
}

void CBBA::obtainConsensus() {
    try {
        // Implement obtainConsensus logic here
    } catch (const std::exception& e) {
        std::cerr << "Error in obtainConsensus: " << e.what() << std::endl;
    }
}*/