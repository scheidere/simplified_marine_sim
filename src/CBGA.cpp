#include <cstdio>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "world.hpp"
#include "robot.hpp"
#include "CBGA.hpp"
#include "parser.hpp"
#include "distance.hpp"
#include <utils.hpp>

class Robot;

CBGA::CBGA(Robot& r, World& w, JSONParser& p) : robot(r), world(w), parser(p) {
    init();
}

void CBGA::init() {

    max_depth = parser.getMaxDepth(); // Currently, this is only thing CBGA parses directly (the rest robot should have)
    convergence_threshold = parser.getConvergenceThreshold();
    std::string blorg = "CBGA init convergence_threshold: " + std::to_string(convergence_threshold);
    robot.log_info(blorg);
    std::string blorgl = "max depth: " + std::to_string(max_depth);
    robot.log_info(blorgl);
    //std::cout << max_depth << std::endl;

}

template <typename T>
void CBGA::print1DVector(const std::vector<T>& vec) {
    for (const auto& elem : vec) {
        std::cout << elem << " ";  // Print each element
    }
    std::cout << std::endl;  // Newline after
    
}

template <typename T>
void CBGA::print2DVector(const std::vector<std::vector<T>>& vec) {
    for (const auto& row : vec) {
        for (const auto& elem : row) {
            std::cout << elem << " ";  // Print each element
        }
        std::cout << std::endl;  // Newline after each row
    }
}

void CBGA::testGetTaskIndex() {

    // Passed (idx of 2 is 1)

    std::vector<int> testv = {1,2,3};
    utils::log1DVector(testv, robot);
    int idx = getTaskIndex(2, testv);
    std::string bla = "Task index of 2 in the above vector is: " + std::to_string(idx);
    robot.log_info(bla);

}

void CBGA::testBundleAdd(std::map<int, double>& bids) {

    robot.log_info("Testing bundleAdd...");

    // Getting references so we actual update robot's objects
    std::vector<int>& bundle = robot.getBundle();
    std::vector<int>& path = robot.getPath();
    std::vector<std::vector<double>>& winning_bids_matrix = robot.getWinningBidsMatrix();
    
    // Testing co-op (group_size > 1) case where group already fully assigned, so only assign if better than worst
    // For this test we choose values for the fully assigned task 5 such that the bids calculated will be higher than lowest
    // Consider robot 1 and must have robots 2 and 3 specified in input or functions like getAssignedAgents will break things
    // This test will need to be adapted if/when input changed later (if testing needed again in this form)
    bundle = {1,2,-1};
    path = {1,2,-1};
    winning_bids_matrix = {
        //  R1    R2    R3
        { 6.0,  0.0,  0.0},  // Task 1, solo
        { 4.0,  0.0,  0.0},  // Task 2, solo 
        { 0.0,  0.0,  0.0},  // Task 3, solo
        { 0.0,  0.0,  0.0},  // Task 4 (no winners), solo
        { 0.0,  2.0,  3.0}   // Task 5 (fully assigned at 2 any-type), co-op (input.json must match, i.e., "any-agent_type" = 2)
    };

    utils::log1DVector(bundle,robot);
    utils::log1DVector(path,robot);
    utils::log2DVector(winning_bids_matrix, robot);

    bundleAdd(bundle, path, bids, winning_bids_matrix);

    robot.log_info("After testing bundleAdd:");
    utils::log1DVector(bundle,robot);
    utils::log1DVector(path,robot);
    utils::log2DVector(winning_bids_matrix, robot);

}


void CBGA::buildBundle() {

    // not yet tested
    try {

        std::cout << "in CBGA::buildBundle..." << std::endl;

        // Set at_consensus to false for given robot regardless of if build bundle ultimately changes path, we don't know yet
        robot.setAtConsensus(false); // Stops iterative path execution while potential path changes occur (current executing action should complete)
        robot.log_info("Setting at_consensus to FALSE at start of buildBundle");
        // This is only set to true in checkConvergence when convergence threshold met

        // Let's test the individual functions used in bundleRemove()
        //testGetTaskIndex(); // Passed
        //testRemovePathGaps();
        //testBundleRemove();

        robot.log_info("Timestamps BEFORE change in CBGA::buildBundle:");
        utils::logUnorderedMap(robot.getTimestamps(),robot);

        std::vector<int>& bundle = robot.getBundle();
        std::vector<int>& path = robot.getPath();
        std::vector<std::vector<double>>& winning_bids_matrix = robot.getWinningBidsMatrix();

        robot.log_info("BEFORE bundleRemove...");
        utils::log1DVector(bundle,robot);
        utils::log1DVector(path,robot);
        utils::log2DVector(winning_bids_matrix, robot);

        bundleRemove(bundle, path, winning_bids_matrix); // Update bundle if new info reveals mistakes

        // Initialize bids map to track current robot's bids for this round of CBGA
        // This should accomodate for changes that will occur with faults or other newly available co-op tasks
        // std::map<int, double>& bids = robot.getBids();
        // bids = robot.initBids();
        robot.resetBids(); // this and below line are new way to resetBids that should avoid seg fault (actually don't think it was related)
        std::map<int, double>& bids = robot.getBids();
        
        robot.log_info("Bids before bundle add should be reset, here they are:");
        utils::logMap(bids, robot);

        // Testing bundleAdd group_size > 1 but task fully assigned case where "group_info": {"any_agent_type": 2} for task 5
        testBundleAdd(bids);

        // BUNDLE ADD CAUSING SEG FAULT
        //bundleAdd(bundle, path, bids, winning_bids_matrix); // Populate bundle if any empty slots



        /*
        std::string log_msg = "Building bundle for robot " + std::to_string(robot.getID()) + "...";
        robot.log_info(log_msg);
        std::string b = "Bundle: ";
        robot.log_info(b);
        utils::log1DVector(robot.getBundle(), robot);
        std::string p = "Path: ";
        robot.log_info(p);
        utils::log1DVector(robot.getPath(), robot);

        robot.log_info("Timestamps AFTER change in CBGA::buildBundle:");
        utils::logUnorderedMap(robot.getTimestamps(),robot);
*/
        /*
        Notes:
        1. Path order logic for ties: Since getTestPaths() inserts in a certain way (the new task ID first at left, then each between element, then the end/right),
        the lowest task ID won't always be prioritized to the left of an equally performing higher task ID in the path.
        For example, the path may be 2 3 1 (and 1 and 3 affect the path the same way) instead of 2 1 3... 
        In this example, these paths perform the same, and during bundle building, task 2 is chosen, then task 1 (path is 2 1 -1 at this point), then task 3...
        When task 3 is added, it first looks at test paths 3 2 1, then 2 3 1, then 2 1 3. Since the final two have the same score, 
        the first test path to win is chosen: 2 3 1.
        */

        /*std::cout << "at end of CBGA::buildBundle..." << std::endl;*/
    } catch (const std::exception& e) {
        std::cerr << "Error in buildBundle: " << e.what() << std::endl;
    }
}

int CBGA::getTaskIndex(int task_id, std::vector<int>& vec) {
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

void CBGA::removeGaps(std::vector<int>& vec) {
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

void CBGA::testRemoveGaps() {

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

bool CBGA::isFeasible(int task_id, bool do_test_3) {

    if (do_test_3) {
        // Without changes it's {1,2,3,4} for the other tests
        // Let's make it 3,4 so 1 and 2 must be removed from bundles
        std::vector<int> test_doable_task_ids = {3,4};
        return std::find(test_doable_task_ids.begin(), test_doable_task_ids.end(), task_id) != test_doable_task_ids.end();
    }

    std::vector<int> doable_task_ids = robot.getDoableTaskIDs(); // We will remove tasks from this elsewhere when they are found to become infeasible
    return std::find(doable_task_ids.begin(), doable_task_ids.end(), task_id) != doable_task_ids.end();
}

void CBGA::bundleRemove(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        std::vector<std::vector<double>>& winning_bids_matrix,
                        bool do_test_3) {

    robot.log_info("In bundleRemove...");
    utils::log1DVector(bundle,robot);
    utils::log1DVector(path,robot);
    robot.log_info("winning_bids_matrix: ");
    utils::log2DVector(winning_bids_matrix, robot);

    // below here not updated to deal with winning bids matrix vs winning_bids, winners vectors

    for (auto it = bundle.begin(); it != bundle.end(); ++it) {
        int task_id = *it;

        // Skip if task_id is -1 because empty
        if (task_id == -1) {
            continue;
        }

        int current_robot_id = robot.getID(); // self
        int current_robot_idx = current_robot_id - 1; // self - 1 for vector indices
        int task_idx = task_id - 1; // for vector indices

        std::string blor = "task_idx: " + std::to_string(task_idx) + ", current_robot_idx: " + std::to_string(current_robot_idx);
        robot.log_info(blor);

        // Current robot no longer has winning bid for this task, another robot has newly won and taken its place in the group
        bool new_winner_found = (winning_bids_matrix[task_idx][current_robot_idx] == 0); 

        robot.log_info("New winner found: " + std::to_string(new_winner_found) + " winning_bids_matrix[task_idx][current_robot_idx]: " + std::to_string(winning_bids_matrix[task_idx][current_robot_idx]));

        // If robot has lost spot in group or it is in the group still but it is no longer feasible for it to do the task
        if (new_winner_found || !isFeasible(task_id, do_test_3)) {
            *it = -1;  // Remove task id from bundle and replace with -1 to denote empty

            // Find in path and remove
            auto p_idx = getTaskIndex(task_id, path);
            path[p_idx] = -1;

            // Update winners and winning_bids vectors if not already
            // If task feasibility (not loss of spot in group) is result of change, reflect this in winning_bids_matrix
            if (!new_winner_found) {
                robot.log_info("Task not feasible for robot anymore, update winning bid to 0...");
                winning_bids_matrix[task_idx][current_robot_idx] = 0.0;
            }

        }
    }

    removeGaps(bundle);
    removeGaps(path);

    robot.log_info("Final results for bundleRemove (bundle, path, winning_bids_matrix): ");
    utils::log1DVector(bundle,robot);
    utils::log1DVector(path,robot);
    utils::log2DVector(winning_bids_matrix, robot);

}

void CBGA::testBundleRemove() {

    // Tests passed for CBGA update

    // Test 1: Passed with the below (robot 2 ends up with empty bundle and path, robot 1 has no changes)
    /*std::vector<int> bundle = {1,2,3};
    std::vector<int> path = {1,2,3};
    std::vector<std::vector<double>> winning_bids_matrix = {
        //  R1    R2    R3
        {10.5,  0.0,  0.0},  // Task 1, solo
        {12.0,  0.0,  0.0},  // Task 2, solo 
        { 9.8,  0.0,  0.0},  // Task 3, solo
        { 0.0,  0.0,  0.0},  // Task 4 (no winners), solo
        { 0.0,  0.0,  0.0}   // Task 5 (no winners), co-op
    };*/
    
    //std::string bla = "winning_bids_matrix[0][0]: " + std::to_string(winning_bids_matrix[0][0]);
    //robot.log_info(bla);

    // Test 2: Passed (robot 2 ends with 1,-1,-1, robot 1 ends with 2,-1,-1)
    /*std::vector<int> bundle = {1,2,-1};
    std::vector<int> path = {2,1,-1};
    std::vector<std::vector<double>> winning_bids_matrix = {
        //  R1    R2    R3
        { 0.0, 10.5,  0.0},  // Task 1, solo
        {12.0,  0.0,  0.0},  // Task 2, solo 
        { 0.0,  0.0,  0.0},  // Task 3, solo, not relevant for this test
        { 0.0,  0.0,  0.0},  // Task 4 (no winners), solo, not relevant for this test
        { 0.0,  0.0,  0.0}   // Task 5 (no winners), co-op, not relevant for this test
    };*/

    //bundleRemove(bundle,path,winning_bids_matrix); // Call for test 1 or test 2

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
    // Testing change in task feasibility (making it {3,4} instead of {1,2,3,4,5})
    // Note robot 2 will just end up with empty bundle (this is because of winners being all robot 1)
    std::vector<int> bundle = {2,1,3};
    std::vector<int> path = {3,2,1};
    std::vector<std::vector<double>> winning_bids_matrix = { // this is irrelevant since change will be due to infeasibility
        //  R1    R2    R3
        {10.5,  0.0,  0.0},  // Task 1, solo
        {12.0,  0.0,  0.0},  // Task 2, solo 
        { 9.8,  0.0,  0.0},  // Task 3, solo
        { 0.0,  0.0,  0.0},  // Task 4 (no winners), solo
        { 0.0,  0.0,  0.0}   // Task 5 (no winners), co-op
    };

    bundleRemove(bundle,path,winning_bids_matrix, true); // Call for test 3

}

// original
std::unordered_map<int,int> CBGA::initLocalWinIndicatorH() {

    std::unordered_map<int,int> local_win_indicator_h;

    for (auto pair : robot.getBids()) { // traverse doable tasks not in bundle as tracked by bids map
        int task_id = pair.first;
        local_win_indicator_h[task_id] = 0; // Initialize each key with 0
    }

    return local_win_indicator_h;
}

// Moved this function to robot class
// std::vector<int> CBGA::getAssignedAgents(int task_id) {

//     // Gets any robots assigned to task current robot is considering assigning to itself (SO NOT INCLUDING CURRENT)
//     // Basic testing done

//     robot.log_info("start getAssignedAgents()");

//     std::vector<int> assigned_agent_ids;

//     // CONVERT TO INDEX: To accomodate winning bid matrix row 0 being for task 1, etc. (same reason we use a and not a+1 for indexing below)
//     int task_idx = task_id-1; 

//     std::vector<std::vector<double>> winning_bids_matrix = robot.getWinningBidsMatrix();
//     robot.log_info("winning_bids_matrix: ");
//     utils::log2DVector(winning_bids_matrix, robot);

//     // BELOW COMMENTED OUT BIT IS FOR TEST ONLY
//     /*robot.log_info("ADDING agent assignment to co-op task, id 5 (idx 4), for TEST (see robot 1 logs only)");
//     winning_bids_matrix[4][1] = 2.0; // assigning agent 2 (at idx 1)
//     utils::log2DVector(winning_bids_matrix, robot);*/

//     // For each agent with nonzero winning bid for given task
//     for (int a = 0; a < world.getNumAgents(); a++) {
//         int agent_id = a+1; // CONVERT TO AGENT ID, assumed to be +1 since never agent 0, starts at agent 1

//         //robot.log_info(std::to_string(a));
//         std::string blop = "winning bid element for (" + std::to_string(task_idx) + "," + std::to_string(a) + "): " + std::to_string(winning_bids_matrix[task_idx][a]);
//         robot.log_info(blop);
        
//         if (winning_bids_matrix[task_idx][a] != 0) { // if task_id, agent_id has nonzero winning bid, then that agent is assigned to that task

//             assigned_agent_ids.push_back(agent_id);
//         }

//     }

//     robot.log_info("end getAssignedAgents()");
//     return assigned_agent_ids;
// }

std::pair<double, std::unordered_map<int,Pose2D>> CBGA::getFurthestPossibleDistanceInGroup(int task_id, std::unordered_map<int,Pose2D> prev_locations) {

    // Basic testing done

    // Allows for determination of "time" it takes for group assigned to task to fully arrive, estimated by distance
    // Requires id of task to consider and robot's tracking of locations (that are updated as getDistanceAlongPathToTask() runs)
    // Returns distance the furthest group member must travel to get to task location
    // This works for solo "groups" as well (group size = 1), it will automatically be the max

    robot.log_info("in getFurthestPossibleDistanceInGroup()");

    // Largest distance an agent assigned to given task must travel before group can start task
    double max_dist = 0;

    // Get location of task (or approx./start location if a task involves exploration of an area)
    std::pair<int,int> task_location = world.getTaskLocation(task_id); // path[i] is a task id
    int task_x = task_location.first;
    int task_y = task_location.second;
    std::string bla2 = "task loc: " + std::to_string(task_x) + ", " + std::to_string(task_y);
    robot.log_info(bla2);

    // Get assignment group for task (agents already assigned so not including current which is considering assigning itself)
    std::vector<int> assigned_agent_ids = robot.getAssignedAgents(task_id);
    robot.log_info("assigned_agent_ids: ");
    utils::log1DVector(assigned_agent_ids, robot);

    std::vector<int> potential_new_group_ids = assigned_agent_ids;
    potential_new_group_ids.push_back(robot.getID()); // Adding current robot's id, as if it were assigned (order doesn't matter)
    robot.log_info("potential_new_group_ids: ");
    utils::log1DVector(potential_new_group_ids, robot);

    // final log before seg fault is above this
    // robot.log_info("test print");
    // std::cout << "test print" << std::endl;
    
    for (auto& assigned_agent_id : potential_new_group_ids) { // for agent in group for task

        std::cout << "Processing agent ID: " << assigned_agent_id << std::endl;
        robot.log_info("About to access prev_locations for agent " + std::to_string(assigned_agent_id));
        
        // Check if the key exists first
        if (prev_locations.find(assigned_agent_id) == prev_locations.end()) {
            robot.log_info("ERROR: Agent ID " + std::to_string(assigned_agent_id) + " not found in prev_locations!");
            robot.log_info("Available agent IDs in prev_locations:");
            for (const auto& pair : prev_locations) {
                robot.log_info("  Agent ID: " + std::to_string(pair.first));
            }
            return {0.0, prev_locations}; // Safe return
        }
        
        std::cout << "Agent exists in map, about to access..." << std::endl;
        Pose2D agent_loc = prev_locations[assigned_agent_id];
        std::cout << "Successfully got agent location: (" << agent_loc.x << "," << agent_loc.y << ")" << std::endl;

        // Get location from locations vector (current robot's belief on where all robots are)
        //Pose2D agent_loc = prev_locations[assigned_agent_id];

        // Get distance between agent location and task location
        double dist = Distance::getEuclideanDistance(agent_loc.x,agent_loc.y,task_x,task_y);
        std::string b = "distance for " + std::to_string(assigned_agent_id) + " is: ";
        robot.log_info(b);
        robot.log_info(std::to_string(dist));
        std::cout << b << dist << std::endl;        

        // Update agents tracked location (for calculation only, not in world)
        prev_locations[assigned_agent_id] = {task_x,task_y,0}; // Agent is now counted as having arrived at this task

        if (dist > max_dist) {
            max_dist = dist;
        }
    }

    robot.log_info("end getFurthestPossibleDistanceInGroup()");
    std::pair<double, std::unordered_map<int,Pose2D>> max_dist_and_new_locs = {max_dist, prev_locations};

    return max_dist_and_new_locs;
}

double CBGA::getDistanceAlongPathToTask(std::vector<int> path, int task_id) {

    // UPDATING FOR CBGA
    // For each cooperative task considered, use distance of furthest agent assigned to it
    // Basic testing done

    // Get *maximum* distance along path to location of given task_id

    robot.log_info("in cumulative distance function");

    if (std::find(path.begin(), path.end(), task_id) == path.end()) {
        throw std::runtime_error("Task ID " + std::to_string(task_id) + " not found in path.");
    }

    double distance = 0;
    std::unordered_map<int,Pose2D> prev_locations = robot.getLocations();

    // For each task in bundle/path (by definition the same tasks), in order of appearance in path
    for (int i = 0; i < getBundleOrPathSize(path); i++) {

        int task_id_itr = path[i];

       std::pair<double, std::unordered_map<int,Pose2D>> max_dist_and_new_locs = getFurthestPossibleDistanceInGroup(task_id_itr, prev_locations);

       double max_distance = max_dist_and_new_locs.first;
       std::string s = "max_dist: " + std::to_string(max_distance);
       robot.log_info(s);
       std::unordered_map<int,Pose2D> new_locations = max_dist_and_new_locs.second;
       robot.log_info("new_locations: ");
       utils::logUnorderedMap(new_locations, robot);

        distance += max_distance;

        if (task_id_itr == task_id) {
            std::string s2 = "distance at end: " + std::to_string(distance);
            robot.log_info(s2);
            robot.log_info("end cumulative distance function");
            return distance; // return distance once you have calculated distance to the given task via the path
        } else {
            prev_locations = new_locations;
        }
    }

    // Should never reach here due to check above
    throw std::logic_error("Unexpected error in getDistanceAlongPathToTask");

}
/*    // OLD VERSION BELOW TO PREVENT ERRORS WHILE ADDING pose update to messaging and neighbor distance tracker to robot
    // Get distance along path to location of given task_id

    robot.log_info("in cumulative distance function");

    if (std::find(path.begin(), path.end(), task_id) == path.end()) {
        throw std::runtime_error("Task ID " + std::to_string(task_id) + " not found in path.");
    }

    double distance = 0;

    // Get robot location to init "prev" location
    Pose2D current_pose = robot.getPose();
    int prev_x = current_pose.x;
    int prev_y = current_pose.y;
    std::string bla = "init prev loc: " + std::to_string(prev_x) + ", " + std::to_string(prev_y);
    robot.log_info(bla);

    for (int i = 0; i < getBundleOrPathSize(path); i++) {

        std::pair<int,int> location = world.getTaskLocation(path[i]); //&robot); // robot only for testing, path[i] is a task id
        int current_x = location.first;
        int current_y = location.second;
        std::string bla2 = "task loc: " + std::to_string(current_x) + ", " + std::to_string(current_y);
        robot.log_info(bla2);


        double new_dist = Distance::getEuclideanDistance(prev_x,prev_y,current_x,current_y);
        robot.log_info(std::to_string(new_dist));

        distance += new_dist;

        if (path[i] == task_id) {
            return distance;
        } else {
            prev_x = current_x;
            prev_y = current_y;
        }
    }

    robot.log_info("end cumulative distance function");

    // Should never reach here due to check above
    throw std::logic_error("Unexpected error in getDistanceAlongPathToTask");

}*/

double CBGA::getPathScore(std::vector<int> path, bool do_test) {

    double discount_factor = 0.999; // % future reward loses value
    double score = 0.0;

    robot.log_info("getPathScore distance check for path:");
    utils::log1DVector(path, robot);
    
    double distance;
    // for each task id in given vector (bundle or path)
    for (int i = 0; i < getBundleOrPathSize(path); i++) {

        int task_id = path[i];

        if (!do_test) {

            robot.log_info("in the right if**********");

            // Get distance along path to given task
            distance = getDistanceAlongPathToTask(path, path[i]);

            // log for testing
            /*std::string bla = "i: " + std::to_string(i);
            robot.log_info(bla);
            std::string bla2 = "Distance: " + std::to_string(distance);
            robot.log_info(bla2);*/

            int reward = world.getTaskReward(task_id);
            //score += reward / (distance + 1.0); // Doesn't weigh distance enough, want to prioritize closer tasks more
            //score += reward / pow(distance+1.0,3); // Still not enough to make diff
            /*double alpha = 5.0/400.0; // max reward / max distance
            double normalized = reward - alpha * distance;
            score += normalized;*/

            score += reward * pow(discount_factor, distance);

        } else {
            if (task_id == 2) {
                score += (i == 0) ? 3.0 : 2.0; // val if true: val if false
            } else {
                score += 1.0; // Assign base reward for other tasks
            }
        }
    }

    return score;
}

std::vector<int> CBGA::addTaskToPath(int task_id, std::vector<int> test_path, int position_n) {
 
    // Pass in path that has a valid empty slot (assume that position n is valid)

    //robot.log_info("in addTaskToPath");

    //utils::log1DVector(test_path, robot);

    test_path[position_n] = task_id;

    //utils::log1DVector(test_path, robot);

    //robot.log_info("end addTaskToPath");

    return test_path;
}

std::vector<int> CBGA::shiftTaskInPath(int i, std::vector<int> path) {

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

int CBGA::findFirstEmptyElement(std::vector<int> path) {

    for (int i=0; i<path.size(); i++) {
        if (path[i]==-1) {
            return i;
        }
    }

    return -1; // Path is full (should not get here because bundle would be full and stop it)
}

std::unordered_map<int, std::vector<int>> CBGA::getTestPaths(int new_task_id) {

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

std::pair<double, int> CBGA::computeBid(int task_id) {
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
    std::string bloop = "Current score: " + std::to_string(current_path_score);
    robot.log_info(bloop);
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

//int CBGA::getBestTaskID(const std::unordered_map<int, double>& bids, const std::unordered_map<int, int>& h) {
int CBGA::getBestTaskID(const std::map<int, double>& bids, const std::unordered_map<int, int>& h, std::vector<int>& bundle) {
    // Get the task id for the task that has the max new winning bid, J
    // h is local win indicator with elements 0 or 1

    std::cout << "in getBestTaskID" << std::endl;
    robot.log_info("in getBestTaskID");
    robot.log_info("indicator h:");
    utils::logUnorderedMap(h, robot);

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

    robot.log_info("end getBestTaskID");
    return ID_of_max;

}

void CBGA::addTaskToBundleEnd(std::vector<int>& bundle, int task_id) {

    /*static volatile int canary = 0x12345678;

    // Before any major operation:
    if (canary != 0x12345678) {
        std::cout << "MEMORY CORRUPTION DETECTED!" << std::endl;
        abort();
    }*/

    std::cout << "in addTaskToBundleEnd" << std::endl;
    robot.log_info("in addTaskToBundleEnd");

    auto it = std::find(bundle.begin(), bundle.end(), -1); // Find first occurrence of -1
    if (it != bundle.end()) {
        *it = task_id; // Replace -1 with the new task ID
    }
}

int CBGA::getBundleOrPathSize(const std::vector<int>& vec) {
    return std::count_if(vec.begin(), vec.end(), [](int task_id) {
        return task_id != -1;
    });
}

void CBGA::updatePath(std::vector<int>& path, int n, int new_task_id) {

    if (new_task_id==-1) { // only if the new task exists
        robot.log_info("FOUND new task id of -1 in updatePath");
    }
    else {
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

}

double CBGA::getSoloWinningBid(std::vector<std::vector<double>>& winning_bids_matrix, int task_id) {

    // ONLY for tasks with group_size = 1, solo tasks, where agent is assigned
    // Previously this value was stored in the unordered map winning_bids, with the task_id as the key

    double solo_winning_bid = 0.0; // if no agent assigned yet, stays 0

    int task_idx = task_id-1; // Need conversion bc matrix is 2D vector (unlike bids which is unordered map so can directly use task_id)
    int count = 0; // Make sure only one winner for solo task
    for (auto& bid : winning_bids_matrix[task_idx]) {
        if (bid > solo_winning_bid) {
            solo_winning_bid = bid;
            count += 1;
        }
    }

    if (count > 1) {
        robot.log_info("ERROR in CBGA.getSoloWinningBid: more than one winner counted for solo task!");
    } 

    return solo_winning_bid;
}


/*bool CBGA::isGroupEffectivelyFull(std::unordered_map<std::string, int> task_group_fullness) {

    // not yet tested
    // set up to work with getTaskGroupFullnessbyType

    bool full_for_current_robot_type = false;

    std::string current_type = robot.getType(); // Current robot type

    std::unordered_map<std::string, int>& task_group_max_size = world->getTaskGroupInfo(task_id);

    if (task_group_fullness[current_type] == task_group_max_size[current_type] &&
        task_group_fullness["any_agent_type"] == task_group_max_size["any_agent_type"]) {
        // Both potential options for assigning robot of current type are full
        full_for_current_robot_type = true;
    } else if (task_group_fullness[current_type] > task_group_max_size[current_type] ||
        task_group_fullness["any_agent_type"] > task_group_max_size["any_agent_type"]) {
        robot.log_info("ERROR found in CBGA.isGroupEffectivelyFull: over-assignment");
    }

    return full_for_current_robot_type;
}*/

bool CBGA::isGroupEffectivelyFull(std::unordered_map<std::string, std::vector<int>> task_sub_group_assignments, std::unordered_map<std::string, int>& task_group_max_size, std::string current_robot_type) {

    // not yet tested

    robot.log_info("in isGroupEffectivelyFull()...");

    bool full_for_current_robot_type = false;

    robot.log_info("current_robot_type: " + current_robot_type);
    robot.log_info("task_sub_group_assignments[current_robot_type].size(): " + std::to_string(task_sub_group_assignments[current_robot_type].size()));
    robot.log_info("task_group_max_size[current_robot_type]: " + std::to_string(task_group_max_size[current_robot_type]));
    robot.log_info("task_sub_group_assignments[any_agent_type].size(): " + std::to_string(task_sub_group_assignments["any_agent_type"].size()));
    robot.log_info("task_group_max_size[any_agent_type]: " + std::to_string(task_group_max_size["any_agent_type"]));

    if (task_sub_group_assignments[current_robot_type].size() == task_group_max_size[current_robot_type] &&
        task_sub_group_assignments["any_agent_type"].size() == task_group_max_size["any_agent_type"]) {
        // Both potential options for assigning robot of current type are full
        robot.log_info("IS CONSIDERED FULL");
        full_for_current_robot_type = true;
    } else if (task_sub_group_assignments[current_robot_type].size() > task_group_max_size[current_robot_type] ||
        task_sub_group_assignments["any_agent_type"].size() > task_group_max_size["any_agent_type"]) {
        robot.log_info("ERROR found in CBGA.isGroupEffectivelyFull: over-assignment");
    }

    if (!full_for_current_robot_type) {
        robot.log_info("IS NOT CONSIDERED FULL");
    }

    return full_for_current_robot_type;
}

std::vector<int> CBGA::getRelevantAssignedIDs(std::unordered_map<std::string, std::vector<int>> task_sub_group_assignments, std::string current_robot_type) {

    std::vector<int> relevant_assigned_ids;

    for (auto& agent_id : task_sub_group_assignments[current_robot_type]) {
        relevant_assigned_ids.push_back(agent_id);
    }

    for (auto& agent_id : task_sub_group_assignments["any_agent_type"]) {
        relevant_assigned_ids.push_back(agent_id);
    }

    return relevant_assigned_ids;
}

/*below feels redundant, maybe should just populate relevant_assigned_ids in robot.getTaskGroupFullnessbyType??
what do we want relevant_assigned_ids to contain, does it change if type sub-group full vs not?
std::pair<bool, std::vector<int>> CBGA::getRelevantGroupFullnessInfo(td::unordered_map<std::string, int> task_group_fullness, int task_id) {

    std::pair<bool, std::vector<int>> relevant_fullness_info;
    std::unordered_map<std::string, int>& task_group_max_size = world->getTaskGroupInfo(task_id);
    winning_bids_matrix

    bool full_wrt_current_robot_type = false;
    std::vector<int> relevant_assigned_ids;

    for id of each agent 

    std::pair<bool, std::vector<int>> relevant_fullness_info = std::make_pair(full_wrt_current_robot_type, relevant_assigned_ids);
    return relevant_fullness_info
}*/

/*std::vector<int> getAssignedAgentsFromFullGroup(std::vector<std::vector<double>>& winning_bids_matrix) {

    // This group may not be 100% full, but it is full wrt the current robot's ability to assign itself
    // This function returns the ids of the agents already filling the sub-groups the current robot *could* assign itself to
    // I.e., same type sub-group and "any" type sub-group

    std:vector<int> agent_ids;



    return agent_ids;
}*/

void CBGA::bundleAdd(std::vector<int>& bundle, 
                        std::vector<int>& path, 
                        std::map<int, double>& bids,
                        std::vector<std::vector<double>>& winning_bids_matrix) {
    try {

        // not yet tested with matrix updates for CBGA etc.

        robot.log_info("++++++++++in bundleAdd START++++++++++");
        robot.log_info("Bundle:");
        utils::log1DVector(bundle, robot);
        robot.log_info("Path:");
        utils::log1DVector(path, robot);
        robot.log_info("Winning bids matrix:");
        utils::log2DVector(winning_bids_matrix, robot);
        robot.log_info("+++++++++++++++++++++++++++++++++++++");

        int size = getBundleOrPathSize(bundle);

        std::vector<int> test = robot.getDoableTaskIDs();

        std::string current_robot_type = robot.getType();

        //std::unordered_map<int, int> local_win_indicator_h = initLocalWinIndicatorH(); // Initialize here once, don't need to create keys
    
        // Check if bundle is full (i.e., at max_depth) 
        while (getBundleOrPathSize(bundle) < max_depth) { // continues until full or stops via check that no new task found to add

            /*robot.log_info("++++ While loop start ++++");
            robot.log_info("Bundle:");
            utils::log1DVector(bundle, robot);
            robot.log_info("Path:");
            utils::log1DVector(path, robot);
            robot.log_info("Bids:");
            utils::logMap(bids, robot); // bids is map, not unordered
            robot.log_info("++++  ++  ++++");*/

            // Re-init with new method
            /*local_win_indicator_h.clear();
            for (const auto& pair : bids) {
                local_win_indicator_h[pair.first] = 0;
            }*/

            std::unordered_map<int, int> local_win_indicator_h = initLocalWinIndicatorH();

            robot.log_info("local_win_indicator_h: ");
            utils::logUnorderedMap(local_win_indicator_h, robot);

            std::unordered_map<int,int> best_position_n_tracker; // keys are task id's and values are best index n in path

            // init flags for full co-op group case where current robot has lower winning bid than lowest current winner
            bool coop_group_full_and_new_winner = false;
            int winning_agent_idx_to_clear = -1;

            for (auto task_id : robot.getDoableTaskIDs()) {
                std::string bla = "Looking at doable task " + std::to_string(task_id);
                robot.log_info(bla);

                std::unordered_map<std::string, int>& task_group_max_size = world.getTaskGroupInfo(task_id);
                robot.log_info("task_group_max_size: ");
                utils::logUnorderedMap(task_group_max_size, robot);

                // Get total group_size for current task
                int full_group_size = world.getTaskGroupSize(task_id);
                std::string bloop = "full_group_size: " + std::to_string(full_group_size);
                robot.log_info(bloop);

                if ( std::find(bundle.begin(), bundle.end(), task_id) == bundle.end() ) { // for each doable task not in bundle already
                    robot.log_info("Bundle currently is:");
                    utils::log1DVector(bundle,robot);

                    std::string str1 = "Looking at the following doable task not in bundle:" + std::to_string(task_id);
                    //std::string str1 = "-----Potential next task: " + std::to_string(task_id);
                    robot.log_info(str1);

                    // Compute bid and associated position in path
                    std::pair<double,int> bid_and_best_path_position = computeBid(task_id);
                    bids[task_id] = bid_and_best_path_position.first; best_position_n_tracker[task_id] = bid_and_best_path_position.second;

                    robot.log_info("After computeBid update, bids: ");
                    utils::logMap(bids, robot);
                    robot.log_info("and best position tracker: ");
                    utils::logUnorderedMap(best_position_n_tracker, robot);

                    if (full_group_size == 1) { // task is solo, revert to CBBA logic
                        std::string bop = "Task " + std::to_string(task_id) + " is solo";
                        robot.log_info(bop);

                        // Get winning bid (only one in task row of matrix since group_size = 1 for solo task)
                        double current_winning_bid = getSoloWinningBid(winning_bids_matrix, task_id);
                        std::string ack = "current_winning_bid: " + std::to_string(current_winning_bid);
                        robot.log_info(ack);

                        if ( bids[task_id] - current_winning_bid > epsilon ) {
                    //         // Found better bid than current solo winner so track in local win indicator h
                            std::string log_msg1 = "For task " + std::to_string(task_id) + " bid (" + std::to_string(bids[task_id]) + ") > winning bid(" + std::to_string(current_winning_bid) + ")";
                            robot.log_info(log_msg1);
                            local_win_indicator_h[task_id] = 1;
                        } else {
                            std::string log_msg1 = "For task " + std::to_string(task_id) + "bid (" + std::to_string(bids[task_id]) + ") NOT > winning bid(" + std::to_string(current_winning_bid) + ")";
                            robot.log_info(log_msg1);
                        }

                    } else { // group_size > 1, task is co-op, so do CBGA
                        std::string bop2 = "Task " + std::to_string(task_id) + " is co-op";
                        robot.log_info(bop2);

                        // Count how many robots of each type are assigned (prioritizing explicit type sub-groups of "any_agent_type" sub-group)
                        // std::unordered_map<std::string, int> task_group_fullness = robot.getTaskGroupFullnessbyType(task_id);

                        // Divide assigned robots by type, filling their type subgroups before "any_agent_type" sub-group
                        std::unordered_map<std::string, std::vector<int>> task_sub_group_assignments =  robot.trackAssignedRobotsbySubGroup(task_id);

                        robot.log_info("task_sub_group_assignments: ");
                        utils::logMapOfVectors(task_sub_group_assignments, robot);

                        // True if no room to assign robot to task (type sub-group and "any agent type" sub-group both full)
                        bool full_wrt_current_robot_type = isGroupEffectivelyFull(task_sub_group_assignments, task_group_max_size, current_robot_type);

                        std::string hep = "Group full? " + std::to_string(full_wrt_current_robot_type);
                        robot.log_info(hep);

                        /*bool full_wrt_current_robot_type = false;
                        if type sub-group full:
                            // Check if any type sub-group also full, and if so
                            full_wrt_current_robot_type = true;
                            // Also get l
                        */
                        // // Examine group fullness wrt current robot's type 
                        // std::pair<bool, std::vector<int>> relevant_fullness_info = getRelevantGroupFullnessInfo(td::unordered_map<std::string, int> task_group_fullness, int task_id);

                        // // Given the current robot's type, is there room to assign it to this group 
                        // // first wrt own type, and if full then wrt "any_agent_type" sub-group?
                        // bool full_wrt_current_robot_type = relevant_fullness_info.first;

                        // // IDs of robots assigned to current robot type sub-group and "any_agent_type" sub-group
                        // std::vector<int> relevant_assigned_ids = relevant_fullness_info.second;

                        // Given the current robot's type, is there room to assign it to this group 
                        // first wrt own type, and if full then wrt "any_agent_type" sub-group? If not, below is true.
                        // bool full_wrt_current_robot_type = isGroupEffectivelyFull(task_group_fullness);

                        // group not full wrt current robot type (either by type OR any-type sub-group)
                        if (!full_wrt_current_robot_type) {
                            std::string current_robot_type = robot.getType();
                            std::string bop3 = "Group not full for current robot type sub-group (" + std::to_string(task_sub_group_assignments[current_robot_type].size()) + ") OR/AND any-type sub-group (" + std::to_string(task_sub_group_assignments["any_agent_type"].size()) + ")";
                            robot.log_info(bop3);

                            // There is space, so assign (automatically win)
                            local_win_indicator_h[task_id] = 1;
                        } else { // full for current robot (type sub-group and any-type sub-group)
                            // Only assign if it has a higher bid than another robot of the same type of or robot type counted toward any-type group
                            // need to figure out how to differentiate between agents assigned as part of their type sub-group and part of the any-type
                            // need to count for the type sub-groups and *fill* those before counting any-type fullness

                            std::string glorp = "Group full, so checking for replacement given higher bid than lowest assigned";
                            robot.log_info(glorp);

                            // Get ids of assigned agents that current robot could replace (same type or any type sub-groups)
                            std::vector<int> relevant_assigned_ids = getRelevantAssignedIDs(task_sub_group_assignments, current_robot_type);

                            // Find the LOWEST winning bid among relevant assigned agents
                            double prev_lowest_winning_bid = std::numeric_limits<double>::max();
                            int prev_lowest_bid_winner_idx = -1;
                            for (auto& assigned_agent_id : relevant_assigned_ids) {
                                int task_idx = task_id-1; 
                                int assigned_agent_idx = assigned_agent_id-1;
                                double winning_bid = winning_bids_matrix[task_idx][assigned_agent_idx];
                                
                                if (winning_bid < prev_lowest_winning_bid) {
                                    prev_lowest_winning_bid = winning_bid;
                                    prev_lowest_bid_winner_idx = assigned_agent_idx;
                                }
                            }
                            
                            robot.log_info("Lowest assigned bid: " + std::to_string(prev_lowest_winning_bid) + 
                                           " from agent " + std::to_string(prev_lowest_bid_winner_idx + 1));
                            robot.log_info("Current bid: " + std::to_string(bids[task_id]));
                            
                            // Only win if current bid beats the lowest winning bid
                            if (bids[task_id] - prev_lowest_winning_bid > epsilon) {
                                robot.log_info("Current bid beats lowest winning bid - marking as winner");
                                coop_group_full_and_new_winner = true;
                                winning_agent_idx_to_clear = prev_lowest_bid_winner_idx;
                                local_win_indicator_h[task_id] = 1;
                            } else {
                                robot.log_info("Current bid does NOT beat lowest winning bid - no assignment");
                            }
                        }

                           /* // For each ID of agent assigned to task in same type sub-group as current robot or in "any_agent_type" sub-group if former full
                            for (auto& assigned_agent_id : relevant_assigned_ids) {
                                int task_idx = task_id-1; int agent_idx = assigned_agent_id-1; // Accounts for task ids starting at 1, not 0 like vector indices

                                if (bids[task_id] - winning_bids_matrix[task_idx][agent_idx] > epsilon) {
                                    // then found new winner in current bid
                                    local_win_indicator_h[task_id] = 1;
                                } else { // no win for group with room for robot of current type or group that's full wrt bids
                                    std::string log_msg1 = "For task " + std::to_string(task_id) + "bid (" + std::to_string(bids[task_id]) + ") NOT > winning bid(" + std::to_string(winning_bids_matrix[task_idx][agent_idx]) + ") for relevant sub-groups";
                                    robot.log_info(log_msg1);
                                }

                            }

                        }*/
                    }

                }
            }


            // At the very end of the for loop (after all task processing):
            robot.log_info("Finished processing all tasks in this iteration");
            //std::cout << "CHECKPOINT 3: End of for loop" << std::endl;

            // Right before getBestTaskID:
            //robot.log_info("About to call getBestTaskID");
            //std::cout << "CHECKPOINT 4: Before getBestTaskID" << std::endl;
            int J = getBestTaskID(bids, local_win_indicator_h, bundle); // Does commenting this out prevent seg fault?
            //std::cout << "CHECKPOINT 5: After getBestTaskID, J = " << J << std::endl;


            // // Get the task id for the task that has the max new winning bid, J
            // int J = getBestTaskID(bids, local_win_indicator_h, bundle);

            if (J == -1) {
                robot.log_info("No valid task found to add to bundle. Exiting loop.");
                break;
            }

            std::cout << "Best task ID is " << J << std::endl;
            std::string blob = "Best task ID is " + std::to_string(J);
            robot.log_info(blob);
            int n = best_position_n_tracker[J];
            // std::string loggy = "Best position in path is: " + std::to_string(n);
            // robot.log_info(loggy);
            // robot.log_info("before bundle change ===============================");
            // utils::log1DVector(bundle, robot);
            addTaskToBundleEnd(bundle, J); // Insert J at end of bundle
            // robot.log_info("after bundle change ===============================");
            // utils::log1DVector(bundle, robot);
            // robot.log_info("=====");
            // robot.log_info("before path change ++++++++++");
            // utils::log1DVector(path, robot);
            updatePath(path,n,J);
            // robot.log_info("after path change ++++++++++");
            // utils::log1DVector(path, robot);
            // robot.log_info("+++++");

            // std::cout << "Past bundle and path updates..." << std::endl;
            // robot.log_info("Past bundle and path updates...");

            winning_bids_matrix[J-1][robot.getID()-1] = bids[J]; // +1 to convert to index since IDs start at 1 and indices at 0 (bids uses J because it is a map)
            if (coop_group_full_and_new_winner) {
                robot.log_info("Clearing lowest winning bid since added new winning bid");
                winning_bids_matrix[J-1][winning_agent_idx_to_clear] = 0.0;
            }
            // std::cout << "after wb matrix update" << std::endl;
            // robot.log_info("after wb matrix update");
            bids.erase(J); // Remove task now in bundle from future consideration

            robot.log_info("haayyyy");

        }
        robot.log_info("hey");
    
        //std::string log_msg2 = "Bundle is this after bundleAdd:";
        //robot.log_info(log_msg2);
        //utils::log1DVector(bundle, robot);

        robot.log_info("==========in bundleAdd END==========");
        robot.log_info("Bundle:");
        utils::log1DVector(bundle, robot);
        robot.log_info("Path:");
        utils::log1DVector(path, robot);
        robot.log_info("winning_bids_matrix:");
        utils::log2DVector(winning_bids_matrix, robot);
        robot.log_info("====================================");

    } catch (const std::exception& e) {
        std::cerr << "Error in bundleAdd: " << e.what() << std::endl;
    };
}

void CBGA::update(int j, std::unordered_map<int, int>& winners_i, std::unordered_map<int, int> winners_k,
    std::unordered_map<int, double>& winning_bids_i, std::unordered_map<int, double> winning_bids_k) {

    // Precedence given to k, which is determined to have more recent info than i
    // Winners update z_ij = z_kj and winning bids update y_ij = y_kj
    winners_i[j] = winners_k[j]; winning_bids_i[j] = winning_bids_k[j];

}

void CBGA::reset(int j, std::unordered_map<int, int>& winners_i, std::unordered_map<int, double>& winning_bids_i) {

    winners_i[j] = -1; winning_bids_i[j] = 0.0;
}

void CBGA::resolveConflicts(bool do_test) {

    // Should be identical to previous resolveConflicts but without direct timestamps[bla] accessing 
    // to avoid default 0 creation for non-existence keys (i.e., self id i)
    // Have not confirmed it being identical by rerunning all the tests in testResolveConflicts

    int id_i = robot.getID(); 
    std::vector<Msg>& message_queue_i = robot.getMessageQueue();
    std::unordered_map<int, int>& winners_i = robot.getWinners();
    std::unordered_map<int, double>& winning_bids_i = robot.getWinningBids();
    std::unordered_map<int, double>& timestamps_i = robot.getTimestamps();

    /*robot.log_info("Timestamps BEFORE change in CBGA::resolveConflicts:");
    utils::logUnorderedMap(timestamps_i, robot);*/

    std::string blorpl = "id_i in resolveConflicts: " + std::to_string(id_i);
    robot.log_info(blorpl);

    robot.log_info("Current robot's winners, winning_bids and timestamps before resolveConflicts...");
    utils::logUnorderedMap(winners_i, robot);
    utils::logUnorderedMap(winning_bids_i, robot);
    utils::logUnorderedMap(timestamps_i, robot);

    for (auto& msg : message_queue_i) { // resolve Conflicts with all other robots in comms that have sent messages
        int id_k = msg.id;
        std::unordered_map<int, int>& winners_k = msg.winners;
        std::unordered_map<int, double>& winning_bids_k = msg.winning_bids;
        std::unordered_map<int, double>& timestamps_k = msg.timestamps;

        std::string bla = "Message from robot " + std::to_string(id_k) + ":";
        robot.log_info(bla);
        robot.log_info("Winners: ");
        utils::logUnorderedMap(winners_k, robot);
        robot.log_info("Winning bids: ");
        utils::logUnorderedMap(winning_bids_k, robot);
        robot.log_info("Timestamps: ");
        utils::logUnorderedMap(timestamps_k, robot);

        if (do_test) {
            testResolveConflicts(id_i, message_queue_i, winners_i, winning_bids_i, timestamps_i,
                                 id_k, winners_k, winning_bids_k, timestamps_k);
        }

        for (auto& [j, winner_ij] : winners_i) {
            int winner_kj = winners_k[j];

            auto ts_k_m = [&](int m) {
                return timestamps_k.find(m) != timestamps_k.end() ? timestamps_k.at(m) : -1.0;
            };

            auto ts_i_m = [&](int m) {
                return timestamps_i.find(m) != timestamps_i.end() ? timestamps_i.at(m) : -1.0;
            };

            if (winner_ij == winner_kj) {
                if (winner_ij == id_k) {
                    update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                } else if (winner_ij != -1 && winner_ij != id_i) {
                    int m = winner_ij;
                    if (ts_k_m(m) > ts_i_m(m)) {
                        update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                    }
                }

            } else if (winner_kj == id_k) {
                robot.log_info("winner_kj == id_k: ");
                robot.log_info(std::to_string(id_k));
                if (winner_ij == id_i) {
                    if (winning_bids_k[j] > winning_bids_i[j]) {
                        robot.log_info("k thinks k, i thinks i, k has higher bid so update");
                        update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                    }
                } else if (winner_ij == -1) {
                    robot.log_info("k thinks k, i does not know (-1), so update");
                    update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                } else {
                    int m = winner_ij;
                    std::string blorpl2 = "id_m in resolveConflicts: " + std::to_string(m);
                    robot.log_info(blorpl2);
                    if (ts_k_m(m) > ts_i_m(m) || winning_bids_k[j] > winning_bids_i[j]) {
                        robot.log_info("k thinks k, i thinks m, k timestamp more recent and k bid higher, so update");
                        update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                    }
                }

            } else if (winner_kj == id_i) {
                if (winner_ij == id_k) {
                    reset(j, winners_i, winning_bids_i);
                } else if (winner_ij != -1) {
                    int m = winner_ij;
                    if (ts_k_m(m) > ts_i_m(m)) {
                        reset(j, winners_i, winning_bids_i);
                    }
                }

            } else if (winner_kj != -1) {
                int m = winner_kj;

                if (winner_ij == id_i) {
                    if (ts_k_m(m) > ts_i_m(m) && winning_bids_k[j] > winning_bids_i[j]) {
                        update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                    }
                } else if (winner_ij == id_k) {
                    if (ts_k_m(m) > ts_i_m(m)) {
                        update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                    } else {
                        reset(j, winners_i, winning_bids_i);
                    }
                } else if (winner_ij == -1) {
                    if (ts_k_m(m) > ts_i_m(m)) {
                        update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                    }
                } else {
                    int n = winner_ij;
                    if (ts_k_m(m) > ts_i_m(m) &&
                        (ts_k_m(n) > ts_i_m(n) || winning_bids_k[j] > winning_bids_i[j])) {
                        update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                    } else if (ts_k_m(n) > ts_i_m(n) && ts_i_m(m) > ts_k_m(m)) {
                        reset(j, winners_i, winning_bids_i);
                    }
                }

            } else if (winner_kj == -1) {
                if (winner_ij == id_k) {
                    update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                } else if (winner_ij != id_i && winner_ij != -1) {
                    int m = winner_ij;
                    if (ts_k_m(m) > ts_i_m(m)) {
                        update(j, winners_i, winners_k, winning_bids_i, winning_bids_k);
                    }
                }
            }
        }

        if (true) {
            robot.log_info("Current robot's winners, winning_bids and timestamps after resolveConflicts...");
            utils::logUnorderedMap(winners_i, robot);
            utils::logUnorderedMap(winning_bids_i, robot);
            utils::logUnorderedMap(timestamps_i, robot);
        }
    }

    /*robot.log_info("Timestamps AFTER change in CBGA::resolveConflicts:");
    utils::logUnorderedMap(timestamps_i, robot);
*/
    message_queue_i.clear();
}

/*void CBGA::resolveConflicts(bool do_test) {

    // THIS SHOULD BE IDENTICAL FUNCTIONALLY TO THE ABOVE UNCOMMENTED VERSION EXCEPT:
    // above we do not directly access timestamps[bla] becaues it will default give a value of 0 for keys that don't exist (aka self id i)

    // Given communication between at least two agents, check for conflicts in shared info
    // Option 1: Update agent i's y (winning bids) and z (winners) with agent k's
    // Option 2: Reset agent i's y and z's (to empty/0)
    // Option 3: Leave aka no changes
    // Following table 1
    // ACTION RULE FOR AGENT i (receiver, current) BASED ON COMMUNICATION WITH AGENT k (sender, other) REGARDING TASK j

    // Current robot i
    int id_i = robot.getID(); 
    std::vector<Msg>& message_queue_i = robot.getMessageQueue();
    std::unordered_map<int, int>& winners_i = robot.getWinners();
    std::unordered_map<int, double>& winning_bids_i = robot.getWinningBids();
    std::unordered_map<int,double>& timestamps_i = robot.getTimestamps();

    robot.log_info("Timestamps BEFORE change in CBGA::resolveConflicts:");
    utils::logUnorderedMap(timestamps_i,robot);

    std::string blorp = "Size of message queue: " + std::to_string(message_queue_i.size());
    robot.log_info(blorp);


    for (auto& msg : message_queue_i) {
        int id_k = msg.id;
        std::unordered_map<int, int> winners_k = msg.winners;
        std::unordered_map<int, double> winning_bids_k = msg.winning_bids;
        std::unordered_map<int, double> timestamps_k = msg.timestamps;

        // For reference in log file
        robot.log_info("Current robot's winners, winning_bids and timestamps after resolveConflicts...");
        utils::logUnorderedMap(winners_i, robot);
        utils::logUnorderedMap(winning_bids_i, robot);
        utils::logUnorderedMap(timestamps_i, robot);

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

        // (do_test)
        if (true) {
            robot.log_info("Current robot's winners and winning_bids after resolveConflicts...");
            utils::logUnorderedMap(winners_i, robot);
            utils::logUnorderedMap(winning_bids_i, robot);

        }


    }

    robot.log_info("Timestamps AFTER change in CBGA::resolveConflicts:");
    utils::logUnorderedMap(timestamps_i,robot);

    message_queue_i.clear(); // Remove messages from robot i's queue because they have been processed (info is no longer new)

}*/

// inputs: robot i winners, winning_bids, message_queue (to get msg's from each k), timestamps
void CBGA::testResolveConflicts(int id_i, std::vector<Msg>& message_queue, std::unordered_map<int, int>& winners_i, 
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
    int task_j = 2; //1; // This is random

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
    winners_k[task_j] = id_k; // UNCOMMENT part 1/3

    //  // Test 2.A
    // i thinks i and k thinks k - Result depends on winning bids
    winners_i[task_j] = id_i;

    // 2.A.1 - Should result in update because k winning bid higher than i winning bid - PASSED
    /*winning_bids_i[task_j] = 5;
    winning_bids_k[task_j] = 10;*/

    // 2.A.2 - Should result in no change because i winning bid higher than k winning bid - PASSED
    winning_bids_i[task_j] = 4.37521; // 10;
    winning_bids_k[task_j] =  4.36367; // 5;

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
    //winners_k[task_j] = -1;

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
    /*int id_m = 3;
    winners_i[task_j] = id_m; // UNCOMMENT part 2/3 (example)
*/
    // 5.C.1 - Should result in no change because i more confident and has more recent info from m - PASSED
    // UNCOMMENT part 3/3 (example)
    /*timestamps_i[id_m] = 3;
    timestamps_k[id_m] = 2;*/

    // 5.C.2 - Should result in update since k has more recent info - PASSED
    /*timestamps_i[id_m] = 2;
    timestamps_k[id_m] = 3;
    */

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