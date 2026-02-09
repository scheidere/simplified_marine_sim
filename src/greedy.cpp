#include <cstdio>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "world.hpp"
#include "robot.hpp"
#include "CBBA.hpp"
#include "parser.hpp"
#include "distance.hpp"
#include "greedy.hpp"
#include <utils.hpp>

class Robot;

Greedy::Greedy(Robot& r, World& w) : robot(r), world(w) {
}

/*void Greedy::run() {

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    robot.setTaskAllocStartTime(start_time);

    std::vector<int> assigned_task_order;

    std::vector<int> doable_task_ids = robot.getDoableTaskIDs();
    std::vector<int> available_tasks = doable_task_ids; // Remove tasks from this when they are assigned

    // Loop through all possible tasks based on robot's capabilities
    std::vector<double> costs = {};
    Pose2D current_pose = robot.getPose();

    while (!available_tasks.empty()) { // Until you have assigned all tasks robot is capable of
        int best_task_id = -1; // Reset for each comparison of remaining tasks
        double max_score = -1.0; // Reset also for the same

        for (int task_id : available_tasks) { // Find task that gives highest score
            // double score = getTaskScore(task_id, current_pose.x, current_pose.y);

            auto start_score = std::chrono::high_resolution_clock::now();
            double score = getTaskScore(task_id, current_pose.x, current_pose.y);
            auto end_score = std::chrono::high_resolution_clock::now();
            
            auto duration = end_score - start_score;
            double score_time = std::chrono::duration<double>(duration).count();
            std::string score_log = "getTaskScore for task " + std::to_string(task_id) + " took: " + std::to_string(score_time) + " seconds";
            robot.log_info(score_log);

            if (score > max_score) { // Save max score and id of robot that achieved it
                max_score = score;
                best_task_id = task_id;
            }

        }

        // If found, assign task and remove from available tasks vector
        if (best_task_id != -1) {
            assigned_task_order.push_back(best_task_id); // Assign

            // Save the score for this task
            robot.saveTaskScore(best_task_id, max_score);

            available_tasks.erase( // Make unavailable
                std::remove(available_tasks.begin(), available_tasks.end(), best_task_id),
                available_tasks.end()
            );

            // auto start_erase = std::chrono::high_resolution_clock::now();
            // available_tasks.erase(
            //     std::remove(available_tasks.begin(), available_tasks.end(), best_task_id),
            //     available_tasks.end()
            // );
            // auto end_erase = std::chrono::high_resolution_clock::now();
            // auto duration = end_erase - start_erase;
            // double erase_time = std::chrono::duration<double>(duration).count();
            // std::string erase_log = "Erase operation took: " + std::to_string(erase_time) + " seconds";
            // robot.log_info(erase_log);

            // Update robot's theoretical location to location of assigned task
            std::pair<int, int> current_position = world.getTaskLocation(best_task_id);
            Pose2D new_pose = {current_position.first, current_position.second,0};
            current_pose = new_pose;

        }
    }

    // Now that all tasks robot is capable of are assigned in order of decreasing score, set the path for execution
    std::vector<int>& path = robot.getPath();
    path = assigned_task_order;
    robot.log_info("Path from greedy method: ");
    utils::log1DVector(path, robot);

    // bool& at_consensus = robot.getAtConsensus();
    // at_consensus = true; // this variable is really for CBBA/CBGA, since there is no communication for greedy method, but needs to be true for action execution
    robot.setAtConsensus(true);

    // Log time it took to reach convergence, maintained for the threshold number of rounds
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = end_time - robot.getTaskAllocStartTime();
    double seconds = std::chrono::duration<double>(duration).count();
    std::string time = "Time to convergence: " + std::to_string(seconds) + " seconds";
    robot.log_info(time);

}

double Greedy::getTaskScore(int task_id, int prev_x, int prev_y) {
    // This should be the SAME score/reward logic as CBBA for fair comparison (see CBBA::getPathScore())

    double discount_factor = 0.999;
    
    // Reward for completing task
    int reward = world.getTaskReward(task_id);
    
    // Calculate distance from current position to task
    std::pair<int, int> task_position = world.getTaskLocation(task_id);
    int current_x = task_position.first;
    int current_y = task_position.second;
    double distance_to_task = Distance::getEuclideanDistance(prev_x,prev_y,current_x,current_y);
    
    // Apply SAME discount formula as CBBA
    double score = reward * pow(discount_factor, distance_to_task);
    
    return score;
}*/

void Greedy::run() {

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    robot.setTaskAllocStartTime(start_time);

    std::vector<int> assigned_task_order;

    std::vector<int> doable_task_ids = robot.getDoableTaskIDs();
    std::vector<int> available_tasks = doable_task_ids; // Remove tasks from this when they are assigned

    // Loop through all possible tasks based on robot's capabilities
    std::vector<double> costs = {};
    Pose2D current_pose = robot.getPose();

    // Track which tasks have been completed by the team (to avoid duplicate scoring)
    static std::set<int> team_completed_tasks;

    while (!available_tasks.empty()) { // Until you have assigned all tasks robot is capable of
        int best_task_id = -1; // Reset for each comparison of remaining tasks
        double max_score = -1.0; // Reset also for the same

        for (int task_id : available_tasks) { // Find task that gives highest score
            // double score = getTaskScore(task_id, current_pose.x, current_pose.y);

            auto start_score = std::chrono::high_resolution_clock::now();
            double score = getTaskScore(task_id, current_pose.x, current_pose.y);
            auto end_score = std::chrono::high_resolution_clock::now();
            
            auto duration = end_score - start_score;
            double score_time = std::chrono::duration<double>(duration).count();
            std::string score_log = "getTaskScore for task " + std::to_string(task_id) + " took: " + std::to_string(score_time) + " seconds";
            robot.log_info(score_log);

            if (score > max_score) { // Save max score and id of robot that achieved it
                max_score = score;
                best_task_id = task_id;
            }

        }

        // If found, assign task and remove from available tasks vector
        if (best_task_id != -1) {
            assigned_task_order.push_back(best_task_id); // Assign

            // Save the score for this task only if first robot to complete it
            if (team_completed_tasks.find(best_task_id) == team_completed_tasks.end()) {
                robot.saveTaskScore(best_task_id, max_score);
                team_completed_tasks.insert(best_task_id);
            }

            available_tasks.erase( // Make unavailable
                std::remove(available_tasks.begin(), available_tasks.end(), best_task_id),
                available_tasks.end()
            );

            // auto start_erase = std::chrono::high_resolution_clock::now();
            // available_tasks.erase(
            //     std::remove(available_tasks.begin(), available_tasks.end(), best_task_id),
            //     available_tasks.end()
            // );
            // auto end_erase = std::chrono::high_resolution_clock::now();
            // auto duration = end_erase - start_erase;
            // double erase_time = std::chrono::duration<double>(duration).count();
            // std::string erase_log = "Erase operation took: " + std::to_string(erase_time) + " seconds";
            // robot.log_info(erase_log);

            // Update robot's theoretical location to location of assigned task
            std::pair<int, int> current_position = world.getTaskLocation(best_task_id);
            Pose2D new_pose = {current_position.first, current_position.second,0};
            current_pose = new_pose;

        }
    }

    // Now that all tasks robot is capable of are assigned in order of decreasing score, set the path for execution
    std::vector<int>& path = robot.getPath();
    path = assigned_task_order;
    robot.log_info("Path from greedy method: ");
    utils::log1DVector(path, robot);

    // bool& at_consensus = robot.getAtConsensus();
    // at_consensus = true; // this variable is really for CBBA/CBGA, since there is no communication for greedy method, but needs to be true for action execution
    robot.setAtConsensus(true);

    // Log time it took to reach convergence, maintained for the threshold number of rounds
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = end_time - robot.getTaskAllocStartTime();
    double seconds = std::chrono::duration<double>(duration).count();
    std::string time = "Time to convergence: " + std::to_string(seconds) + " seconds";
    robot.log_info(time);

}

double Greedy::getTaskScore(int task_id, int prev_x, int prev_y) {
    // This should be the SAME score/reward logic as CBBA for fair comparison (see CBBA::getPathScore())

    double discount_factor = 0.999;
    
    // Reward for completing task
    int reward = world.getTaskReward(task_id);
    
    // Calculate distance from current position to task
    std::pair<int, int> task_position = world.getTaskLocation(task_id);
    int current_x = task_position.first;
    int current_y = task_position.second;
    double distance_to_task = Distance::getEuclideanDistance(prev_x,prev_y,current_x,current_y);
    
    // Apply SAME discount formula as CBBA
    double score = reward * pow(discount_factor, distance_to_task);
    
    return score;
}