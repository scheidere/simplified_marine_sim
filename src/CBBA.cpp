#include <cstdio>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "world.hpp"
#include "robot.hpp"
#include "CBBA.hpp"
#include "parser.hpp"

class Robot;

CBBA::CBBA(JSONParser& parser) : parser(parser) {
    init();
}

void CBBA::init() {

    // Parse at all
    json j = parser.parse();
    //std::cout << "Printing parsed json from CBBA init" << std::endl;
    //std::cout << j.dump(4) << std::endl;
    //std::cin.get();

    num_agents = parser.getNumAgents();
    //std::cout << num_agents << std::endl;

    num_tasks = parser.getNumLocalTasks();

    max_depth = parser.getMaxDepth(); //j["cbba"]["max_depth"];
    //std::cout << max_depth << std::endl;

    std::vector<int> agent_indices = parser.getAgentIndices();
    std::vector<std::string> agent_types = parser.getAgentTypes();
    for (auto& t: agent_types) {
        std::cout << t << std::endl;
    }

    std::vector<std::string> task_types = parser.getTaskTypes();
    for (auto& t: task_types) {
        std::cout << t << std::endl;
    }

    std::cin.get();
}

double CBBA::calculatePathUtility(Robot& robot, Path path) {
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
}

void CBBA::buildBundle(World& world, Robot& robot) {
    try {
        std::cout << "in CBBA::buildBundle..." << std::endl;

        // Example code for building bundle
        // std::vector<Task> allTasks = world.getAllTasks();
        // int numTasks = allTasks.size();
        // WinningBids y_i(numTasks); y_i = robot.getWinningBids();
        // WinningAgentIndices z_i(numTasks); z_i = robot.getWinningAgentIndices();
        // Bundle b_i = robot.getBundle(); Path p_i = robot.getPath();
        // NewWinIndicator h_i(numTasks);

        // while (b_i.tasks.size() < allTasks.size()) {
        //     auto [best_task_J, best_path_index_n, overall_max_score_improvement] = findTaskForMaxScoreImprovement(world, robot, allTasks, b_i, p_i, h_i, y_i);
        //     int j = world.getTaskIndex(best_task_J);
        //     b_i.addTask(best_task_J);
        //     p_i.addTask(best_task_J, best_path_index_n);
        //     y_i.winning_bids[j] = overall_max_score_improvement;
        //     z_i.winning_agent_indices[j] = robot.getID();
        // }

        std::string log_msg = "Building bundle for robot " + std::to_string(robot.getID()) + "...";
        robot.log(log_msg);

        std::cout << "at end of CBBA::buildBundle..." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error in buildBundle: " << e.what() << std::endl;
    }
}

void CBBA::printBundle() {
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
}