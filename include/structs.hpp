#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>
#include <stdexcept>
#include <iostream>
#include <limits>
#include <string>

// leaving this structs for now to see if we can get a simple version working with just vectors in cbba class

struct WinningBids {
    std::vector<double> winning_bids;
    int numTasks;

    WinningBids(int numTasks) : numTasks(numTasks), winning_bids(init(numTasks)) {}

    static std::vector<double> init(int numTasks) {
        if (numTasks <= 0 || numTasks > std::numeric_limits<std::size_t>::max()) {
            std::cerr << "Error: Invalid number of tasks (" << numTasks << ") for WinningBids vector allocation." << std::endl;
            throw std::length_error("Invalid number of tasks for WinningBids vector allocation.");
        }
        return std::vector<double>(numTasks, 0.0);
    }
};

struct WinningAgentIndices {
    std::vector<int> winning_agent_indices;
    int numTasks;

    WinningAgentIndices(int numTasks) : numTasks(numTasks), winning_agent_indices(init(numTasks)) {}

    static std::vector<int> init(int numTasks) {
        if (numTasks <= 0 || numTasks > std::numeric_limits<std::size_t>::max()) {
            std::cerr << "Error: Invalid number of tasks (" << numTasks << ") for WinningAgentIndices vector allocation." << std::endl;
            throw std::length_error("Invalid number of tasks for WinningAgentIndices vector allocation.");
        }
        return std::vector<int>(numTasks, 0);
    }
};

struct Task {
    int id;
    std::string name;
    Pose2D location;
    double priority;
    double utility;
    double bid;

    Task(int id, const std::string& name, const Pose2D& location, double priority, double utility, double bid)
        : id(id), name(name), location(location), priority(priority), utility(utility), bid(bid) {}

    void print() const {
        std::cout << "Task ID: " << id << std::endl;
        std::cout << "Name: " << name << std::endl;
        std::cout << "Location: (" << location.x << ", " << location.y << ")" << std::endl;
        std::cout << "Priority: " << priority << std::endl;
        std::cout << "Utility: " << utility << std::endl;
        std::cout << "Bid: " << bid << std::endl;
    }
};

struct Path {
    std::vector<Task> tasks;

    void addTask(Task& task, int index) {
        std::cout << "Adding task at index " << index << "..." << std::endl;

        if (index < 0 || index > tasks.size()) {
            std::cerr << "Error: Index (" << index << ") is out of range for adding task." << std::endl;
            throw std::out_of_range("Index is out of range for adding task.");
        }

        tasks.insert(tasks.begin() + index, task);
    }

    void print() {
        for (const auto& task : tasks) {
            task.print();
        }
    }

    Task popNextTask() {
        if (tasks.empty()) {
            std::cerr << "Error: Path is empty." << std::endl;
            throw std::out_of_range("Path is empty");
        }
        Task nextTask = tasks.front();
        tasks.erase(tasks.begin());
        return nextTask;
    }
};

struct Bundle {
    std::vector<Task> tasks;

    void addTask(Task& task) {
        std::cout << "Adding task to bundle..." << std::endl;
        tasks.push_back(task);
    }

    void print() {
        for (const auto& task : tasks) {
            task.print();
        }
    }

    Task popMostRecentTask() {
        if (tasks.empty()) {
            std::cerr << "Error: Bundle is empty." << std::endl;
            throw std::out_of_range("Bundle is empty");
        }
        Task nextTask = tasks.back();
        tasks.pop_back(); // Ensure the task is removed from the bundle
        return nextTask;
    }
};

#endif // STRUCTS_H