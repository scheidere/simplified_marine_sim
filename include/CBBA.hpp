#ifndef CBBA_H
#define CBBA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <planners.hpp>
#include <distance.hpp>

class Robot;

struct Task {
    int id;
    std::string name;
    Pose2D location;
    double priority; // All tasks start with same priority, because world is unknown and all tasks need to be completed
    // Note not all actions are tasks, like charging is an action but not a task
    double utility; // Benefit of completing a task: depends on proximity/ease, priority, 
    double bid; // How willing robot is to do this task

    Task(int id, const std::string& name, const Pose2D& location, double priority, double utility, double bid)
        : id(id), name(name), location(location), priority(priority), utility(utility), bid(bid) {}

    void print() const {
        std::cout << "Task ID: " << id << std::endl;
        std::cout << "Name: " << name << std::endl;
        std::cout << "Location: (" << location.x << ", " << location.y << ")" << std::endl; // Ignoring angle, third term of pose
        std::cout << "Priority: " << priority << std::endl;
        std::cout << "Utility: " << utility << std::endl;
        std::cout << "Bid: " << bid << std::endl;
    }

};

struct Bundle {

    std::vector<Task> bundle;

    void addTask(Task& task) {
        std::cout << "Adding task..." << std::endl;

        auto it = std::upper_bound(bundle.begin(), bundle.end(), task, [](const Task& a, const Task& b) {
            return a.utility > b.utility; // Sorting in descending order of utility (front should be highest utility)
        });
        bundle.insert(it, task);
    }

    void print() {
        for (const auto& task : bundle) {
            task.print();
        }
    }

    Task popNextTask() {
        if (bundle.empty()) {
            throw std::out_of_range("Bundle is empty");
        }
        Task nextTask = bundle.front();
        bundle.erase(bundle.begin());
        return nextTask;
    }

};

// Bundle struct task, utility, anything else?

class CBBA {
private:
    int temp;

public:
    CBBA();

    double createBid(Robot * robot, Task& task); // ???

    double calculateUtility(Robot * robot, Task& task);

    void buildBundle(Robot * robot);

    void printBundle();

    void obtainConsensus();


};

#endif
