#ifndef WORLD_H
#define WORLD_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "message.hpp"

struct Msg;
class Robot;

class World {
protected:
    int X; int Y;
    Distance* distance;
    SensorModel* sensor_model;
    cv::Mat image;
    std::unordered_map<int, Robot*> robot_tracker; // id, instance
    mutable std::mutex world_mutex; // mutex for MR access to robot tracker (and downstream), mutable for access of consts
    double comms_range;
    std::unordered_map<int,std::vector<Msg>> message_tracker; // receiving robot ID, message struct

public:
    World(int X, int Y, Distance* distance, SensorModel* sensor_model, double comms_range);

    int getX() const { return X; }
    int getY() const { return Y; }

    cv::Mat& getImage();

    cv::Mat init();

    void plot();

    void trackRobot(Robot* robot);

    void printTrackedRobots();

    bool inComms(int id1, int id2);

    std::unordered_map<int, Robot*>& getRobotTracker() { return robot_tracker; }

    std::mutex& getWorldMutex() { return world_mutex; }

    std::unordered_map<int,std::vector<Msg>>& getMessageTracker() {return message_tracker; }

};

#endif
