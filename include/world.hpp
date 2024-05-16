#ifndef WORLD_H
#define WORLD_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "distance.hpp"
#include "sensor_model.hpp"

class World {
private:
    int X; int Y;
    Distance* distance;
    SensorModel* sensor_model;

public:
    World(int X, int Y, Distance* distance, SensorModel* sensor_model);

    int getX() const { return X; }
    int getY() const { return Y; }

    void test();

    void distance_test();

    cv::Mat init();

    void plot(cv::Mat& image);

};

#endif
