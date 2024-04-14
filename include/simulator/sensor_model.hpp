#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "simulator/distance.hpp"

class SensorModel {
private:
    Distance* distance;

public:
    SensorModel(Distance* distance);

    void test();

    void distance_test();

};

#endif