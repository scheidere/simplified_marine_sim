#include <cstdio>
#include "sensor_model.hpp"

SensorModel::SensorModel(Distance* d) : distance(d) {
}

void SensorModel::test() {
    std::cout << "Hello world from the sensor model class!" << std::endl;
}

void SensorModel::distance_test() {
    cv::Point vertex1(10, 20);  // Vertex 1 with coordinates (10, 20)
    cv::Point vertex2(30, 40);  // Vertex 2 with coordinates (30, 40)
    double dist = distance->euclidean(vertex1, vertex2);
    std::cout << "Distance test from sensor model: " << dist << std::endl;
}