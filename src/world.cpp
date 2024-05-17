#include <cstdio>
#include "distance.hpp"
#include "sensor_model.hpp"
#include "world.hpp"

World::World(int X, int Y, Distance* d, SensorModel* s) 
    : X(X),
    Y(Y), 
    distance(d), 
    sensor_model(s),
    image(init())
{
    cv::Mat image = init(); // Create world background image of dimensions X and Y
}

void World::test() {
    std::cout << "Hello world from the world class!" << std::endl;
}

void World::distance_test() {
    std::cout << "Testing world class access to distance class: " << distance->getD() << std::endl;

}

cv::Mat World::init() {
    // White background
    std::cout << "Initializing world..." << std::endl;
    cv::Mat image(X, Y, CV_8UC3, cv::Scalar(255, 255, 255));

    return image;
}

void World::plot(cv::Mat& image) {
    std::cout << "Plotting world..." << std::endl;
    cv::imshow("Quadrant Image", image);
    cv::waitKey(300);
}