#ifndef DISTANCE_H
#define DISTANCE_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>


class Distance {
private:
    std::string D;

public:
    Distance();

    void test();

    double euclidean(cv::Point vertex_1, cv::Point vertex_2);

    std::string getD() const { return D; }

};

#endif
