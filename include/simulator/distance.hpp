#ifndef DISTANCE_H
#define DISTANCE_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>


class Distance {
private:

public:
    Distance();

    void test();

    double euclidean(cv::Point vertex_1, cv::Point vertex_2);
};

#endif
