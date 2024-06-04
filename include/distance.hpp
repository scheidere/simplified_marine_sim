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

    //double euclidean(cv::Point vertex_1, cv::Point vertex_2);
    static double getEuclideanDistance(int x1, int y1, int x2, int y2); // E.g. weight between vertices (pixels) for dijkstra

    std::string getD() const { return D; }

};

#endif
