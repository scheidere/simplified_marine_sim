#include <cstdio>
#include <cmath>
#include "distance.hpp"

Distance::Distance() : D ("Test passed!") {

}

void Distance::test() {
    std::cout << "Hello world from the distance class!" << std::endl;
}

/*double Distance::euclidean(cv::Point vertex_start, cv::Point vertex_end) {
    double x = vertex_start.x - vertex_end.x;
    double y = vertex_start.y - vertex_end.y;
    return sqrt(x * x + y * y);

}*/

double Distance::getEuclideanDistance(int x1, int y1, int x2, int y2) {

    int dx = x2 - x1; int dy = y2 - y1;
    double distance = std::sqrt(dx*dx + dy*dy);

    return distance;
}
