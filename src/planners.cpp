#include <cstdio>
#include <cmath>
#include "planners.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"


Planner::Planner() : current_plan(std::make_shared<BT::ProtectedQueue<Pose2D>>()) {

}

void Planner::test() {
    std::cout << "Hello world from the planner class!" << std::endl;
}

ShortestPath::ShortestPath() : Planner() {

}

double Planner::euclideanDistance(cv::Point p1, cv::Point p2) {

    int dx = p2.x - p1.x; int dy = p2.y - p1.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    return distance;
}

template <int N, int M>
void Planner::printDistancesArray(std::array<std::array<double, N>, M>& arr) {

    // Only use this for smaller N and M, otherwise you will be sad

    std::cout << "Printing std::array 2d style!" << std::endl;

    for (int i=0; i < N; i++) {
        for (int j=0; j < M; j++) {
            std::cout << arr[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void Planner::initializeDistances(cv::Mat image, cv::Point p1) {

    //const int N = image.rows; const int M = image.cols; // Are these correct?
    std::cout << "Type of image.cols: " << typeid(image.cols).name() << image.cols << std::endl;
    const int N = 400; const int M = 400;
    std::array<std::array<double, N>, M> distance_array;
    std::cout << p1.x << " " << p1.y << std::endl;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            if (i == p1.x && j == p1.y) {
                distance_array[i][j] = 0;
            } else {
                distance_array[i][j] = std::numeric_limits<double>::infinity();
            }
        }
    }

    //printDistancesArray<N,M>(distance_array);

    //return distance_array;

}

std::shared_ptr<BT::ProtectedQueue<Pose2D>> ShortestPath::plan(Pose2D current_pose, Pose2D waypoint) {

    // First hardcode direct path to point 4 pixels up from the robot (200,200) initial location
    std::cout << "Current robot location x, y: " << current_pose.x << ", " << current_pose.y << std::endl;
    auto plan = std::make_shared<BT::ProtectedQueue<Pose2D>>(); //shared queue
    double step_size = getStepSize();
    std::cout << "step_size: " << step_size << std::endl;
    std::cout << "HELLO" << std::endl;
    double x_prev = current_pose.x; double y_prev = current_pose.y;
    cv::Point old(current_pose.x,current_pose.y);
    for (int i = 0; i < 4; ++i) {
        Pose2D next_waypoint{ x_prev, y_prev + step_size, 0 };
        plan->items.push_back(next_waypoint);
        std::cout << "Generated waypoint: (" << next_waypoint.x << ", " << next_waypoint.y << ", " << next_waypoint.theta << ")" << std::endl;
        x_prev = next_waypoint.x; y_prev = next_waypoint.y;
        cv::Point newPoint(next_waypoint.x,next_waypoint.y);
        double dist = this->euclideanDistance(old, newPoint);
        std::cout << "Distance: " << dist << std::endl;
    }    

    return plan;

}