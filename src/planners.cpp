#include <cstdio>
#include <cmath>
#include "planners.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"

Planner::Planner(int step_size) : step_size(step_size), current_plan(std::make_shared<BT::ProtectedQueue<Pose2D>>())
{
}

void Planner::test() {
    std::cout << "Hello world from the planner class!" << std::endl;
}

ShortestPath::ShortestPath(int step_size) : Planner(step_size) {

}

double Planner::euclideanDistance(cv::Point p1, cv::Point p2) {

    int dx = p2.x - p1.x; int dy = p2.y - p1.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    return distance;
}

void Planner::printDistances(std::vector<std::vector<double>> distance_array) {

    for (const auto& row : distance_array) {
        for (double element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}

std::vector<std::vector<double>> Planner::initializeDistances(int X, int Y, Pose2D robot_start_loc) {

    std::vector<std::vector<double>> distance_array(X, std::vector<double>(Y, std::numeric_limits<double>::infinity()));
    distance_array[robot_start_loc.x][robot_start_loc.y] = 0; // Set distance from start to start to 0

    return distance_array;
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

/*double ShortestPath::updateDistanceFromStart(Pose2D current, Pose2D neighbor) {

    // Get neighbor node distance from array


    // Get current node distance from array


    // Distance between nodes is always the step size in this case, so add step size to get new current node distance from start


    // If new dist < old dist at current node, update dist array

    //return // updated dist array
    return 0;
}*/