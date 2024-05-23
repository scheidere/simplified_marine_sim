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

void Planner::printMatrix(matrix m) {

    for (const auto& row : m) {
        for (double element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}

matrix Planner::initializeDistances(int X, int Y, Pose2D robot_start_loc) {

    // Deprecated (we are mapping x,y from world image indices to 1D vector for efficiency next)

    // infinity everywhere, 0 where the robot is now (at start)

    matrix distance_tracker(X, std::vector<double>(Y, std::numeric_limits<double>::infinity()));
    distance_tracker[robot_start_loc.x][robot_start_loc.y] = 0; // Set distance from start to start to 0

    return distance_tracker;
}

matrix Planner::initializeVisits(int X, int Y) {

    // This will also not be used (same reason as above, want to use 1D vector for memory efficiency, fewer nested loops needed)

    // 0 everywhere for unvisited, will set to 1 when visited (done elsewhere)

    matrix visit_tracker(X, std::vector<double>(Y, 0));

    return visit_tracker;
}

std::shared_ptr<BT::ProtectedQueue<Pose2D>> ShortestPath::plan(Pose2D current_pose, Pose2D waypoint, 
    matrix distance_tracker, matrix visit_tracker) {

    std::cout << "IN PLAN ######################" << std::endl;

    // Inits
    auto plan = std::make_shared<BT::ProtectedQueue<Pose2D>>(); //shared queue

    printMatrix(distance_tracker); printMatrix(visit_tracker);

    // Update neighbor distances to start using distance from current
    std::pair<matrix,matrix> trackers = updateDistanceFromStart(current_pose, distance_tracker, visit_tracker);

    distance_tracker = trackers.first; visit_tracker = trackers.second;

    printMatrix(distance_tracker); printMatrix(visit_tracker);

    /*

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
    */

    std::cout << "END PLAN ######################" << std::endl;

    return plan;

}

bool inBounds(double x, double y, matrix m) {
    int X = m.size(); // Rows
    int Y = m[0].size(); // Columns in first row
    if (x >= 0 && x < X && y >= 0 && y < Y) {
        return true; // In bounds

    } else {
        return false; // Out of bounds
    }
}



std::pair<matrix,matrix> ShortestPath::updateDistanceFromStart(Pose2D current, 
    matrix distance_tracker, matrix visit_tracker) {

    // Double check/change this function (wrt neighbors) before using with a step_size of 2+
    if (step_size != 1) {
    std::cout << "Your step size is larger than 1! updateDistanceFromStart is not updated! See planners." << std::endl;
    std::cin.get();
    }

    // Get current node distance to start from tracker
    double current_dist = distance_tracker[current.x][current.y];
    std::cout << current_dist << std::endl;

    // Loop through neighbor nodes (use step size)
    for (int i=-step_size; i<=step_size; i++) {
        for (int j=-step_size; j<=step_size; j++){
            if (!(i==0 && j==0)) { // Exclude current position as a neighbor
                double new_x = current.x + i; 
                double new_y = current.y + j;
                std::cout << new_x << ", " << new_y << std::endl;
                // Check that neighbor is valid, i.e. on map (and eventually not in an obstacle)
                if (inBounds(new_x,new_y, distance_tracker)) {
                    //std::cout << "Neighbor is valid" << std::endl;

                    double new_dist_to_start = current_dist + step_size;

                    // Get neighbor node distance from tracker
                    if (new_dist_to_start < distance_tracker[new_x][new_y]) {
                        distance_tracker[new_x][new_y] = new_dist_to_start;
                    }

                }
            }

        }
    }

    // Having calculated all neighbor distances, mark current node as visited
    visit_tracker[current.x][current.y] = 1;

    return std::make_pair(distance_tracker,visit_tracker);
}