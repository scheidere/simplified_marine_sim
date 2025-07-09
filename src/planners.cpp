#include <cstdio>
#include <cmath>
#include "planners.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "distance.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"

Planner::Planner(int step_size, int obs_radius) : step_size(step_size), obs_radius(obs_radius), current_plan(std::make_shared<BT::ProtectedQueue<Pose2D>>())
{
}

void Planner::test() {
    std::cout << "Hello world from the planner class!" << std::endl;
}

adjacency_vector Planner::convertImageToAdjacencyVector(int X, int Y) {

    adjacency_vector adj_vec;

    // Loop through every pixel i,j on world of dims X,Y
    for (int x=0; x<X; x++) {
        for (int y=0; y<Y; y++){
            std::vector<P> neighbors = getNeighbors(x,y,X,Y); // Vector of (dist,idx) pairs (double,int)
            adj_vec.push_back(neighbors); // idx of graph element represents what the neighbors are of
        }
    }

    return adj_vec;
}

void Planner::printAdjacencyVector(const adjacency_vector &adj_vec, int Y) {
    for (const auto& v : adj_vec) { // v is vector of P pairs (double, int), vec is main adjacency vector
        for (const auto& p : v) {
            std::pair<int,int> coords = getCoords(p.second, Y);
            std::cout << "Distance: " << p.first << ", neighbor idx:" << p.second << ", neighbor coords: " << coords.first << ", " << coords.second << std::endl;
        }
    }
}

int Planner::getIndex(int x, int y, int Y) {

    // Y is number of columns in world image

    // double/int might be an issue

    int idx = x*Y + y;

    return idx;
}

std::pair<int,int> Planner::getCoords(int idx, int Y) {

    int x = idx/Y; int y = idx%Y;
    return std::make_pair(x,y);
}

bool Planner::inBounds(int x, int y, int X, int Y) {
    if (x >= 0 && x < X && y >= 0 && y < Y) {
        return true; // In bounds

    } else {
        return false; // Out of bounds
    }
}

std::vector<std::pair<double,int>> Planner::getNeighbors(int x, int y, int X, int Y) {

    // Once again double/int might be an issue

    // Double check/change this function (wrt neighbors) before using with a step_size of 2+
    if (step_size != 1) {
    std::cout << "Your step size is larger than 1! updateDistanceFromStart is not updated! See planners." << std::endl;
    std::cin.get();
    }

    //std::cout << "Current: " << x << ", " << y << std::endl;

    std::vector<P> neighbors;

    for (int i=-step_size; i<=step_size; i++) {
        for (int j=-step_size; j<=step_size; j++){
            if (!(i==0 && j==0)) { // Exclude current position as a neighbor
                double new_x = x + i; 
                double new_y = y + j;
                //std::cout << new_x << ", " << new_y << std::endl;
                // Check that neighbor is valid, i.e. on map (and eventually not in an obstacle)
                if (inBounds(new_x,new_y,X,Y)) {
                    //std::cout << "Neighbor " << new_x << ", " << new_y << " is valid" << std::endl;
                    int neighbor_idx = getIndex(new_x,new_y,Y);
                    //std::cout << "neigh idx: " << neighbor_idx << std::endl;
                    double distance = Distance::getEuclideanDistance(x,y,new_x,new_y);
                    neighbors.push_back(std::make_pair(distance, neighbor_idx)); // pair = (dist, index)
                    

                }
            }

        }
    }

    return neighbors; // WITH DISTANCE BEFORE INDEX
}

/*double Planner::getEuclideanDistance(int x1, int y1, int x2, int y2) {

    int dx = x2 - x1; int dy = y2 - y1;
    double distance = std::sqrt(dx*dx + dy*dy);

    return distance;
}*/

/*void Planner::printVector(const std::vector<int> &vector) {

    for (int i=0;i<vector.size();i++) {
        std::cout << "Vertex: " << i << ", value: " << vector[i] << std::endl;

    }
}*/

template <typename T>
void Planner::printVector(const std::vector<T> &vec) const {
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << "Vertex: " << i << ", value: " << vec[i] << std::endl;
    }
}

std::vector<double> Planner::initializeDistances(int V, int robot_start_loc_idx, int Y) {

    // N is the number of nodes in the graph (X * Y)

    std::vector<double> distance_tracker(V,std::numeric_limits<double>::infinity());

    distance_tracker[robot_start_loc_idx] = 0;

    return distance_tracker;
}

std::vector<bool> Planner::initializeVisits(int V) {

    std::vector<bool> visit_tracker(V,false);
    return visit_tracker;
}

/*void Planner::printPlan(const std::shared_ptr<BT::ProtectedQueue<Pose2D>>& plan) {
    std::cout << "Plan contents:" << std::endl;
    for (const auto& pose : plan->items) {
        std::cout << "Pose2D(x: " << pose.x << ", y: " << pose.y << ", theta: " << pose.theta << ")" << std::endl;
    }
}*/


ShortestPath::ShortestPath(int step_size) : Planner(step_size, obs_radius) {

}  

// //std::make_shared<BT::ProtectedQueue<Pose2D>> ShortestPath::getPlan()

// std::shared_ptr<BT::ProtectedQueue<Pose2D>> ShortestPath::plan(Pose2D start_pose, Pose2D waypoint, int X, int Y) {

//     std::cout << "IN PLAN ######################" << std::endl;

//     // Inits
//     int start_idx = getIndex(start_pose.x,start_pose.y,Y); // Idx of robot current location
//     int goal_idx = getIndex(waypoint.x, waypoint.y, Y); // Goal idx
//     std::cout << "Goal idx: " << goal_idx << std::endl;
//     auto plan = std::make_shared<BT::ProtectedQueue<Pose2D>>(); //shared queue that will hold the shortest path
//     adjacency_vector graph = convertImageToAdjacencyVector(X, Y);
//     //printAdjacencyVector(graph, Y);
//     int V = X*Y; // Number of vertices
//     std::vector<double> distance_tracker = initializeDistances(V, start_idx, Y);
//     std::vector<bool> visit_tracker = initializeVisits(V);
//     //printVector(distance_tracker);
//     //printVector(visit_tracker);
//     // Priority queue, top being vertex with minimum distance to start
//     std::priority_queue<P, std::vector<P>, std::greater<P>> priority_queue; //min heap (distance to start, vertex idx) (double,int)
//     priority_queue.push({0, start_idx});

//     // Track previous points in shortest path
//     std::vector<int> predecessor(V, -1);
    
//     int count = 0;
//     while (!priority_queue.empty()) {
//         //count += 1;
//         //std::cout << "Iteration count: " << count << std::endl;
//         int min_idx = priority_queue.top().second; // Get unvisited vertex with minimum distance to start (do we have to explicitly check that its unvisted?)
//         priority_queue.pop(); // Remove that distance, vertex pair from priority_queue (maybe this is all the visit tracking needed?)
//         // Check if the node has already been visited
//         if (visit_tracker[min_idx]) {
//             continue; // Skip if you have already visited the node you just chose (why not removed from the priority queue then?)
//         }

//         // Mark the current node as visited
//         visit_tracker[min_idx] = true;

//         // Update path with current point (THIS IS WRONG)
//         /*std::pair<int,int> c = getCoords(min_idx,Y); // Current
//         Pose2D next_point{c.first,c.second,0}; //x,y,theta
//         plan->items.push_back(next_point); // Add to path*/

//         // Check if current is goal waypoint
//         if (min_idx == goal_idx) {
//             std::cout << "Goal reached at node " << min_idx << "!" << std::endl;
//             break;
//         }

//         // Get distances to neighbors, where neighbors have not yet been visited
//         //std::vector<std::pair<int,int>> neighbors = getNeighbors(c.first, c.second, X, Y);
//         std::vector<P> neighbors = graph[min_idx]; // pair (double,int)
//         for (const P &np : neighbors) { // For each dist,idx_neighbor pair, i.e. std::pair<double,int>
//             double nc_dist = np.first; // Get distance to neighbor from current
//             int n_idx = np.second;
//             //std::pair<int,int> n = getCoords(idx,Y); // Don't need this?
//             if (!visit_tracker[n_idx]) { // Neighbor has not been visited (??)
                
//                 // Update distance vector
//                 double new_dist =  nc_dist + distance_tracker[min_idx]; // Neighbor distance to start, neighbor-current dist + start-current dist
//                 if (new_dist < distance_tracker[n_idx]) {
//                     distance_tracker[n_idx] = new_dist;
//                     predecessor[n_idx] = min_idx;

//                     // Update priority queue (i.e., add neighbor)
//                     P new_p(new_dist,n_idx);
//                     priority_queue.push(new_p);
//                 }


//                 // Do I need to check if visited? Where?
//             }

//             // Need goal check handling

//         }

//         if (priority_queue.empty()) {
//             std::cout << "ALERT: PQ is empty!" << std::endl;
//         }

//     }

//     // Reconstruct the path from start to goal using the predecessor map
//     std::vector<int> path;
//     for (int at = goal_idx; at != -1; at = predecessor[at]) { // **Path reconstruction using predecessor**
//         path.push_back(at);
//     }
//     std::reverse(path.begin(), path.end());

//     for (int idx : path) {
//         std::pair<int, int> c = getCoords(idx, Y); // Current coordinates
//         Pose2D next_waypoint{c.first, c.second, 0}; // x, y, theta
//         plan->items.push_back(next_waypoint); // **Add reconstructed path to plan**
//     }

//     printPlan(plan);
//     std::cout << "END PLAN ######################" << std::endl;
//     //std::cin.get();
//     return plan;

// }

std::vector<Pose2D> ShortestPath::plan(Pose2D start_pose, Pose2D waypoint, int X, int Y) {

    std::cout << "IN PLAN ######################" << std::endl;

    // Inits
    int start_idx = getIndex(start_pose.x,start_pose.y,Y);
    int goal_idx = getIndex(waypoint.x, waypoint.y, Y);
    std::cout << "Goal idx: " << goal_idx << std::endl;
    
    // Init plan (of poses)
    std::vector<Pose2D> plan;
    
    adjacency_vector graph = convertImageToAdjacencyVector(X, Y);
    int V = X*Y;
    std::vector<double> distance_tracker = initializeDistances(V, start_idx, Y);
    std::vector<bool> visit_tracker = initializeVisits(V);
    
    std::priority_queue<P, std::vector<P>, std::greater<P>> priority_queue;
    priority_queue.push({0, start_idx});

    std::vector<int> predecessor(V, -1);
    
    int count = 0;
    while (!priority_queue.empty()) {
        int min_idx = priority_queue.top().second;
        priority_queue.pop();
        
        if (visit_tracker[min_idx]) {
            continue;
        }

        visit_tracker[min_idx] = true;

        if (min_idx == goal_idx) {
            std::cout << "Goal reached at node " << min_idx << "!" << std::endl;
            break;
        }

        std::vector<P> neighbors = graph[min_idx];
        for (const P &np : neighbors) {
            double nc_dist = np.first;
            int n_idx = np.second;
            
            if (!visit_tracker[n_idx]) {
                double new_dist = nc_dist + distance_tracker[min_idx];
                if (new_dist < distance_tracker[n_idx]) {
                    distance_tracker[n_idx] = new_dist;
                    predecessor[n_idx] = min_idx;
                    P new_p(new_dist,n_idx);
                    priority_queue.push(new_p);
                }
            }
        }

        if (priority_queue.empty()) {
            std::cout << "ALERT: PQ is empty!" << std::endl;
        }
    }

    // Reconstruct the path from start to goal using the predecessor map
    std::vector<int> path;
    for (int at = goal_idx; at != -1; at = predecessor[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    // Change: Add directly to vector instead of plan->items
    for (int idx : path) {
        std::pair<int, int> c = getCoords(idx, Y);
        Pose2D next_waypoint{c.first, c.second, 0};
        plan.push_back(next_waypoint);  // Direct push_back
    }

    std::cout << "END PLAN ######################" << std::endl;
    
    // Change: Return vector directly
    return plan;
}



CoveragePath::CoveragePath(int step_size, int obs_radius) : ShortestPath(step_size) {

}

//std::shared_ptr<BT::ProtectedQueue<Pose2D>> CoveragePath::plan(Pose2D start_pose, Pose2D corner1, Pose2D corner2, Pose2D corner3, Pose2D corner4, int X, int Y) {
std::vector<Pose2D> CoveragePath::plan(Pose2D start_pose, Pose2D corner1, Pose2D corner2, Pose2D corner3, Pose2D corner4, int X, int Y) {


    /*int start_idx = getIndex(start_pose.x,start_pose.y,Y); // Idx of robot current location
    auto plan = std::make_shared<BT::ProtectedQueue<Pose2D>>(); //shared queue that will hold the shortest path
    adjacency_vector graph = convertImageToAdjacencyVector(X, Y);

    int V = X*Y; // Number of vertices
    std::vector<double> distance_tracker = initializeDistances(V, start_idx, Y);
    std::vector<bool> visit_tracker = initializeVisits(V);*/

    std::cout << "shortest path issue here..." << std::endl;
    std::cout << "Step size check: " << step_size << std::endl;

    auto plan = ShortestPath::plan(start_pose, corner2, X, Y); // Testing that shortest path works from here

    return plan;
}