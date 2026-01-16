#include <cstdio>
#include <cmath>
#include "planners.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "distance.hpp"
#include "behaviortree_cpp/actions/pop_from_queue.hpp"

Planner::Planner(int step_size, World* w) : step_size(step_size), current_plan(std::make_shared<BT::ProtectedQueue<Pose2D>>()), world(w) 
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
    /*if (step_size != 1) {
    std::cout << "Your step size is larger than 1! updateDistanceFromStart is not updated! See planners." << std::endl;
    std::cin.get();
    }*/ // updateDistanceFromStart function no longer exists

    //std::cout << "Current: " << x << ", " << y << std::endl;

    std::vector<P> neighbors;

    for (int i=-step_size; i<=step_size; i++) {
        for (int j=-step_size; j<=step_size; j++){
            if (!(i==0 && j==0)) { // Exclude current position as a neighbor
                double new_x = x + i; 
                double new_y = y + j;
                //std::cout << new_x << ", " << new_y << std::endl;
                // Check that neighbor is valid, i.e. on map (and eventually not in an obstacle)
                if (inBounds(new_x,new_y,X,Y) && !world->isObstacle(new_x, new_y)) { // added check for obstacle
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


ShortestPath::ShortestPath(int step_size, World* w) : Planner(step_size, w) {

}


//testing, just added prints of timing to try to diagnose interobot delay at start
std::vector<Pose2D> ShortestPath::plan(Pose2D start_pose, Pose2D waypoint, int X, int Y) {
    auto graph_start = std::chrono::high_resolution_clock::now();
    adjacency_vector graph = convertImageToAdjacencyVector(X, Y);
    auto graph_end = std::chrono::high_resolution_clock::now();
    
    // Get indices
    int start_idx = getIndex(start_pose.x, start_pose.y, Y);
    int goal_idx = getIndex(waypoint.x, waypoint.y, Y);
    std::vector<Pose2D> plan;
    
    auto dijkstra_start = std::chrono::high_resolution_clock::now();
    int V = X*Y;
    std::vector<double> distance_tracker = initializeDistances(V, start_idx, Y);
    std::vector<bool> visit_tracker = initializeVisits(V);
    
    std::priority_queue<P, std::vector<P>, std::greater<P>> priority_queue;
    priority_queue.push({0, start_idx});
    std::vector<int> predecessor(V, -1);
    
    while (!priority_queue.empty()) {
        int min_idx = priority_queue.top().second;
        priority_queue.pop();
        
        if (visit_tracker[min_idx]) continue;
        visit_tracker[min_idx] = true;
        if (min_idx == goal_idx) break;
        
        std::vector<P> neighbors = graph[min_idx];
        for (const P &np : neighbors) {
            double nc_dist = np.first;
            int n_idx = np.second;
            
            if (!visit_tracker[n_idx]) {
                double new_dist = nc_dist + distance_tracker[min_idx];
                if (new_dist < distance_tracker[n_idx]) {
                    distance_tracker[n_idx] = new_dist;
                    predecessor[n_idx] = min_idx;
                    priority_queue.push({new_dist, n_idx});
                }
            }
        }
    }

    // Adding for case where goal is impossible due to obstacles
    if (predecessor[goal_idx] == -1 && start_idx != goal_idx) {
        // Goal was never reached - no path exists
        std::cout << "No path to goal - unreachable!" << std::endl;
        return std::vector<Pose2D>();  // Return empty
    }
    
    auto dijkstra_end = std::chrono::high_resolution_clock::now();
    
    // Reconstruct path
    std::vector<int> path;
    for (int at = goal_idx; at != -1; at = predecessor[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    
    for (int idx : path) {
        std::pair<int, int> c = getCoords(idx, Y);
        plan.push_back({c.first, c.second, 0});
    }
    
    // Only keep timing output - much cleaner
    double graph_time = std::chrono::duration<double>(graph_end - graph_start).count();
    double dijkstra_time = std::chrono::duration<double>(dijkstra_end - dijkstra_start).count();
    std::cout << "ShortestPath timing - Graph: " << graph_time << "s, Dijkstra: " << dijkstra_time << "s" << std::endl;
    
    return plan;
}

/*std::vector<Pose2D> ShortestPath::plan(Pose2D start_pose, Pose2D waypoint, int X, int Y) {

    static std::atomic<int> call_count{0};
    static thread_local bool in_plan = false;
    
    if (in_plan) {
        std::cout << "ERROR: RECURSIVE/RE-ENTRANT CALL TO PLAN()!" << std::endl;
        std::cout << "This explains the corruption!" << std::endl;
        std::abort();
    }
    
    in_plan = true;
    int my_call = call_count++;
    std::cout << "=== PLAN CALL #" << my_call << " START ===" << std::endl;

    std::cout << "IN PLAN ######################" << std::endl;

    std::cout << "BEFORE getIndex: waypoint = (" << waypoint.x << ", " << waypoint.y << ")" << std::endl;

    // Inits
    int start_idx = getIndex(start_pose.x,start_pose.y,Y);
    int goal_idx = getIndex(waypoint.x, waypoint.y, Y);
    std::cout << "AFTER getIndex: waypoint = (" << waypoint.x << ", " << waypoint.y << ")" << std::endl;
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
            std::cout << "goal_idx is now: " << goal_idx << std::endl;  // ADD THIS
            std::cout << "min_idx == goal_idx? " << (min_idx == goal_idx) << std::endl;  // ADD THIS
            std::cout << "waypoint: " << waypoint.x << ", " << waypoint.y << std::endl;
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

    in_plan = false;
    std::cout << "=== PLAN CALL #" << my_call << " END ===" << std::endl;
    
    return plan;
}*/



CoveragePath::CoveragePath(int step_size, int obs_radius, World* w) : ShortestPath(step_size, w), obs_radius(obs_radius)  {
    std::cout << "HELLLLLOOOOOOOOO obs_radius in coverage path: " << obs_radius << std::endl;

}

std::vector<Pose2D> CoveragePath::generateBoustrophedonWaypoints(std::unordered_map<std::string,int> area) {

    // Long way across (e.g., left to right), short down, long way back, short down, repeat

    // std::cout << "2222HELLLLLOOOOOOOOO obs_radius in coverage path: " << obs_radius << std::endl;

    std::vector<Pose2D> waypoints;

    int x_min = area["xmin"]; //std::min({corner1.x, corner2.x, corner3.x, corner4.x});
    int x_max = area["xmax"]-1; //std::max({corner1.x, corner2.x, corner3.x, corner4.x});
    int y_min = area["ymin"]; //std::min({corner1.y, corner2.y, corner3.y, corner4.y});
    int y_max = area["ymax"]-1; //std::max({corner1.y, corner2.y, corner3.y, corner4.y});

    // 10% overlap between parallel across and back
    int distance_between_parallel_paths = static_cast<int>(std::round(obs_radius * 1.8));

    /*std::cout << "Area bounds: x=" << x_min << " to " << x_max 
          << ", y=" << y_min << " to " << y_max << std::endl;
    std::cout << "obs_radius=" << obs_radius << ", distance_between_parallel_paths=" 
              << distance_between_parallel_paths << std::endl;
    std::cout << "Y range = " << (y_max - y_min) << std::endl;*/

    // Direction to start/toggle
    bool left_to_right = true;

    for (int y = y_min + obs_radius; y <= y_max - obs_radius; y += distance_between_parallel_paths) {
        if (left_to_right) {
            waypoints.push_back({x_min, y, 0});
            waypoints.push_back({x_max, y, 0});
        } else {
            waypoints.push_back({x_max, y, 0});
            waypoints.push_back({x_min, y, 0});
        }
        left_to_right = !left_to_right;
    }


    return waypoints;
}

//std::shared_ptr<BT::ProtectedQueue<Pose2D>> CoveragePath::plan(Pose2D start_pose, Pose2D corner1, Pose2D corner2, Pose2D corner3, Pose2D corner4, int X, int Y) {
std::vector<Pose2D> CoveragePath::plan(Pose2D start_pose, std::unordered_map<std::string,int> area, int X, int Y) {

    std::vector<Pose2D> goal_waypoints = generateBoustrophedonWaypoints(area); // Waypoints at end of each segment of the coverage path

    std::cout << "Simple coverage waypoints before shortest path:" << std::endl;
    for (const auto& wp : goal_waypoints) {
        std::cout << "(" << wp.x << "," << wp.y << ") ";
    }
    std::cout << std::endl;

    std::vector<Pose2D> full_plan;
    Pose2D current_pose = start_pose;

    for (const auto& waypoint : goal_waypoints) {
        // Use your existing shortest path to get from current position to next waypoint
        auto segment = ShortestPath::plan(current_pose, waypoint, X, Y);
        
        // Add segment to full path (skip first point to avoid duplicates)
        if (!full_plan.empty() && !segment.empty()) {
            segment.erase(segment.begin());
        }
        full_plan.insert(full_plan.end(), segment.begin(), segment.end());
        
        current_pose = waypoint;
    }

    return full_plan;
}