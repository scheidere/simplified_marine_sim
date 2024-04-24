#include <cstdio>
#include <cmath>
#include "planners.hpp"
#include "world.hpp"
#include "robot.hpp"

Planner::Planner() : P ("In planner") {

}

void Planner::test() {
    std::cout << "Hello world from the planner class!" << std::endl;
}

std::vector<std::pair<int, int>> Planner::smoothBigStep(int prev_x, int prev_y, int big_x_step, int big_y_step, int num_substeps) {
      // Divide the step into substeps for smoother animation
      std::vector<std::pair<int,int>> lil_steps; // Init action sequence

      double x_step = static_cast<double>(big_x_step) / num_substeps;
      double y_step = static_cast<double>(big_y_step) / num_substeps;
      for (int i = 0; i < num_substeps; ++i) {
        int next_x = prev_x + static_cast<int>(x_step * i);
        int next_y = prev_y + static_cast<int>(y_step * i);

        lil_steps.push_back(std::make_pair(next_x, next_y));


        //robot.move(next_x, next_y, background);
        //world.plot(background);

      }
      return lil_steps;
}

RandomWalkPlanner::RandomWalkPlanner() : Planner() {

}

std::vector<std::pair<int,int>> RandomWalkPlanner::randomStep(int prev_x, int prev_y) {
    int max_step_size = 10;
    std::uniform_int_distribution<int> dist(-max_step_size, max_step_size);
    int x_step = dist(rng); int y_step = dist(rng);
    std::cout << "X step: " << x_step << ", Y step: " << y_step << std::endl;
    int new_x = prev_x + x_step; int new_y = prev_y + y_step;

    // Might want to add some animation smoothing here (if the step size is over 1, e.g., 10)
    /*if (x_step > 1 || y_step > 1) {
      std::vector<std::pair<int,int>> path = Planner::smoothBigStep(prev_x, prev_y, x_step,y_step,max_step_size);
      return path;
    }*/ // Commenting out smoothing while testing doIteration
    
    std::cout << "New X: " << new_x << ", New Y: " << new_y << std::endl;
    std::vector<std::pair<int,int>> path = {{new_x, new_y}};
    return path;
}

void RandomWalkPlanner::performRandomWalk(cv::Mat& background, int steps, World& world, Robot& robot) {
for (int i = 0; i < steps; ++i) {
    int prev_x = robot.getX();
    int prev_y = robot.getY();
    std::vector<std::pair<int, int>> step_path = randomStep(prev_x, prev_y);
    // Robot will either move one pixel or several to take a larger "step"
    //update step to be a list or something that either has 1 or more pairs in it
    //update move so that it can take a list and carry out the plan with 1 or more pairs (this is all for smoothing)
    //robot.move(step.first, step.second, background);
    robot.followPath(step_path,background);
    world.plot(background);
};
}
