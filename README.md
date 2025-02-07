# Self-Adapting Multirobot Behavior Tree

Currently, this repository includes a basic simulator that represents a simplified multirobot coordination task. We are in the process of evaluating the benefits and efficacy of online behavior tree adaptation in conjunction with a variation of the consensus based bundle algorithm (CBBA).

## Getting started

### Dependencies

* [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
* [ROS 2 Iron Irwini](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
* Create a [workspace](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
* Clone [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
* ```sudo apt install nlohmann-json3-dev``` for parsing JSON

### Installation

* Clone this repository and run the following: ```colcon build``` and ```source install/setup.bash```

### Testing

* ```ros2 run simulator sim```
