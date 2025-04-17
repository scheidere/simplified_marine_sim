cmake_minimum_required(VERSION 3.5)
project(simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# Add executable
add_executable(sim src/sim.cpp src/CBBA.cpp src/distance.cpp src/message.cpp src/sensor_model.cpp src/world.cpp src/planners.cpp src/robot.cpp src/behavior_tree.cpp src/parser.cpp)

ament_target_dependencies(sim behaviortree_cpp)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(sim 
  ${OpenCV_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${catkin_LIBRARIES} nlohmann_json::nlohmann_json
)

# Install the executable
install(TARGETS sim
        DESTINATION lib/${PROJECT_NAME})

ament_package()