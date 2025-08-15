cmake_minimum_required(VERSION 3.0.2)
cmake_policy(SET CMP0048 NEW)
project(ImmersionBot VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 1. Dependencies
# ---------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    message_generation
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
add_message_files(
  FILES
  GaussianParams.msg
  Goal.msg
  Obstacle.msg
  Robot.msg
  ObstacleList.msg
  Point.msg
)

add_service_files(
  FILES
  generateTraj.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp geometry_msgs std_msgs message_runtime
    DEPENDS Eigen3
)


# ---------------------------------------------------------------------------
# 2. Include directories
# ---------------------------------------------------------------------------
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 3. Executables


add_executable(GPF_node
    src/pathPlanner.cpp
    src/potential_fields/gauss/gaussian_potential_field.cpp
)
target_link_libraries(GPF_node
    ${catkin_LIBRARIES}
    Threads::Threads
)

