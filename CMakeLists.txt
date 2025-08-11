cmake_minimum_required(VERSION 3.0.2)
cmake_policy(SET CMP0048 NEW)
project(PathPlanningAlgos VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 1. Dependencies
# ---------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp geometry_msgs std_msgs
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
# ---------------------------------------------------------------------------
add_executable(obstacle_node
    src/main.cpp
)
target_link_libraries(obstacle_node
    ${catkin_LIBRARIES}
)

add_executable(GPF_node
    src/potential_fields/gauss/gaussian_potential_field.cpp
)
target_link_libraries(GPF_node
    ${catkin_LIBRARIES}
    Threads::Threads
)

