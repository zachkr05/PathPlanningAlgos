##############################################################################
#  Path Planning – ROS 1 (catkin)                                            #
#  CMakeLists.txt                                                            #
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(path_planning)                       # ← all-lowercase is ROS style

# ---------------------------------------------------------------------------
# 1. Build options
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 2. Dependencies
# ---------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# 3. catkin_package – export headers & deps
# ---------------------------------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS Eigen3
)

# ---------------------------------------------------------------------------
# 4. Include directories
# ---------------------------------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 5. Executables
# ---------------------------------------------------------------------------
add_executable(robot_tracker_node    src/tracking/robot_tracker.cpp)
add_executable(obstacle_tracker_node src/tracking/obstacle_tracker.cpp)

target_link_libraries(robot_tracker_node    ${catkin_LIBRARIES})
target_link_libraries(obstacle_tracker_node ${catkin_LIBRARIES})

add_dependencies(robot_tracker_node    ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacle_tracker_node ${catkin_EXPORTED_TARGETS})

# ---------------------------------------------------------------------------
# 6. Install targets, headers, launch files
# ---------------------------------------------------------------------------
# Headers (so other packages can include them)
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Binaries
install(TARGETS robot_tracker_node obstacle_tracker_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Launch files (*.launch)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch")

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
message(STATUS "Configured path_planning package")

