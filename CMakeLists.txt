cmake_minimum_required(VERSION 3.0.2)
project(slam_karto_ceres)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_filters
  nav_msgs
  open_karto
  roscpp
  sensor_msgs
  tf
)

# Eigen3
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message(STATUS ${EIGEN3_INCLUDE_DIRS})
endif()

# Ceres
find_package(Ceres REQUIRED)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(slam_karto_ceres src/slam_karto_ceres.cpp src/ceres_solver.cpp)
target_link_libraries(slam_karto_ceres ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(slam_karto_gt src/slam_karto_gt.cpp src/ceres_solver.cpp)
target_link_libraries(slam_karto_gt ${catkin_LIBRARIES} ${CERES_LIBRARIES})