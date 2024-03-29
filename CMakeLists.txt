cmake_minimum_required(VERSION 3.0.2)
project(slam_karto_ceres)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra")

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/slam_karto_ceres;
#   -warnings-as-errors=*;
# )

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

# Ceres
find_package(Ceres 2.1.0 REQUIRED)

# Glog
find_package(glog REQUIRED)

catkin_package()

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(ceres_solver slam_karto_ceres/ceres_solver.cc)
target_link_libraries(ceres_solver Ceres::ceres Eigen3::Eigen glog::glog ${catkin_LIBRARIES})

add_executable(slam_karto_ceres
  slam_karto_ceres/slam_karto_ceres.cc
  slam_karto_ceres/slam_karto_ceres_node.cc
)
target_link_libraries(slam_karto_ceres ceres_solver ${catkin_LIBRARIES})

add_executable(slam_karto_ceres_test test/slam_karto_ceres_test.cc)
target_link_libraries(slam_karto_ceres_test ceres_solver)
