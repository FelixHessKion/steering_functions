cmake_minimum_required(VERSION 2.8.3)
project(steering_functions)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

## Get all source files
file(GLOB SRC_HC_CC_SS src/hc_cc_state_space/*.cpp)
set(SRC_FILES
    src/dubins_state_space/dubins_state_space.cpp
    src/reeds_shepp_state_space/reeds_shepp_state_space.cpp
    src/utilities/utilities.cpp
    src/filter/ekf.cpp
    ${SRC_HC_CC_SS})

## Declare a C++ library
add_library(${PROJECT_NAME} ${SRC_FILES})
