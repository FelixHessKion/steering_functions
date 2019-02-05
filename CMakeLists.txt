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
## Get OMPL state space definitions
file(GLOB OMPL_SPACES include/ompl_state_spaces/*.hpp)

set(SRC_FILES
    src/dubins_state_space/dubins_state_space.cpp
    src/reeds_shepp_state_space/reeds_shepp_state_space.cpp
    src/utilities/utilities.cpp
    src/filter/ekf.cpp
    ${SRC_HC_CC_SS}
    ${OMPL_SPACES})

## Declare a C++ library
add_library(${PROJECT_NAME} ${SRC_FILES})

add_executable(ompl_demo ${SRC_FILES} src/ompl_demo.cpp)
target_link_libraries(ompl_demo -lompl)
