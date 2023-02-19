cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_matching
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
    include
    include/unitree_arm_sdk
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread")

link_directories(lib)

add_executable(calibration src/calibration.cpp)
target_link_libraries(
    calibration
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Z1_SDK_Linux64
)

add_executable(keyboard_control src/keyboard_control.cpp)
target_link_libraries(
    keyboard_control
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Z1_SDK_Linux64
)

add_executable(arm_control src/arm_control.cpp)
target_link_libraries(
    arm_control
    ${catkin_LIBRARIES}
    Z1_SDK_Linux64
    ${PCL_LIBRARIES}
)

add_executable(reachability_map src/reachability_map.cpp)
target_link_libraries(
    reachability_map
    ${catkin_LIBRARIES}
    Z1_SDK_Linux64
    ${PCL_LIBRARIES}
)




