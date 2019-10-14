cmake_minimum_required(VERSION 2.8.3)
project(moveo_moveit_move_group)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
    trac_ik_lib
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
find_package(Boost REQUIRED system filesystem date_time thread)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ executable
add_executable(ik_test src/moveo_ik_test.cpp)
add_executable(moveo_set_pose src/moveo_set_pose.cpp)
add_executable(parallel_ik_test src/parallel_ik_test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ik_test
  ${catkin_LIBRARIES}
)
target_link_libraries(moveo_set_pose
  ${catkin_LIBRARIES}
)
target_link_libraries(parallel_ik_test
  ${catkin_LIBRARIES}
)
