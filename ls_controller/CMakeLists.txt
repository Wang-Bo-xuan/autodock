cmake_minimum_required(VERSION 2.8.3)
project(ls_controller)

find_package(catkin
    REQUIRED COMPONENTS
            roscpp
            rospy
            ls_msgs
            std_msgs
            sensor_msgs
            geometry_msgs
            tf
            actionlib
            actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS
            include
    CATKIN_DEPENDS
            roscpp
            rospy
            ls_msgs
            std_msgs
            geometry_msgs
            actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ls_step_controller src/ls_step_controller.cpp src/control.cpp)
add_dependencies(ls_step_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ls_step_controller ${catkin_LIBRARIES})
