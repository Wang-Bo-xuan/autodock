cmake_minimum_required(VERSION 2.8.3)
project(ls_laser_sensor)

add_compile_options(-std=c++11)

find_package(catkin
    REQUIRED COMPONENTS
            roscpp
            rospy
            ls_msgs
            std_msgs
            sensor_msgs
            tf
)

catkin_package(
    INCLUDE_DIRS
            include
    CATKIN_DEPENDS
            roscpp
            rospy
            ls_msgs
            std_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_executable(laser_sensor src/laser_sensor.cpp)
add_dependencies(laser_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_sensor ${catkin_LIBRARIES})
