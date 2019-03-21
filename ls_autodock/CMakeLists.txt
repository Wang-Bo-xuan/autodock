cmake_minimum_required(VERSION 2.8.3)
project(ls_autodock)

add_compile_options(-std=c++11)

find_package(catkin
    REQUIRED COMPONENTS
            roscpp
            rospy
            ls_msgs
            ls_laser_sensor
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
            ls_laser_sensor
            std_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_executable(sawtooth_detecter src/sawtooth_detecter.cpp)
add_dependencies(sawtooth_detecter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sawtooth_detecter ${catkin_LIBRARIES})

#add_executable(move_controller src/move_controller.cpp)
#add_dependencies(move_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(move_controller ${catkin_LIBRARIES})

add_executable(autodock src/main.cpp src/move_controller.cpp)
add_dependencies(autodock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autodock ${catkin_LIBRARIES})
