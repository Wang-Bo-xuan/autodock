cmake_minimum_required(VERSION 2.8.3)
project(ls_msgs)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    message_generation
    genmsg
    actionlib_msgs
    actionlib
    )

add_message_files(
    DIRECTORY msg
    FILES
    check.msg
    LSLaserScan.msg
    LSLaserSensor.msg
    LSMultiLaserSensor.msg
    LSChargeBankPose.msg
    )

add_service_files(
    FILES
    cmd.srv
    )

add_action_files(
    FILES
    LSStepController.action
    )

generate_messages(DEPENDENCIES std_msgs sensor_msgs actionlib_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs)
