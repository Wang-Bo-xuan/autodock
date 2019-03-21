#ifndef __INCLUDE_LS_CONTROLLER_LS_STEP_CONTROLLER_H__
#define __INCLUDE_LS_CONTROLLER_LS_STEP_CONTROLLER_H__

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <ls_controller/control.h>
#include <ls_msgs/LSStepControllerAction.h>
#include <actionlib/server/simple_action_server.h>
using namespace std;

typedef actionlib::SimpleActionServer<ls_msgs::LSStepControllerAction> Server;

namespace LS_SLAM
{
  class LSStepController
  {
    public:
      LSStepController(ros::NodeHandle *nh);
      ~LSStepController();

    private:
      void init(void);
      void execute(const ls_msgs::LSStepControllerGoalConstPtr &goal);
      void preempt(void);

      void OdomCallBack(const nav_msgs::Odometry msg);

      ros::Subscriber *odom_sub_;
      ros::Publisher vel_pub_;

      Server *server_;
      Control *control_;
      double x_,y_,yaw_;
  };
}

#endif
