#ifndef CONTROL_H
#define CONTROL_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>

#define DEGREE2RAD        0.01745329251994329577
#define RAD2DEGREE        57.29577951308232087685

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}

// angle diff from b to a
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}


typedef enum{
  ROTATE =-1,
  TURN_LEFT,
  TURN_RIGHT,
}TurnEnum;

class Control
{
public:

  Control();
  ~Control();

  void moveToPose(double goal_wx, double goal_wy, double goal_yaw);
  void moveTo(double goal_wx, double goal_wy, double goal_yaw);

  void rotateTo(double goal_yaw);

  void getPose(double &x, double &y, double &yaw);

  void getOdomPose(double &x, double &y, double &yaw);

  bool getTfPose(double &x, double &y, double &yaw);

  bool isInCell(double wx, double wy);

  void uTurn(TurnEnum td, double sx, double sy, double ex, double ey);

  void mapToOdom(double x_in, double y_in, double & x_out, double & y_out);
private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void coreSensorCallback(const kobuki_msgs::SensorStateConstPtr& msg);
  void bumperCallback(const kobuki_msgs::BumperEventConstPtr& msg);


  double pidController(double kp, double ki, double kd, double dt
                       , double &pre_error, double error, double &pre_integral);

  void publishVel(double x, double z);
  void publishZeroVel();


  double dist(double curr_x, double curr_y, double goal_x, double goal_y);

  void checkZValid(double &z, double z_max, double z_min);

  void poseToGridMap(double px, double py, int mx, int my);

private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber core_sensor_sub_;
  ros::Subscriber bumper_sub_;

  tf::TransformListener tf_;
//  tf::TransformListener tf_;

  boost::mutex control_mutex_;
  bool is_bumper;

  double odom_pose_[3];
  double curr_vel_x;
  double curr_vel_z;
  double sonar_dist_;
  int left_encoder_;
  int right_encoder_;

  double tick_to_rad;
  double wheel_radius; // in [m]
//  void controlThread();



};


#endif // MOVE_H
