#include <ls_controller/control.h>

Control::Control():is_bumper(false), sonar_dist_(0.0), left_encoder_(0), right_encoder_(0)
                   ,tick_to_rad(M_PI*2/720), wheel_radius(0.0295), curr_vel_x(0.0), curr_vel_z(0.0)
{
  odom_sub_ = nh_.subscribe("odom", 1, &Control::odomCallback, this);
  bumper_sub_ = nh_.subscribe("mobile_base/events/bumper",10, &Control::bumperCallback, this);
  core_sensor_sub_ = nh_.subscribe("mobile_base/sensors/core", 1, &Control::coreSensorCallback, this);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
}

Control::~Control()
{

}

void Control::mapToOdom(double x_in, double y_in, double & x_out, double & y_out)
{
  geometry_msgs::PointStamped pt_in;
  pt_in.header.stamp = ros::Time();
  pt_in.header.frame_id = "map";
  pt_in.point.x = x_in;
  pt_in.point.y = y_in;
  pt_in.point.z = 0;
  geometry_msgs::PointStamped pt_out;
  try{
    tf_.waitForTransform("map", "odom", ros::Time(0), ros::Duration(3.0));
    tf_.transformPoint("odom", pt_in, pt_out);
    x_out = pt_out.point.x;
    y_out = pt_out.point.y;
    ROS_INFO("x_in = %f, y_in = %f, x_out = %f, y_out = %f", x_in, y_in, x_out, y_out);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }


}

double Control::dist(double curr_x, double curr_y, double goal_x, double goal_y)
{
  return std::sqrt((curr_y-goal_y)*(curr_y-goal_y) + (curr_x-goal_x)*(curr_x-goal_x));
}

void Control::checkZValid(double &z, double z_max, double z_min)
{
  int sgn = 0;
  sgn = (z > 0) ? (1) : (-1);

  if (fabs(z) > z_max)
    z = sgn*z_max;
  else if (fabs(z) < z_min)
    z = sgn*z_min;
}



void Control::moveToPose(double goal_wx, double goal_wy, double goal_yaw)
{
  ROS_INFO("move to pose (%f, %f, %f)", goal_wx, goal_wy, goal_yaw*RAD2DEGREE);
ROS_WARN("yaw:%lf",goal_yaw*180/3.14);
  double x, y, yaw;
//  getOdomPose(x, y, yaw);
  getPose(x, y, yaw);
  double diff = fabs(angle_diff(goal_yaw, yaw));
  ROS_INFO("robot_yaw = %f, goal_yaw = %f, diff = %f", yaw*RAD2DEGREE, goal_yaw*RAD2DEGREE, diff*RAD2DEGREE);
  if (diff > 5*DEGREE2RAD)
  {
    rotateTo(goal_yaw);
  }

  ros::Rate r(20);
  double goal_vel_x = 0.13;

  double kp = 5.5, ki = 0.1, kd = 1.2, dt = 0.05;
  double pre_yaw_error = 0.0, pre_yaw_integral = 0.0;
  double yaw_error = 0.0;
  double d_error = 0.0;
  double error;
  double k1 = 1.0;
  double k2 = 0.5;
  bool is_goal_reached = false;
  double pre_dist_to_goal = 4.0;

  double dist_to_goal = dist(x, y, goal_wx, goal_wy);
  ROS_INFO("dist to goal = %f", dist_to_goal);
  double start_x, start_y, start_yaw;
//  getOdomPose(start_x, start_y, start_yaw);
  getPose(start_x, start_y, start_yaw);
  ROS_INFO("start_x = %f, start_y = %f, start_yaw = %f", start_x, start_y, start_yaw);
  while(/*!is_bumper && */!is_goal_reached && ros::ok())
  {
    double curr_x, curr_y, curr_yaw;
//    getOdomPose(curr_x, curr_y, curr_yaw);
    getPose(curr_x, curr_y, curr_yaw);
    double dist_to_start = dist(curr_x, curr_y, start_x, start_y);

    if (dist_to_start >  dist_to_goal-0.01)
    {
      ROS_INFO("break");
      ROS_INFO("dist_to_start = %f yaw_error = %f, curr_yaw = %f, goal_yaw = %f"
               , dist_to_start, yaw_error*RAD2DEGREE, curr_yaw*RAD2DEGREE, goal_yaw*RAD2DEGREE    );
      publishZeroVel();
      is_goal_reached = true;
      break;
    }
    else if (dist_to_start >  dist_to_goal - 0.01)
    {
      goal_vel_x = 0.05;
    }

    yaw_error = angle_diff(goal_yaw, curr_yaw);

    double d = dist(curr_x, curr_y, goal_wx, goal_wy);
    double theta = std::atan2(goal_wy-curr_y, goal_wx-curr_x);
    double theta_error = angle_diff(goal_yaw, theta);


    if (std::fabs(d) < 0.005)
    {
      d_error = 0;
    }
    else
      d_error = d * std::sin(theta_error);

    error = k1*yaw_error/* - k2*d_error*/;
    double v_yaw;
    if(std::fabs(yaw_error) > 1 * DEGREE2RAD)
    {
      v_yaw = pidController(kp, ki, kd, dt,
                                   pre_yaw_error, error, pre_yaw_integral);
    }
    else
      v_yaw = 0;

    ROS_INFO_THROTTLE(0.5, "dist_to_start = %f error = %f, curr_yaw = %f, goal_yaw = %f"
                      , dist_to_start, yaw_error*RAD2DEGREE, curr_yaw*RAD2DEGREE, goal_yaw*RAD2DEGREE);
    checkZValid(v_yaw, 0.2, 0.0);

    double vel_x = curr_vel_x;
    if (vel_x <= goal_vel_x)
    {
      vel_x += 0.02;
      if (vel_x > goal_vel_x)
        vel_x = goal_vel_x;
    }
    else if (vel_x >= goal_vel_x)
    {
      vel_x -= 0.02;
      if (vel_x < goal_vel_x)
        vel_x = goal_vel_x;
    }

//    ROS_INFO_THROTTLE(0.1, "dist = %f yaw_e = %f, d_e = %f, error = %f, v_yaw = %f"
//                      , dist_to_start, yaw_error*RAD2DEGREE, d_error, error, v_yaw);
    publishVel(vel_x, v_yaw);

    pre_dist_to_goal = dist_to_goal;
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("move to pose out");
}

void Control::moveTo(double goal_wx, double goal_wy, double goal_yaw)
{
  double x, y, yaw;
//  getOdomPose(x, y, yaw);
  getPose(x, y, yaw);

  double dist_to_goal = dist(x, y, goal_wx, goal_wy);
  int last_left_encoder = left_encoder_;
  int last_right_encoder = right_encoder_;
  int left_encoder_sum = 0;
  int right_encoder_sum = 0;
  int encoder = int(dist_to_goal/(tick_to_rad*wheel_radius));

  double kp = 5, ki = 0.5, kd = 0.1, dt = 0.05;
  double pre_yaw_error = 0.0, pre_yaw_integral = 0.0;
  double yaw_error;
  double vel_x = 0.3;

  ros::Rate r(20);
  while(ros::ok())
  {
    if (left_encoder_sum > encoder || right_encoder_sum > encoder)
    {
      break;
    }

    left_encoder_sum += (left_encoder_ - last_left_encoder);
    right_encoder_sum += (right_encoder_ - last_right_encoder);

    ROS_INFO_THROTTLE(0.5, "left_encoder = %d, right_encoder = %d, "
                           "sum left = %d, sum right = %d, thread = %d", left_encoder_, right_encoder_
                            , left_encoder_sum, right_encoder_sum, encoder);

    last_left_encoder = left_encoder_;
    last_right_encoder = right_encoder_;


    yaw_error = angle_diff(goal_yaw, odom_pose_[2]);
    double v_yaw = pidController(kp, ki, kd, dt,
                                 pre_yaw_error, yaw_error, pre_yaw_integral);

    checkZValid(v_yaw, 0.6, 0.0);
    publishVel(vel_x, v_yaw);

    ros::spinOnce();
    r.sleep();
  }
}

void Control::rotateTo(double goal_yaw)
{
  ROS_INFO("rotateTo in");
  double kp = 5, ki = 0, kd = 0, dt = 0.01;
  std::string control_type = "PID";
  double pre_error = 0, pre_integral = 0;
  bool is_goal_reached = false;

  double x, y, yaw;
//  getOdomPose(x, y, yaw);
  getPose(x, y, yaw);
  ros::Rate r(20);
  double error = angle_diff(goal_yaw, yaw);
  double pre_yaw_error = std::fabs(error);
  int sgn = (error > 0) ? 1 : -1;
  while(/*!is_bumper && */!is_goal_reached && ros::ok())
  {
//    getOdomPose(x, y, yaw);
    getPose(x, y, yaw);
    error = angle_diff(goal_yaw, yaw);
    ROS_INFO_THROTTLE(0.5, "yaw_error = %f", error*RAD2DEGREE);
//    if (sgn == 1 && error < 0)
//    {
//      publishZeroVel();
//      is_goal_reached = true;
//      break;
//    }
//    else if (sgn == -1 && error > 0)
//    {
//      publishZeroVel();
//      is_goal_reached = true;
//      break;
//    }
    if (std::fabs(error) < 10*DEGREE2RAD)
    {
      if (std::fabs(pre_yaw_error) < std::fabs(error))
      {
        ROS_INFO("diff = %f", error*RAD2DEGREE);
        publishZeroVel();
        break;
      }
    }
    pre_yaw_error = error;

    double v_yaw = pidController(kp, ki, kd, dt, pre_error, error, pre_integral);
    checkZValid(v_yaw, 0.4, 0.2);
    publishVel(0, v_yaw);


    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("rotateTo out");
}

void Control::getPose(double &x, double &y, double &yaw)
{
  getOdomPose(x, y, yaw);
//  getTfPose(x, y, yaw);
}

void Control::getOdomPose(double &x, double &y, double &yaw)
{
  boost::unique_lock<boost::mutex> lock(control_mutex_);
  x = odom_pose_[0];
  y = odom_pose_[1];
  yaw = odom_pose_[2];
}

bool Control::getTfPose(double &x, double &y, double &yaw)
{
  tf::Stamped<tf::Pose> global_pose;
  global_pose.setIdentity();
  tf::Stamped<tf::Pose> robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_="base_footprint";
  robot_pose.stamp_=ros::Time();
  try
  {
    tf_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
    tf_.transformPose("map", robot_pose, global_pose);
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();
    yaw = tf::getYaw(global_pose.getRotation());
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  return true;
}

bool Control::isInCell(double wx, double wy)
{
  double x, y, yaw;
//  getOdomPose(x, y, yaw);
  getPose(x, y, yaw);

  double d = dist(x, y, wx, wy);
  ROS_INFO("isIncell robot(%f, %f), cell(%f, %f), d = %f", x, y, wx, wy, d);
  if (d < 0.2)
  {
    return true;
  }
  return false;
}

void Control::uTurn(TurnEnum td, double sx, double sy, double ex, double ey)
{

  double x, y, yaw;
//  getOdomPose(x, y, yaw);
  getPose(x, y, yaw);
  ROS_INFO("uTurn in yaw = %f", yaw*RAD2DEGREE);

  double delta_x = fabs(sx - ex);
  double delta_y = fabs(sy - ey);
  double radius;
  if (delta_x < delta_y)
  {// y direction u turn
    radius = fabs(ey - y) / 2;
  }
  else
  {// x direction u turn
    radius = fabs(ex - x) / 2;
  }
  ROS_INFO("radius = %f", radius);
  radius = (radius > 0.2) ? 0.1 : radius;

  double uturn_yaw = std::atan2(ey-sy, ex-sx);
  double goal_yaw;
  double turn_direction = 0.0;

  if (td == TURN_LEFT)
  {
    turn_direction = 1.0;
    goal_yaw = normalize(uturn_yaw + M_PI_2);
  }
  else if (td == TURN_RIGHT)
  {
    turn_direction = -1.0;
    goal_yaw = normalize(uturn_yaw - M_PI_2);
  }

  double yaw_eps = 20*DEGREE2RAD;
  double pre_yaw_error = M_PI;
  bool is_goal_reached = false;
  ros::Rate r(20);
  while(ros::ok()&& /*!is_bumper && */!is_goal_reached)
  {
    double curr_x, curr_y, curr_yaw;
//    getOdomPose(curr_x, curr_y, curr_yaw);
    getPose(curr_x, curr_y, curr_yaw);

    double yaw_error = angle_diff(goal_yaw, curr_yaw);
    if (fabs(yaw_error) < std::fabs(yaw_eps))
    {
      if (std::fabs(pre_yaw_error) < std::fabs(yaw_error))
      {
        publishZeroVel();
        break;
      }
    }

    double vel_z = turn_direction * 0.7;
    double vel_x = fabs(vel_z * radius);

    publishVel(vel_x, vel_z);

    pre_yaw_error = yaw_error;
    ros::spinOnce();
    r.sleep();
  }
//  ROS_INFO("uturn out yaw = %f", curr_yaw*RAD2DEGREE);
}

void Control::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  ROS_INFO_ONCE("Odom call back");
  boost::unique_lock<boost::mutex> lock(control_mutex_);
  odom_pose_[0] = msg->pose.pose.position.x;
  odom_pose_[1] = msg->pose.pose.position.y;
  odom_pose_[2] = tf::getYaw(msg->pose.pose.orientation);

  curr_vel_x = msg->twist.twist.linear.x;
  curr_vel_z = msg->twist.twist.angular.z;

//  ROS_INFO("odom pose :%f, %f, %f", odom_pose_[0], odom_pose_[1], odom_pose_[2]);
}

void Control::coreSensorCallback(const kobuki_msgs::SensorStateConstPtr &msg)
{
//  ROS_INFO("coreSensorCallback");
  boost::unique_lock<boost::mutex> lock(control_mutex_);
  sonar_dist_ = msg->analog_input[1] / 1000.0;
  left_encoder_ = msg->left_encoder;
  right_encoder_ = msg->right_encoder;
//  ROS_INFO("sonar_dist = %f, left %d, right = %d", sonar_dist_, left_encoder_, right_encoder_);
}

void Control::bumperCallback(const kobuki_msgs::BumperEventConstPtr &msg)
{
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    is_bumper = true;
  }
}

double Control::pidController(double kp, double ki, double kd, double dt
                              , double &pre_error, double error, double &pre_integral)
{
  double eps = 1e-2;
  double integral = 0, derivative = 0;
  double u = 0;
  std::string type = "PID";

  if (fabs(error) > eps)
    integral = pre_integral + error * dt;
  else
    integral = 0;

  derivative = (error - pre_error) / dt;

  if (type.compare("PID"))
    u = kp * error + ki * integral + kd * derivative;
  else if (type.compare("PI"))
    u = kp * error + ki * integral;
  else if (type.compare("PD"))
    u = kp * error + kd * derivative;
  else if (type.compare("P"))
    u = kp * error;

  pre_error = error;
  pre_integral = integral;
  return u;
}

void Control::publishVel(double x, double z)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = x;
  cmd_vel.angular.z = z;
  vel_pub_.publish(cmd_vel);
}

void Control::publishZeroVel()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub_.publish(cmd_vel);
}
