#include <ls_controller/control.h>
#include <ls_controller/ls_step_controller.h>

LS_SLAM::LSStepController::LSStepController(ros::NodeHandle *nh)
{
  this->init();

  this->server_ = new Server(*nh,"/LSStepController",boost::bind(&LSStepController::execute,this,_1),false);
  this->server_->registerPreemptCallback(boost::bind(&LSStepController::preempt,this));
  this->server_->start();

  this->control_ = new Control();

  this->odom_sub_ = new ros::Subscriber();
  *this->odom_sub_ = nh->subscribe("odom",1,&LSStepController::OdomCallBack,this);

  this->vel_pub_ = nh->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",10);
}

LS_SLAM::LSStepController::~LSStepController()
{
  if(this->server_)
  {
    this->server_->shutdown();
    delete this->server_;
  }
  this->server_ = NULL;

  if(this->control_)
  {
    delete this->control_;
  }
  this->control_ = NULL;

  if(this->odom_sub_)
  {
    delete this->odom_sub_;
  }
  this->odom_sub_ = NULL;
}

void LS_SLAM::LSStepController::init()
{

}

void LS_SLAM::LSStepController::execute(const ls_msgs::LSStepControllerGoalConstPtr &goal)
{
  ls_msgs::LSStepControllerResult res;

  double origin_x,origin_y,origin_yaw;
  double target_x,target_y,target_yaw;
  double delta_yaw,delta_x;
ROS_ERROR("odom_x:%lf odom_y:%lf odom_yaw:%lf",this->x_,this->y_,this->yaw_);
  if(goal->ctrl_type == ls_msgs::LSStepControllerGoal::line)
  {
    ROS_INFO("line mode:");
ROS_INFO("testestes yaw:%lf",this->yaw_);

    origin_x = this->x_;
    origin_y = this->y_;
    origin_yaw = this->yaw_;
    delta_yaw = origin_yaw;
    delta_x = goal->value;
    target_x = origin_x+delta_x*cos(delta_yaw);
    target_y = origin_y+delta_x*sin(delta_yaw);
    target_yaw = origin_yaw;

    this->control_->moveToPose(target_x,target_y,target_yaw);

    if(this->server_->isActive())
    {
      res.res_type = res.successed;
      this->server_->setSucceeded(res);
      ROS_INFO("action succeeded");
      return;
    }
  }
  else if(goal->ctrl_type == ls_msgs::LSStepControllerGoal::rot)
  {
    ROS_INFO("rot mode:");

    origin_yaw = this->yaw_;
    target_yaw = origin_yaw+goal->value,delta_yaw = angle_diff(target_yaw,origin_yaw);

    this->control_->moveToPose(this->x_,this->y_,target_yaw);

    if(this->server_->isActive())
    {
      res.res_type = res.successed;
      this->server_->setSucceeded(res);
      ROS_INFO("action succeeded");
      return;
    }
  }
  else
  {
    ROS_ERROR("no this action mode!");
    return ;
  }


  /*
  ls_msgs::LSStepControllerResult res;
  ls_msgs::LSStepControllerFeedback feed;
  geometry_msgs::Twist vel_cmd;

  double origin_x,origin_y,origin_yaw;
  double target_x,target_y,target_yaw;

  origin_x = this->x_;
  origin_y = this->y_;
  origin_yaw = this->yaw_;

  double ang = goal->value;
  if(goal->ctrl_type == ls_msgs::LSStepControllerGoal::line)
  {
    if(goal->value*180/3.14159 < 1)
    {
      ang = 0;
    }
    target_x = origin_x + ang*cos(origin_yaw);
    target_y = origin_y + ang*sin(origin_yaw);
    target_yaw = origin_yaw;
    ROS_INFO("get goal for (x:%lf,y:%lf) on odom",target_x,target_y);

    if(hypot(target_x-this->x_,target_y-this->y_) < this->line_min_range_ || hypot(target_x-this->x_,target_y-this->y_) > this->line_max_range_)
    {
      ROS_WARN("In line mode, the Step Controller value (x) should be in (%.2lf , %.2lf)",this->line_min_range_,this->line_max_range_);
      res.res_type = res.failed;
      this->server_->setAborted(res);
      ROS_WARN("action Failed");
      return;
    }

    bool arrive = false;
    while(!arrive)
    {
      double dis = hypot(target_x-this->x_,target_y-this->y_);
      if(target_x < this->x_)
      {
        dis = -dis;
      }

      if(dis > this->move_min_step_length_)
      {
        vel_cmd.linear.x = dis*this->length_pid_control_param_p_;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;

        double angle = target_yaw - this->yaw_;
        if(fabs(angle) > this->move_min_step_angle_)
        {
          vel_cmd.angular.x = 0;
          vel_cmd.angular.y = 0;
          vel_cmd.angular.z = angle*this->angle_pid_control_param_p_;
        }
      }
      else
      {
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
      }
      vel_pub_->publish(vel_cmd);

      if(hypot(target_x-this->x_,0) <= this->move_min_step_length_)
      {
        arrive = true;

        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
        vel_pub_->publish(vel_cmd);
      }
    }

    feed.feed_type = feed.good;
    this->server_->publishFeedback(feed);

    if(this->server_->isActive())
    {
      res.res_type = res.successed;
      this->server_->setSucceeded(res);
      ROS_INFO("action succeeded");
      return;
    }
  }
  else if(goal->ctrl_type == ls_msgs::LSStepControllerGoal::rot)
  {
    target_yaw = origin_yaw + goal->value;
    ROS_INFO("get goal for (yaw:%lf) on odom",target_yaw);

    if(goal->value < this->rot_min_angle_ || goal->value > this->rot_max_angle_)
    {
      ROS_WARN("In rot mode, the Step Controller value (yaw) should be in (%.2lf , %.2lf)",this->rot_min_angle_,this->rot_max_angle_);
      res.res_type = res.failed;
      this->server_->setAborted(res);
      ROS_WARN("action Failed");
      return;
    }

    bool arrive = false;
    while(!arrive)
    {
      double a,b,d1,d2;
      a = atan2(sin(target_yaw),cos(target_yaw));
      b = atan2(sin(this->yaw_),cos(this->yaw_));
      d1 = a-b;
      d2 = 2*3.14159-fabs(d1);
      if(d1 > 0)
      {
        d2 *= -1.0;
      }
      if(fabs(d1) < fabs(d2))
      {
        ang = d1;
      }
      else
      {
        ang = d2;
      }

      if(ang > this->move_min_step_angle_ || ang < this->move_max_step_angle_)
      {
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = ang*this->angle_pid_control_param_p_;
      }
      else
      {
        arrive = true;

        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
      }
      vel_pub_->publish(vel_cmd);
    }

    feed.feed_type = feed.good;
    this->server_->publishFeedback(feed);

    if(this->server_->isActive())
    {
      res.res_type = res.successed;
      this->server_->setSucceeded(res);
      ROS_INFO("action succeeded");
      return;
    }
  }*/
}

void LS_SLAM::LSStepController::preempt(void)
{
  if(this->server_->isActive())
  {
    server_->setPreempted();
  }
}

void LS_SLAM::LSStepController::OdomCallBack(const nav_msgs::Odometry msg)
{
  this->yaw_ = tf::getYaw(msg.pose.pose.orientation);
  this->x_ = msg.pose.pose.position.x;
  this->y_ = msg.pose.pose.position.y;
}

using namespace LS_SLAM;
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"ls_step_controller_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);

  LSStepController step_controller(&nh);

  while(ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
