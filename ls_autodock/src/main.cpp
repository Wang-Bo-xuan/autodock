#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <ls_autodock/sawtooth_detecter.h>
#include <ls_autodock/move_controller.h>
#include <ls_msgs/LSChargeBankPose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <unistd.h>
#include <time.h>

namespace LS_SLAM
{
class LS_AutoDock
{
public:
    LS_AutoDock(ros::NodeHandle *nh);
    ~LS_AutoDock();

    void ChargeBankCallBack(const ls_msgs::LSChargeBankPose msg);
    void AutoDockCMDCallBack(const std_msgs::Int8 msg);
    void Dock(void);

    bool dock_state_;
    bool dock_finish_;
private:
    double normalize(double z);
    double angle_diff(double a, double b);
    void init(void);

    ros::Subscriber chargebank_sub_,dock_cmd_sub_;
    ros::Publisher vel_pub_,dock_result_pub_;

    MoveController *mover_;
    double x_,y_,yaw_;
    int flag_;
    bool detecter_start_;
};

LS_AutoDock::LS_AutoDock(ros::NodeHandle *nh)
{
    this->mover_ = new MoveController(nh);
    this->vel_pub_ = nh->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",10);
    this->chargebank_sub_ = nh->subscribe("/ls_autodock/chargebank_pose",1,&LS_AutoDock::ChargeBankCallBack,this);
    this->dock_cmd_sub_ = nh->subscribe("/ls_autodock/command",1,&LS_AutoDock::AutoDockCMDCallBack,this);
    this->dock_result_pub_ = nh->advertise<std_msgs::Int8>("/ls_autodock/result",10);

    this->init();
}

LS_AutoDock::~LS_AutoDock()
{
    if(this->mover_)
    {
        delete this->mover_;
    }
    this->mover_ = NULL;
}

void LS_AutoDock::init(void)
{
    this->x_ = 0;
    this->y_ = 0;
    this->yaw_ = 0;

    this->flag_ = 0;

    this->detecter_start_ = true;
    this->dock_state_ = false;
    this->dock_finish_ = false;
}

void LS_AutoDock::ChargeBankCallBack(const ls_msgs::LSChargeBankPose msg)
{
    if(this -> detecter_start_)
    {
        this->x_ = msg.x;
        this->y_ = msg.y-0.05;
        this->yaw_ = msg.yaw;
    }
}

void LS_AutoDock::AutoDockCMDCallBack(const std_msgs::Int8 msg)
{
    if(0 == msg.data)
    {
        this->dock_state_ = true;
    }
    else
    {
        this->dock_state_ = false;
        this->dock_finish_ = true;
        this->flag_ =  3;
        this->Dock();
    } 
}

void LS_AutoDock::Dock(void)
{
    static int rot_num = 0,dock_init = 0,dock_stop = 0;
    geometry_msgs::Twist vel_cmd;
    geometry_msgs::Pose2D charge,fixpoint,base2fix,fix2charge;
    std_msgs::Int8 docker_result; 
    double rot1,line1,rot2,safe_distance = 0.5;
    static time_t init_now,init_last,stop_now,stop_last;

    switch(this->flag_)
    {
    case 0:
        if(0 == dock_init)
        { 
          dock_init = 1;
          time(&init_last);
        }
        
        this->detecter_start_ = true;
	ROS_ERROR("state 0");
        if(this->x_ != 0 && this->y_ != 0)
        {
            vel_cmd.angular.z = 0;
	    this->vel_pub_.publish(vel_cmd);
	    ROS_ERROR("x:%lf y:%lf yaw:%lf",this->x_,this->y_,this->yaw_);
	    sleep(2);
	    ros::spinOnce();
	    ROS_INFO("x:%lf y:%lf yaw:%lf",this->x_,this->y_,this->yaw_);
            dock_init = 0;
	    this->detecter_start_ = false;
            this->flag_ = 1;
        }
        else
        {
            time(&init_now);
            if(difftime(init_now,init_last) > 30)
	    {
              dock_init = 0;
	      this->flag_ = 0;
              this->dock_state_ = false;
              this->dock_finish_ = false;
              vel_cmd.angular.z = 0;
	      docker_result.data = 2; //can't find the charge bank,docing failed
              this->dock_result_pub_.publish(docker_result);
            }
	    else
            {
              vel_cmd.angular.z = 3.14159/6;
            }
            this->vel_pub_.publish(vel_cmd);
	}
    break;
    case 1:
	ROS_ERROR("state 1");
        charge.x = this->x_;
        charge.y = this->y_;
        charge.theta = this->yaw_;

        fixpoint.x = charge.x+safe_distance*cos(charge.theta);
        fixpoint.y = charge.y+safe_distance*sin(charge.theta);
        fixpoint.theta = charge.theta + 3.14159;

        base2fix.x = 0;
        base2fix.y = 0;
        base2fix.theta = atan2(fixpoint.y,fixpoint.x);

        fix2charge.x = fixpoint.x;
        fix2charge.y = fixpoint.y;
        fix2charge.theta = base2fix.theta;

        rot1 = angle_diff(base2fix.theta,0);
        line1 = hypot(fixpoint.x,fixpoint.y);
        rot2 = angle_diff(fixpoint.theta,fix2charge.theta);

        this->mover_->Rot(rot1,15.0);
	ROS_INFO("rot1:%lf",rot1);
        this->mover_->Line(line1,15.0);
	ROS_INFO("line1:%lf",line1);
        this->mover_->Rot(rot2,15.0);
	ROS_INFO("rot2:%lf",rot2);
        this->flag_ = 2;
    break;
    case 2:
	ROS_ERROR("state 2");
        this->detecter_start_ = true;
        this->x_ = this->y_ = this->yaw_ = 0;
	sleep(2);
        ros::spinOnce();
        this->detecter_start_ = false;
	if(fabs(this->y_) < 0.030 && fabs(this->yaw_) < 5*3.14159/180)
	{
	  this->mover_->Rot(3.05,15);
	  this->flag_ = 3;
	}
	else
	{
	  this->flag_ = 1;
	}
    break;
    case 3:
	ROS_ERROR("state 3");

        if(dock_finish_ == true)
        {
          docker_result.data = 0;  //docking successed
          this->dock_result_pub_.publish(docker_result);
          vel_cmd.linear.x = 0;
	  dock_stop = 0;
	  this->flag_ = 0;
	  this->dock_finish_ = false;
        }
        else
        {
          if(0 == dock_stop)
          {
	    dock_stop = 1;
            time(&stop_last);
          }
          time(&stop_now);
          if(difftime(stop_now,stop_last) > 10)
          {
	    docker_result.data = 3; //out of time ,docking failed
            this->dock_result_pub_.publish(docker_result);
            vel_cmd.linear.x = 0;
            dock_stop = 0;
	    this->dock_state_ = false;
	    this->dock_finish_ = false;
	    this->flag_ = 0;
          }
	  else
	  {
	    vel_cmd.linear.x = -0.05;
	  }
        }
        this->vel_pub_.publish(vel_cmd);
    break;
    default:
        ;
    }
}

double LS_AutoDock::normalize(double z)
{
    return atan2(sin(z),cos(z));
}

double LS_AutoDock::angle_diff(double a, double b)
{
    double d1, d2;
    a = this->normalize(a);
    b = this->normalize(b);
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
    {
        d2 *= -1.0;
    }
    if(fabs(d1) < fabs(d2))
    {
        return (d1);
    }
    else
    {
        return (d2);
    }
}
}
using namespace LS_SLAM;

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"ls_autodock_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    LS_AutoDock docker(&nh);

    while(ros::ok())
    {
        if(docker.dock_state_)
        {
          docker.Dock();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
