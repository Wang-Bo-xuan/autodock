#include <ls_autodock/move_controller.h>

LS_SLAM::MoveController::MoveController(ros::NodeHandle *nh)
{
  int type;
  double value;

  this->move_client = new Client("/LSStepController",true);
  this->move_client->waitForServer(ros::Duration(5.0));
}

LS_SLAM::MoveController::~MoveController()
{
  if(this->move_client)
  {
    delete this->move_client;
  }
  this->move_client = NULL;
}

void LS_SLAM::MoveController::init(void)
{

}

void LS_SLAM::MoveController::readConfig(void)
{

}

void LS_SLAM::MoveController::Move(const char ctrl_type,const double value,double time)
{
  ls_msgs::LSStepControllerGoal goal;
  goal.ctrl_type = ctrl_type;
  goal.value = value;
  this->move_client->sendGoalAndWait(goal,ros::Duration(time));
  bool done_before_timeout = this->move_client->waitForResult(ros::Duration(time));

  if(this->move_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    //ROS_INFO("SUCCSSED");
    return;
  }
  else if(this->move_client->getState() == actionlib::SimpleClientGoalState::ABORTED)
  {
    //ROS_INFO("ABORTED");
    return;
  }

  if(!done_before_timeout)
  {
    //ROS_INFO("timeout");
    return;
  }
}

void LS_SLAM::MoveController::Line(double value, double time)
{
  this->Move(ls_msgs::LSStepControllerGoal::line,value,time);
}

void LS_SLAM::MoveController::Rot(double value, double time)
{
  this->Move(ls_msgs::LSStepControllerGoal::rot,value,15.0);
}

