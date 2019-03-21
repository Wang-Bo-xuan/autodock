#ifndef __INCLUDE_LS_AUTODOCK_MOVE_CONTROLLER_H__
#define __INCLUDE_LS_AUTODOCK_MOVE_CONTROLLER_H__

#include <ros/ros.h>
#include <ls_msgs/LSStepControllerAction.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;

typedef actionlib::SimpleActionClient<ls_msgs::LSStepControllerAction> Client;

namespace LS_SLAM
{
  class MoveController
  {
    public:
      MoveController(ros::NodeHandle *nh);
      ~MoveController();

      void Line(double value,double time);
      void Rot(double value,double time);

    private:
      void init(void);
      void readConfig(void);
      void Move(const char ctrl_type,const double value,double time);
      
      Client *move_client;
  };
}

#endif
