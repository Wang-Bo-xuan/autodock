#ifndef __INCLUDE_LS_AUTODOCK_SAWTOOTH_DETECTER_H__
#define __INCLUDE_LS_AUTODOCK_SAWTOOTH_DETECTER_H__

#include <ros/ros.h>
#include <ls_autodock/common.h>
#include <ls_laser_sensor/common.h>
#include <ls_msgs/LSLaserScan.h>
#include <ls_msgs/LSLaserSensor.h>
#include <ls_msgs/LSChargeBankPose.h>
#include <ls_msgs/LSMultiLaserSensor.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

namespace LS_SLAM
{
  class SawtoothDetecter
  {
  public:
    SawtoothDetecter(ros::NodeHandle *nh);
    ~SawtoothDetecter();

  private:
    void init(void);
    void ReadConfig(void);
    void SensorCalback(const ls_msgs::LSMultiLaserSensor msg);
    void OSLFitting(void);
    void FilterLine(void);
    void DetecterChargeBank(void);
    void K_Fitting(void);
    void B_Fitting(void);
    void L_Fitting(void);
    void CalcChargeBankPose(void);
    void Laser2Base(geometry_msgs::PoseStamped *from,geometry_msgs::PoseStamped *to,double x,double y,double angle);
    void Display(void);

    ros::NodeHandle *nh_;
    ros::Subscriber *sensor_sub_;
    ros::Publisher *marker_pub_;
    ros::Publisher *chargebank_pose_pub_;
    tf::TransformListener *tf_;

    vector<LASERLIST> *laser_;
    LASERLIST *laser_list_;
    LINELIST *line_list_;
    LINELIST k_,b_;
    LINELIST chargebank_;
    double x_,y_,angle_;

    double line_k_threshold_;
    double line_b_threshold_;
    double first_baffle_length_;
    double second_baffle_length_;
    double third_baffle_length_;
    double baffle_length_difference_;
  };
}

#endif
