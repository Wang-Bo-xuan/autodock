#ifndef __INCLUDE_LS_AUTODOCK_LASER_SENSOR_H__
#define __INCLUDE_LS_AUTODOCK_LASER_SENSOR_H__

#include <ros/ros.h>
#include <ls_msgs/LSLaserScan.h>
#include <ls_msgs/LSLaserSensor.h>
#include <ls_msgs/LSMultiLaserSensor.h>
#include <sensor_msgs/LaserScan.h>
#include <ls_laser_sensor/common.h>
#include <visualization_msgs/Marker.h>

#define PI 3.14159
#define DEG2RAD(X) ((X)*PI/180)
#define RAD2DEG(X) ((X)*180/PI)
#define ANGLE_DIF(X,Y,Z) (fabs((X)-DEG2RAD(Y))<DEG2RAD(Z))

namespace LS_SLAM
{
  class LaserSensor
  {
    public:
      LaserSensor(ros::NodeHandle *nh);
      ~LaserSensor();

    private:
      void init(void);
      void readConfig(void);
      void LaserCallback(const sensor_msgs::LaserScan &msg);
      void FilterInfNanZeroPointFromSensor(void);
      void CalcLaserGroup(void);
      void GroupingLaser(void);
      void FilterGroup(void);
      void LaserProcess(void);
      void LaserFilter(void);
      void LaserMix(void);
      void PointCloudSpread(void);
      void PointCloudQuadSpread(void);
      void PointCloudDoubleSpread(void);
      void SensorPublish(void);
      void Display(void);

      int laser_num_;
      int laser_group_;
      double angle_increment_;
      bool state_;
      vector<int> tmp_list_;
      vector<LASERLIST> laser_;
      LASERLIST *laser_list_,laser_temp_[4];

      ros::Subscriber *laser_sub_;
      ros::Publisher *sensor_pub_;
      ros::Publisher *marker_pub_;

      double laser_min_range_;
      int laser_process_group_;
      int laser_spread_;
      double laser_group_threshold_;
      int laser_filter_;
      int laser_fitler_threshold_;
  };
}

#endif
