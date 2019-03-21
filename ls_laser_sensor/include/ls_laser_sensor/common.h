#ifndef __INCLUDE_LS_LASER_SENSOR_COMMON_H__
#define __INCLUDE_LS_LASER_SENSOR_COMMON_H__

#include <iostream>
#include <list>
#include <vector>
using namespace std;

namespace LS_SLAM
{
  typedef struct laser_struct
  {
    int index;
    double angle;
    double angle_min;
    double angle_max;
    double range;
    double range_min;
    double range_max;
    double intensity;
    double x;
    double y;
    double z;
  }LASER,*PLASER;

  typedef list<LASER> LASERLIST;
  typedef list<PLASER> PLASERLIST;
}

#endif
