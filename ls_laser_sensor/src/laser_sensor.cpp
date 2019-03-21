#include <ros/ros.h>
#include <ls_laser_sensor/laser_sensor.h>

LS_SLAM::LaserSensor::LaserSensor(ros::NodeHandle *nh)
{
  this->laser_sub_ = new ros::Subscriber();
  *this->laser_sub_ = nh->subscribe("scan",1,&LaserSensor::LaserCallback,this);
  this->sensor_pub_ = new ros::Publisher();
  *this->sensor_pub_ = nh->advertise<ls_msgs::LSMultiLaserSensor>("laser_sensor",1000);
  this->marker_pub_ = new ros::Publisher();
  *this->marker_pub_ = nh->advertise<visualization_msgs::Marker>("laser_sensor_display",1000);

  this->init();
}

LS_SLAM::LaserSensor::~LaserSensor()
{
  if(this->laser_sub_)
  {
    delete this->laser_sub_;
  }
  this->laser_sub_ = NULL;

  if(this->sensor_pub_)
  {
    delete this->sensor_pub_;
  }
  this->sensor_pub_ = NULL;

  if(this->marker_pub_)
  {
    delete this->marker_pub_;
  }
  this->marker_pub_ = NULL;

  if(this->laser_list_)
  {
    delete this->laser_list_;
  }
  this->laser_list_ = NULL;
}

void LS_SLAM::LaserSensor::init(void)
{
  this->readConfig();

  this->state_ = true;
  this->laser_list_ = new LASERLIST();
}

void LS_SLAM::LaserSensor::readConfig(void)
{
  ros::NodeHandle pnh("~");
  pnh.param("laser_min_range",this->laser_min_range_,0.1);
  pnh.param("laser_process_group",this->laser_process_group_,4);
  pnh.param("laser_spread",this->laser_spread_,1);
  pnh.param("laser_group_threshold",this->laser_group_threshold_,0.01);
  pnh.param("laser_filter",this->laser_filter_,1);
  pnh.param("laser_fitler_threshold",this->laser_fitler_threshold_,3);
}

void LS_SLAM::LaserSensor::LaserCallback(const sensor_msgs::LaserScan &msg)
{
  static int laser_init_flag = 0;
  int i;

  //init the laser paramter,only one time
  if(0 == laser_init_flag)
  {
    laser_init_flag = 1;
    this->laser_num_ = msg.ranges.size();
    this->angle_increment_ = msg.angle_increment;
  }

  //init the laser datas in a ls_msgs::LSLaserSensor message
  for(i = 0;i < this->laser_num_;i ++)
  {
    LASER t;
    t.index = i;
    t.angle_min = msg.angle_min;
    t.angle_max = msg.angle_max;
    t.angle = msg.angle_increment*i+msg.angle_min;
    t.range_min = msg.range_min;
    t.range_max = msg.range_max;
    t.range = msg.ranges[i];
    if(std::isnan(t.range))
    {
      t.range = 0;
    }
    if(std::isinf(t.range) || t.range < this->laser_min_range_)
    {
      t.range = 0;
    }
    t.x = t.range*cos(t.angle);
    t.y = t.range*sin(t.angle);
    t.z = 0;
    if(0 == msg.intensities.size())
    {
      t.intensity = 0;
    }
    else
    {
      t.intensity = msg.intensities[i];
    }
    this->laser_list_->push_back(t);
  }

  //laser_process
  this->LaserProcess();

  if(this->state_)
  {
    //publish
    this->SensorPublish();

    //display for debug
    this->Display();
  }

  this->laser_.clear();
  this->laser_list_->clear();
}

void LS_SLAM::LaserSensor::LaserProcess(void)
{
  this->LaserFilter();
  this->LaserMix();
  this->PointCloudSpread();
  this->GroupingLaser();
}

void LS_SLAM::LaserSensor::LaserFilter(void)
{
  int i = 0;
  LASERLIST::iterator laser_iterator;

  for(laser_iterator = this->laser_list_->begin();laser_iterator != this->laser_list_->end();laser_iterator ++,i ++)
  {
    LASER t;
    t.index = (*laser_iterator).index;
    t.angle_min = (*laser_iterator).angle_min;
    t.angle_max = (*laser_iterator).angle_max;
    t.angle = (*laser_iterator).angle;
    t.range_min = (*laser_iterator).range_min;
    t.range_max = (*laser_iterator).range_max;
    t.range = (*laser_iterator).range;
    t.intensity = (*laser_iterator).intensity;
    t.x = (*laser_iterator).x;
    t.y = (*laser_iterator).y;
    t.z = (*laser_iterator).z;

    if(t.range != 0)
    {
      this->laser_temp_[i/(this->laser_list_->size()/this->laser_process_group_)].push_back(t);
    }
  }

  this->laser_list_->clear();
}

void LS_SLAM::LaserSensor::LaserMix(void)
{
  int i;
  LASERLIST::iterator laser_iterator;

  for(i = 0;i < this->laser_process_group_;i ++)
  {
    if(i == 3)
    {
      for(laser_iterator = this->laser_temp_[i].begin();laser_iterator != this->laser_temp_[i].end();laser_iterator ++)
      {
        this->laser_list_->push_back(*laser_iterator);
      }
    }
  }
  for(i = 0;i < this->laser_process_group_;i ++)
  {
    if(i == 0)
    {
      for(laser_iterator = this->laser_temp_[i].begin();laser_iterator != this->laser_temp_[i].end();laser_iterator ++)
      {
        this->laser_list_->push_back(*laser_iterator);
      }
    }
  }

  for(i = 0;i < this->laser_process_group_;i ++)
  {
    this->laser_temp_[i].clear();
  }
}

void LS_SLAM::LaserSensor::PointCloudSpread(void)
{
  static int flag = 0;

  if(0 == flag)
  {
    if(1 == this->laser_spread_)
    {
      flag = 1;

      ROS_INFO("PointCloudSpreading ...");
      if(ANGLE_DIF(this->angle_increment_,1.0,0.1))
      {
        ROS_WARN("Lidar-Angular-Resolution is 1.0 degree,a Quad-Spread will come...");
      }
      else if(ANGLE_DIF(this->angle_increment_,0.5,0.1))
      {
        ROS_WARN("Lidar-Angular-Resolution is 0.5 degree,a Double-Spread will come...");
      }
      else if(ANGLE_DIF(this->angle_increment_,0.25,0.1))
      {
        ROS_WARN("Lidar-Angular-Resolution is 0.25 degree,not need to Spread...");
      }
      else
      {
        ROS_WARN("a invalid Lidar-Angular-Resolution,please check you lidar!");
        this->state_ = false;
      }
    }
  }

  if(1 == this->laser_spread_)
  {
    if(ANGLE_DIF(this->angle_increment_,1.0,0.1))
    {
      this->PointCloudQuadSpread();
    }
    else if(ANGLE_DIF(this->angle_increment_,0.5,0.1))
    {
      this->PointCloudDoubleSpread();
    }
    else if(ANGLE_DIF(this->angle_increment_,0.25,0.1))
    {
      ;
    }
    else
    {
      this->state_ = false;
    }
  }
}

void LS_SLAM::LaserSensor::PointCloudDoubleSpread(void)
{
  LASERLIST temp1,temp2;
  LASERLIST::iterator laser_iterator;

  for(laser_iterator = this->laser_list_->begin();laser_iterator != this->laser_list_->end();laser_iterator ++)
  {
    LASER t;
    t.index = (*laser_iterator).index;
    t.angle_min = (*laser_iterator).angle_min;
    t.angle_max = (*laser_iterator).angle_max;
    t.range_min = (*laser_iterator).range_min;
    t.range_max = (*laser_iterator).range_max;
    t.angle = (*laser_iterator).angle;
    t.range = (*laser_iterator).range;
    t.intensity = (*laser_iterator).intensity;
    t.x = (*laser_iterator).x;
    t.y = (*laser_iterator).y;
    t.z = 0;
    temp1.push_back(t);
  }
  for(laser_iterator = temp1.begin();laser_iterator != temp1.end();)
  {
    LASERLIST::iterator t = laser_iterator;
    laser_iterator ++;

    if(laser_iterator != temp1.end())
    {
      LASER t1;
      t1.index = (*laser_iterator).index;
      t1.angle_min = (*laser_iterator).angle_min;
      t1.angle_max = (*laser_iterator).angle_max;
      t1.range_min = (*laser_iterator).range_min;
      t1.range_max = (*laser_iterator).range_max;
      t1.angle = ((*laser_iterator).angle + (*t).angle)/2;
      t1.range = ((*laser_iterator).range + (*t).range)/2;
      t1.intensity = ((*laser_iterator).intensity  + (*t).intensity)/2;
      t1.x = ((*laser_iterator).x + (*t).x)/2;
      t1.y = ((*laser_iterator).y + (*t).y)/2;
      t1.z = 0;
      temp2.push_back(t1);
    }
  }

  this->laser_list_->clear();
  LASERLIST::iterator l1 = temp1.begin(),l2 = temp2.begin();
  for(int i = 0;i < temp1.size()+temp2.size();i ++)
  {
    LASER t1;

    if(0 == i % 2)
    {
      t1.index = i;
      t1.angle_min = (*l1).angle_min;
      t1.angle_max = (*l1).angle_max;
      t1.range_min = (*l1).range_min;
      t1.range_max = (*l1).range_max;
      t1.angle = (*l1).angle;
      t1.range = (*l1).range;
      t1.intensity = (*l1).intensity;
      t1.x = (*l1).x;
      t1.y = (*l1).y;
      t1.z = 0;

      l1 ++;
    }
    else
    {
      t1.index = i;
      t1.angle_min = (*l2).angle_min;
      t1.angle_max = (*l2).angle_max;
      t1.range_min = (*l2).range_min;
      t1.range_max = (*l2).range_max;
      t1.angle = (*l2).angle;
      t1.range = (*l2).range;
      t1.intensity = (*l2).intensity;
      t1.x = (*l2).x;
      t1.y = (*l2).y;
      t1.z = 0;

      l2 ++;
    }

    this->laser_list_->push_back(t1);
  }
}

void LS_SLAM::LaserSensor::PointCloudQuadSpread(void)
{
  this->PointCloudDoubleSpread();
  this->PointCloudDoubleSpread();
}

void LS_SLAM::LaserSensor::GroupingLaser(void)
{
  this->CalcLaserGroup();

  //group the lase datas into vector,each member is belong the same group
  int i = 0,j = 0;
  int dis_arr_num = this->tmp_list_.size();
  this->laser_.clear();

  LASERLIST llt;
  for(LASERLIST::iterator laser_iterator = this->laser_list_->begin();laser_iterator != this->laser_list_->end();laser_iterator ++,i ++)
  {
    LASER t;
    t.index = (*laser_iterator).index;
    t.angle_max = (*laser_iterator).angle_max;
    t.angle_min = (*laser_iterator).angle_min;
    t.range_max = (*laser_iterator).range_max;
    t.range_min = (*laser_iterator).range_min;
    t.angle = (*laser_iterator).angle;
    t.range = (*laser_iterator).range;
    t.intensity = (*laser_iterator).intensity;
    t.x = (*laser_iterator).x;
    t.y = (*laser_iterator).y;
    t.z = (*laser_iterator).z;
    llt.push_back(t);

    //put other members into another group when the threshold is out
    if(j < dis_arr_num)
    {
      if(i == this->tmp_list_.at(j))
      {
        j ++;
        this->laser_.push_back(llt);
        llt.clear();
      }
    }
  }
  this->laser_.push_back(llt);

  this->FilterGroup();
}

void LS_SLAM::LaserSensor::CalcLaserGroup(void)
{
  this->FilterInfNanZeroPointFromSensor();

  //calc the group num,can not be zero
  this->laser_group_ = 1;
  this->tmp_list_.clear();
  int i;
  LASERLIST::iterator laser_iterator = this->laser_list_->begin();

  for(i = 0;i < this->laser_list_->size()-1;i ++)
  {
    LASERLIST::iterator llit = laser_iterator;
    llit ++;

    double x = (*laser_iterator).x;
    double y = (*laser_iterator).y;
    double next_x = (*llit).x;
    double next_y = (*llit).y;
    double dis = hypot(next_x-x,next_y-y);

    double range = (*laser_iterator).range;
    double angle = 0.25*3.14159/180;
    this->laser_group_threshold_ = 2*range*sin(angle);

    if(dis > this->laser_group_threshold_)
    {
      this->laser_group_ ++;
      this->tmp_list_.push_back(i);
    }
    laser_iterator ++;
  }
}

void LS_SLAM::LaserSensor::FilterInfNanZeroPointFromSensor(void)
{
  //when the ranges data eq nan,inf or zero,it is invaild,we must delete it in order to calc the real group num
  for(LASERLIST::iterator laser_iterator = this->laser_list_->begin();laser_iterator != this->laser_list_->end();)
  {
    if(std::isinf((*laser_iterator).range) || std::isnan((*laser_iterator).range) || fabs((*laser_iterator).range) < DBL_EPSILON)
    {
      this->laser_list_->erase(laser_iterator++);
    }
    else
    {
      laser_iterator ++;
    }
  }
}

void LS_SLAM::LaserSensor::FilterGroup(void)
{
  for(auto i = this->laser_.begin(); i != this->laser_.end();)
  {
    if (i->size() <= this->laser_fitler_threshold_)
    {
      i = this->laser_.erase(i);
      continue;
    }
    i++;
  }
}

void LS_SLAM::LaserSensor::SensorPublish()
{
  vector<LASERLIST>::iterator vlaser_list_iteartor;
  ls_msgs::LSMultiLaserSensor ls_multi_sensor;

  for(vlaser_list_iteartor = this->laser_.begin();vlaser_list_iteartor != this->laser_.end();vlaser_list_iteartor ++)
  {
    LASERLIST::iterator laser_list_iterator;
    ls_msgs::LSLaserSensor ls_sensor;

    for(laser_list_iterator = vlaser_list_iteartor->begin();laser_list_iterator != vlaser_list_iteartor->end();laser_list_iterator ++)
    {
      ls_msgs::LSLaserScan ls_scan;
      ls_scan.index = (*laser_list_iterator).index;
      ls_scan.angle = (*laser_list_iterator).angle;
      ls_scan.angle_min = (*laser_list_iterator).angle_min;
      ls_scan.angle_max = (*laser_list_iterator).angle_max;
      ls_scan.range = (*laser_list_iterator).range;
      ls_scan.range_min = (*laser_list_iterator).range_min;
      ls_scan.range_max = (*laser_list_iterator).range_max;
      ls_scan.intensity = (*laser_list_iterator).intensity;
      ls_scan.x = (*laser_list_iterator).x;
      ls_scan.y = (*laser_list_iterator).y;
      ls_scan.z = (*laser_list_iterator).z;

      ls_sensor.sensor.push_back(ls_scan);
    }
    ls_multi_sensor.multiSensor.push_back(ls_sensor);
  }
  this->sensor_pub_->publish(ls_multi_sensor);
}

void LS_SLAM::LaserSensor::Display()
{
  double intensity;
  vector<LASERLIST>::iterator vlaser_list_iteartor;
  LASERLIST::iterator laserlist_iterator;
  geometry_msgs::Point p;

  visualization_msgs::Marker marker_point;
  marker_point.header.frame_id = "base_laser_link";
  marker_point.header.stamp = ros::Time::now();
  marker_point.ns = "point";
  marker_point.action = visualization_msgs::Marker::ADD;
  marker_point.pose.orientation.w = 1.0f;
  marker_point.id = 1;
  marker_point.type = visualization_msgs::Marker::POINTS;
  marker_point.scale.x = 0.002f;
  marker_point.scale.y = 0.002f;
  marker_point.color.r = 0.0f;
  marker_point.color.g = 1.0f;
  marker_point.color.b = 0.0f;
  marker_point.color.a = 1.0f;

  for(vlaser_list_iteartor = this->laser_.begin();vlaser_list_iteartor != this->laser_.end();vlaser_list_iteartor ++)
  {
    for(laserlist_iterator = vlaser_list_iteartor->begin();laserlist_iterator != vlaser_list_iteartor->end();laserlist_iterator ++)
    {
      p.x = (*laserlist_iterator).x;
      p.y = (*laserlist_iterator).y;
      p.z = (*laserlist_iterator).z;
      intensity = (*laserlist_iterator).intensity;

      if(!std::isnan(p.x) && !std::isnan(p.y))
      {
        marker_point.points.push_back(p);
      }
      else
      {
        ROS_ERROR("draw point : error x or y");
      }
    }
  }
  this->marker_pub_->publish(marker_point);
}

using namespace LS_SLAM;
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"ls_laser_sensor_process_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);

  LaserSensor ls_sensor(&nh);

  while(ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
