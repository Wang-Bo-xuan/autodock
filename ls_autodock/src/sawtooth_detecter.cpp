	#include <ls_autodock/sawtooth_detecter.h>

	LS_SLAM::SawtoothDetecter::SawtoothDetecter(ros::NodeHandle *nh)
	{
	  this->nh_ = nh;
	  this->sensor_sub_ = new ros::Subscriber();
	  *this->sensor_sub_ = this->nh_->subscribe("laser_sensor",1,&SawtoothDetecter::SensorCalback,this);
	  this->marker_pub_ = new ros::Publisher();
	  *this->marker_pub_ = this->nh_->advertise<visualization_msgs::Marker>("sawtooth_detecter_display",1000);
	  this->chargebank_pose_pub_ = new ros::Publisher();
	  *this->chargebank_pose_pub_ = this->nh_->advertise<ls_msgs::LSChargeBankPose>("chargebank_pose",10);
	  tf_ = new tf::TransformListener();

	  this->init();
	}

	LS_SLAM::SawtoothDetecter::~SawtoothDetecter()
	{
	  if(this->sensor_sub_)
	  {
	    delete this->sensor_sub_;
	  }
	  this->sensor_sub_ = NULL;

	  if(this->marker_pub_)
	  {
	    delete this->marker_pub_;
	  }
	  this->marker_pub_ = NULL;

	  if(this->chargebank_pose_pub_)
	  {
	    delete this->chargebank_pose_pub_;
	  }
	  this->chargebank_pose_pub_ = NULL;

	  if(this->laser_)
	  {
	    delete this->laser_;
	  }
	  this->laser_ = NULL;

	  if(this->laser_list_)
	  {
	    delete this->laser_list_;
	  }
	  this->laser_list_ = NULL;

	  if(this->line_list_)
	  {
	    delete this->line_list_;
	  }
	  this->line_list_ = NULL;

	  if(this->tf_)
	  {
	    delete this->tf_;
	  }
	  this->tf_ = NULL;
	}

	void LS_SLAM::SawtoothDetecter::init(void)
	{
	  this->ReadConfig();

	  this->laser_ = new vector<LASERLIST>;
	  this->laser_list_ = new LASERLIST;
	  this->line_list_ = new LINELIST;

	  this->x_ = 0;
	  this->y_ = 0;
	}

	void LS_SLAM::SawtoothDetecter::ReadConfig(void)
	{
	  ros::NodeHandle pnh("~");
	  pnh.param("line_k_threshold",this->line_k_threshold_,0.5);
	  pnh.param("line_b_threshold",this->line_b_threshold_,0.1);
	  pnh.param("first_baffle_length",this->first_baffle_length_,0.035);
	  pnh.param("second_baffle_length",this->second_baffle_length_,0.07);
	  pnh.param("third_baffle_length",this->third_baffle_length_,0.035);
	  pnh.param("baffle_length_difference",this->baffle_length_difference_,0.015);
	}

	void LS_SLAM::SawtoothDetecter::SensorCalback(const ls_msgs::LSMultiLaserSensor msg)
	{
	  int i,j;

	  this->laser_->clear();
	  for(i = 0;i < msg.multiSensor.size();i ++)
	  {
	    LASERLIST tt;
	    ls_msgs::LSLaserSensor laser_list_iterator = msg.multiSensor.at(i);
	    for(j = 0;j < laser_list_iterator.sensor.size();j ++)
	    {
	      LASER t;
	      t.index = laser_list_iterator.sensor.at(j).index;
	      t.angle = laser_list_iterator.sensor.at(j).angle;
	      t.angle_min = laser_list_iterator.sensor.at(j).angle_min;
	      t.angle_max = laser_list_iterator.sensor.at(j).angle_max;
	      t.range = laser_list_iterator.sensor.at(j).range;
	      t.range_min = laser_list_iterator.sensor.at(j).range_min;
	      t.range_max = laser_list_iterator.sensor.at(j).range_max;
	      t.intensity = laser_list_iterator.sensor.at(j).intensity;
	      t.x = laser_list_iterator.sensor.at(j).x;
	      t.y = laser_list_iterator.sensor.at(j).y;
	      t.z = laser_list_iterator.sensor.at(j).z;

	      tt.push_back(t);
	    }
	    this->laser_->push_back(tt);
	    tt.clear();
	  }

	  this->OSLFitting();
	  this->DetecterChargeBank();
	  this->Display();

	  this->laser_list_->clear();
	  this->line_list_->clear();
	}

	void LS_SLAM::SawtoothDetecter::OSLFitting(void)
	{
	  vector<LASERLIST>::iterator llt;
	  LASERLIST::iterator laser_iterator;
	  LINEFIT line;

	  //traversing each group
	  for(llt = this->laser_->begin();llt != this->laser_->end();llt ++)
	  {
	    double x_sum_ave = 0;
	    double y_sum_ave = 0;
	    double xx_sum = 0;
	    double xy_sum = 0;

	    //traversing each data in a group
	    for(laser_iterator = llt->begin();laser_iterator != llt->end();laser_iterator ++)
	    {
	      x_sum_ave += (*laser_iterator).y;
	      y_sum_ave += (*laser_iterator).x;
	      xx_sum += (*laser_iterator).y*(*laser_iterator).y;
	      xy_sum += (*laser_iterator).x*(*laser_iterator).y;
	    }

	    //calc the line paramters
	    int size = llt->size();
	    line.k = (size*xy_sum - x_sum_ave * y_sum_ave) / (size * xx_sum - x_sum_ave*x_sum_ave);
	    line.b = (xx_sum * y_sum_ave - x_sum_ave * xy_sum) / (size * xx_sum - x_sum_ave*x_sum_ave);
	    line.start_index_x = llt->front().x;
	    line.start_index_y = llt->front().y;
	    line.end_index_x = llt->back().x;
	    line.end_index_y = llt->back().y;
	    line.length = hypot(line.start_index_x-line.end_index_x,line.start_index_y-line.end_index_y);
	    this->line_list_->push_back(line);
	}

	  this->FilterLine();
	}

	void LS_SLAM::SawtoothDetecter::FilterLine(void)
	{
	  for(auto i = this->line_list_->begin();i != this->line_list_->end();)
	  {
	    if(std::isnan((*i).k) || std::isnan((*i).b) || std::isinf((*i).k) || std::isinf((*i).b))
	    {
	      i = this->line_list_->erase(i);
	      continue;
	    }
	    i ++;
	  }
	}

	void LS_SLAM::SawtoothDetecter::DetecterChargeBank(void)
	{
	  this->k_.clear();
	  this->b_.clear();

	  this->K_Fitting();
	  this->B_Fitting();
	  this->L_Fitting();
	}

	void LS_SLAM::SawtoothDetecter::K_Fitting(void)
	{
	  //calc the all group k-dif and store as a vector<LINELIST>
	  vector<LINELIST> line_k_result;
	  LINELIST::iterator k_first_iterator,k_second_iterator;
	  LINELIST line_k;
	  for(k_first_iterator = this->line_list_->begin();k_first_iterator != this->line_list_->end();k_first_iterator ++)
	  {
	    line_k.clear();
	    double cur_k = (*k_first_iterator).k;
	    for(k_second_iterator = k_first_iterator;k_second_iterator != this->line_list_->end();k_second_iterator ++)
	    {
	      double pre_k = (*k_second_iterator).k;
	      if(fabs(pre_k - cur_k) < this->line_k_threshold_)
	      {
		LINEFIT t;
		t.k = (*k_second_iterator).k;
		t.b = (*k_second_iterator).b;
		t.length = (*k_second_iterator).length;
		t.start_index_x = (*k_second_iterator).start_index_x;
		t.start_index_y = (*k_second_iterator).start_index_y;
		t.end_index_x = (*k_second_iterator).end_index_x;
		t.end_index_y = (*k_second_iterator).end_index_y;
		line_k.push_back(t);
	      }
	    }
	    line_k_result.push_back(line_k);
	  }

	  //find the max of linelist
	  int max_size = 0;
	  for(vector<LINELIST>::iterator finall_iterator = line_k_result.begin();finall_iterator != line_k_result.end();finall_iterator ++)
	  {
	    int cur_size = finall_iterator->size();
	    if(cur_size > max_size)
	    {
	      max_size = cur_size;
	    }
	  }

	  //find the max index of linelist
	  vector<LINELIST>::iterator finall_iterator;
	  for(finall_iterator = line_k_result.begin();finall_iterator->size() != max_size;finall_iterator ++);

	  line_k.clear();
	  //copy the data from max index of linelist to line_k for b-fitting and length-fitting
	  for(LINELIST::iterator lineiterator = finall_iterator->begin();lineiterator != finall_iterator->end();lineiterator ++)
	  {
	    this->k_.push_back(*lineiterator);
	  }
	}

	void LS_SLAM::SawtoothDetecter::B_Fitting(void)
	{
	  //calc the all group b-dif and store as a vector<LINELIST>
	  vector<LINELIST> line_b_result;
	  LINELIST::iterator b_first_iterator,b_second_iterator;
	  LINELIST line_b;
	  for(b_first_iterator = this->k_.begin();b_first_iterator != this->k_.end();b_first_iterator ++)
	  {
	    line_b.clear();
	    double cur_b = (*b_first_iterator).b;
	    for(b_second_iterator = b_first_iterator;b_second_iterator != this->k_.end();b_second_iterator ++)
	    {
	      double pre_b = (*b_second_iterator).b;
	      if(fabs(pre_b - cur_b) < this->line_b_threshold_)
	      {
		LINEFIT t;
		t.k = (*b_second_iterator).k;
		t.b = (*b_second_iterator).b;
		t.length = (*b_second_iterator).length;
		t.start_index_x = (*b_second_iterator).start_index_x;
		t.start_index_y = (*b_second_iterator).start_index_y;
		t.end_index_x = (*b_second_iterator).end_index_x;
		t.end_index_y = (*b_second_iterator).end_index_y;
		line_b.push_back(t);
	      }
	    }
	    line_b_result.push_back(line_b);
	  }

	  //find the max of linelist
	  int max_size = 0;
	  for(vector<LINELIST>::iterator finall_iterator = line_b_result.begin();finall_iterator != line_b_result.end();finall_iterator ++)
	  {
	    int cur_size = finall_iterator->size();
	    if(cur_size > max_size)
	    {
	      max_size = cur_size;
	    }
	  }

	  //find the max index of linelist
	  vector<LINELIST>::iterator finall_iterator;
	  for(finall_iterator = line_b_result.begin();finall_iterator->size() != max_size;finall_iterator ++);

	  line_b.clear();
	  //copy the data from max index of linelist to line_k for b-fitting and length-fitting
	  for(LINELIST::iterator lineiterator = finall_iterator->begin();lineiterator != finall_iterator->end();lineiterator ++)
	  {
	    this->b_.push_back(*lineiterator);
	  }
	}

	void LS_SLAM::SawtoothDetecter::L_Fitting(void)
	{
	  int match_num = 0;
	  if(this->b_.size() >= 3)
	  {
	    this->chargebank_.clear();
	    for(LINELIST::iterator lineiterator = this->b_.begin();lineiterator != this->b_.end();lineiterator ++)
	    {
	      switch(match_num)
	      {
	      case 0:
		if(fabs((*lineiterator).length - this->first_baffle_length_) < this->baffle_length_difference_)
		{
		  this->chargebank_.push_back(*lineiterator);
		  match_num ++;
		}
		else
		{
		  this->chargebank_.clear();
		  match_num = 0;
		}
		break;
	      case 1:
		if(fabs((*lineiterator).length - this->second_baffle_length_) < this->baffle_length_difference_)
		{
		  this->chargebank_.push_back(*lineiterator);
		  match_num ++;
		}
		else
		{
		  this->chargebank_.clear();
		  match_num = 0;
		}
		break;
	      case 2:
		if(fabs((*lineiterator).length - this->third_baffle_length_) < this->baffle_length_difference_)
		{
		  this->chargebank_.push_back(*lineiterator);
		  this->CalcChargeBankPose();
		}
		else
		{
		  this->chargebank_.clear();
		  match_num = 0;
		}
		break;
	      }
	    }
	  }
	}

	void LS_SLAM::SawtoothDetecter::CalcChargeBankPose(void)
	{
	  if(this->chargebank_.size() == 3)
	  {
	    double left_x,left_y,right_x,right_y;
	    double delta_x,delta_y;
	    LINELIST::iterator t,t1,t2,t3;

	    t = this->chargebank_.begin();
	    t1 = t++;
	    t2 = t++;
	    t3 = t++;

	    left_x = (*t1).end_index_x;
	    left_y = (*t1).end_index_y;
	    right_x = (*t3).start_index_x;
	    right_y = (*t3).start_index_y;

	    delta_x = left_x - right_x;
	    delta_y = right_x - right_y;

	    double k = 0;
	    for(LINELIST::iterator line_iterator = this->chargebank_.begin();line_iterator != this->chargebank_.end();line_iterator ++)
	    {
	      k += (*line_iterator).k;
	    }
	    k /= 4;

	    this->x_ = (left_x+right_x)/2;
	    this->y_ = (left_y+right_y)/2;
	    this->angle_ = atan2(k,1.0);

	    if (k < 0)
	    {
	      this->angle_ = fabs(3.14159/2-this->angle_);
	      this->angle_ = this->angle_ - 3.14159;
	    }
	    else
	    {
	      this->angle_ = -fabs(angle_)-3.14159/2;
	      this->angle_ = this->angle_ + 3.14159;
	    }

	    if(this->angle_ > 0)
	    {
	      this->angle_ = fabs(this->angle_)+1.57;
	    }
	    else
	    {
	      this->angle_ = this->angle_-1.57;
	    }

	    geometry_msgs::PoseStamped base,laser;
	    Laser2Base(&laser,&base,this->x_,this->y_,this->angle_);

            //ROS_INFO("laser_x:%lf laser_y:%lf laser_yaw:%lf",laser.pose.position.x,laser.pose.position.y,tf::getYaw(laser.pose.orientation));
	    //ROS_ERROR("base_x:%lf base_y:%lf base_yaw:%lf",base.pose.position.x,base.pose.position.y,tf::getYaw(base.pose.orientation));

	    ls_msgs::LSChargeBankPose bank_pose;
	    bank_pose.x = base.pose.position.x;
	    bank_pose.y = base.pose.position.y;
	    bank_pose.yaw = tf::getYaw(base.pose.orientation);;
	    this->chargebank_pose_pub_->publish(bank_pose);
	  }
	}

void LS_SLAM::SawtoothDetecter::Laser2Base(geometry_msgs::PoseStamped *from, geometry_msgs::PoseStamped *to,double x,double y,double yaw)
{
  from->pose.position.x = x;
  from->pose.position.y = y;
  tf::Quaternion qua = tf::createQuaternionFromRPY(0,0,yaw);
  from->pose.orientation.x = qua.getX();
  from->pose.orientation.y = qua.getY();
  from->pose.orientation.z = qua.getZ();
  from->pose.orientation.w = qua.getW();

  from->header.stamp = ros::Time();
  from->header.frame_id = "laser_link";
  try
  {
    tf_->waitForTransform("base_footprint", "laser_link", ros::Time(), ros::Duration(5.0));
    tf_->transformPose("base_footprint", *from,*to);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void LS_SLAM::SawtoothDetecter::Display(void)
{
  LASERLIST::iterator laserlist_iterator;
  vector<LASERLIST>::iterator llt;
  LINELIST::iterator line_iterator;
  geometry_msgs::Point p;

  //display the laser datas
  visualization_msgs::Marker marker_point;
  marker_point.header.frame_id = "base_laser_link";
  marker_point.header.stamp = ros::Time::now();
  marker_point.ns = "point";
  marker_point.action = visualization_msgs::Marker::ADD;
  marker_point.pose.orientation.w = 1.0f;
  marker_point.id = 1;
  marker_point.type = visualization_msgs::Marker::POINTS;
  marker_point.scale.x = 0.001f;
  marker_point.scale.y = 0.001f;
  marker_point.color.r = 0.0f;
  marker_point.color.g = 1.0f;
  marker_point.color.b = 0.0f;
  marker_point.color.a = 1.0f;

  for(llt = this->laser_->begin();llt != this->laser_->end();llt ++)
  {
    for(laserlist_iterator = llt->begin();laserlist_iterator != llt->end();laserlist_iterator ++)
    {
      p.x = (*laserlist_iterator).x;
      p.y = (*laserlist_iterator).y;
      p.z = (*laserlist_iterator).z;

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

  int i = 3;
  double p_step = 0.01;
  //display the line datas
  for(line_iterator = this->chargebank_.begin();line_iterator != this->chargebank_.end();line_iterator ++)
  {
    visualization_msgs::Marker marker_line;
    marker_line.header.frame_id = "base_laser_link";
    marker_line.header.stamp = ros::Time::now();
    marker_line.ns = "charge";
    marker_line.action = visualization_msgs::Marker::ADD;
    marker_line.pose.orientation.w = 1.0f;
    marker_line.id = i++;
    marker_line.type = visualization_msgs::Marker::LINE_STRIP;
    marker_line.scale.x = 0.005f;
    marker_line.color.r = 1.0f;
    marker_line.color.g = 0.0f;
    marker_line.color.b = 0.0f;
    marker_line.color.a = 0.5f;
    //y may be opposite
    if((*line_iterator).end_index_y < (*line_iterator).start_index_y)
    {
      p_step = -fabs(p_step);
    }
    else
    {
      p_step = fabs(p_step);
    }
    //traversing each point on the line
    for(p.y = (*line_iterator).start_index_y;p.y < (*line_iterator).end_index_y;p.y += p_step)
    {
      p.x = p.y * (*line_iterator).k + (*line_iterator).b;
      p.z = 0;
      //nan and inf datas does not need to be displayed
      if(!std::isnan(p.x) && !std::isnan(p.y))
      {
        marker_line.points.push_back(p);
      }
      else
      {
        ROS_ERROR("draw line : error x or y");
      }
    }
    this->marker_pub_->publish(marker_line);
  }

  visualization_msgs::Marker marker_target;
  marker_target.header.frame_id = "base_laser_link";
  marker_target.header.stamp = ros::Time::now();
  marker_target.ns = "target";
  marker_target.action = visualization_msgs::Marker::ADD;
  marker_target.pose.position.x = this->x_;
  marker_target.pose.position.y = this->y_;
  marker_target.pose.position.z = 0;
  tf::Quaternion qua = tf::createQuaternionFromYaw(this->angle_);
  marker_target.pose.orientation.x = qua.getX();
  marker_target.pose.orientation.y = qua.getY();
  marker_target.pose.orientation.z = qua.getZ();
  marker_target.pose.orientation.w = qua.getW();
  marker_target.id = 1;
  marker_target.type = visualization_msgs::Marker::ARROW;
  marker_target.scale.x = 0.1f;
  marker_target.scale.y = 0.01f;
  marker_target.color.r = 1.0f;
  marker_target.color.g = 1.0f;
  marker_target.color.b = 1.0f;
  marker_target.color.a = 1.0f;
  this->marker_pub_->publish(marker_target);

  p_step = 0.01;
  //display the line datas
  for(line_iterator = this->line_list_->begin();line_iterator != this->line_list_->end();line_iterator ++)
  {
    visualization_msgs::Marker marker_line;
    marker_line.header.frame_id = "base_laser_link";
    marker_line.header.stamp = ros::Time::now();
    marker_line.ns = "line";
    marker_line.action = visualization_msgs::Marker::ADD;
    marker_line.pose.orientation.w = 1.0f;
    marker_line.id = i++;
    marker_line.type = visualization_msgs::Marker::LINE_STRIP;
    marker_line.scale.x = 0.005f;
    marker_line.color.r = 0.0f;
    marker_line.color.g = 0.0f;
    marker_line.color.b = 1.0f;
    marker_line.color.a = 0.5f;
    //y may be opposite
    if((*line_iterator).end_index_y < (*line_iterator).start_index_y)
    {
      p_step = -fabs(p_step);
    }
    else
    {
      p_step = fabs(p_step);
    }
    //traversing each point on the line
    for(p.y = (*line_iterator).start_index_y;p.y < (*line_iterator).end_index_y;p.y += p_step)
    {
      p.x = p.y * (*line_iterator).k + (*line_iterator).b;
      p.z = 0;
      //nan and inf datas does not need to be displayed
      if(!std::isnan(p.x) && !std::isnan(p.y))
      {
        marker_line.points.push_back(p);
      }
      else
      {
        ROS_ERROR("draw line : error x or y");
      }
    }
    this->marker_pub_->publish(marker_line);
  }
}

using namespace LS_SLAM;
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"ls_detecter_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20.0);

  SawtoothDetecter detecter(&nh);

  while(ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
