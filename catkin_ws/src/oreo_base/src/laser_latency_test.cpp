
#include <cstdlib>
#include <sstream>
#include <cmath>
#include<iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#define laser_size 540
double laser_range[540];
void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  ros::Time begin_time = ros::Time::now(); 
  double t1 = begin_time.toSec();
  for(int i=0;i<540;i++){
    laser_range[i] = msg->ranges[i];
  }
  ros::Time end_time = ros::Time::now();
  double t2 = end_time.toSec();
  ROS_INFO("time need to copy %d laser beams = %lf",laser_size,t2-t1);
  ROS_INFO("laser processing time = %lf s",msg->scan_time);
}
void InitialLaser(double laser[],int size){
  for(int i=0;i<size;i++){
    laser[i] = -1;
  }
}
void ShowLaser(double laser[],int size){
  ROS_INFO("-------------------------------");
  for(int i=0;i<size;i++){
    //ROS_INFO("laser %d = %lf",i,laser[i]);
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_latency_test");
  ros::NodeHandle n;
  //ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);  
  ros::Subscriber sub_laser = n.subscribe("scan",1000,LaserCallback);
  ros::Rate loop_rate(1);
  //InitialLaser(laser_range,laser_size);
  //ROS_INFO("try obstacles avoidance method(use class to access laser");
	while (ros::ok())
  {   
		//pub_cmd_vel.publish(current_velocity);
		//ShowLaser(laser_range,laser_size);
		ros::spinOnce();
    loop_rate.sleep();		
	}
	return 0;
}

