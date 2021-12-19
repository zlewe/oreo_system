#include <stdio.h>
#include <curses.h>
#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "time.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
//#include "bayes_tracking/multitracker.h"
#include "navigator_maths.h"
#include <vector>
#include <mutex>
//#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
using std::mutex;
using std::lock_guard;
using std::vector;
using geometry_msgs::Pose2D;
using sensor_msgs::LaserScan;
using namespace std;
//using namespace MTRK;
using namespace cv;

int main(int argc, char **argv){ 
  ros::init(argc, argv, "keyboard_controller");
  geometry_msgs::Twist robot_velocity;
  ros::NodeHandle n;
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Rate loop_rate(20);
  int isOver = 0;
  unsigned char key,pre_key;
  while (ros::ok()){
    WINDOW *w = initscr();

    
    
    //cout << robot_velocity.angular.z <<endl;
    //printf("key: %c",key);
    if(isOver)
			break;
    robot_velocity.linear.x = 0;
    robot_velocity.linear.y = 0;
    robot_velocity.angular.z = 0;
 //   ros::Duration(0.5).sleep();     // wait for a instance
  //  pub_cmd_vel.publish(robot_velocity); //TIME-DELAY
    key = getch();
    //cout << key <<endl;
    //if(key==pre_key)
    
      
    switch(key)
    {
      case 'w': //up
        robot_velocity.linear.x = 200;
        //cout <<"up"<< robot_velocity.linear.x <<endl;
      break;
      case 's': //down
        robot_velocity.linear.x = -200;
        //cout <<"down"<< robot_velocity.linear.x <<endl;
      break;  
      case 'a': //left
        robot_velocity.linear.y = 200;
        //cout <<"left"<< robot_velocity.linear.y <<endl;
      break;
      case 'q': //left up
        robot_velocity.linear.y = 200;
        robot_velocity.linear.x = 200;
        //cout <<"left"<< robot_velocity.linear.y <<endl;
      break;
      case 'd'://right  
        robot_velocity.linear.y = -200;
        //cout <<"right"<< robot_velocity.linear.y <<endl;
      break;
      case 'e'://right up
        robot_velocity.linear.y = -200;
        robot_velocity.linear.x = 200;
		robot_velocity.angular.z = -0.2;
        //cout <<"right"<< robot_velocity.linear.y <<endl;
      break;
      case 'z'://left down
        robot_velocity.linear.y = 200;
        robot_velocity.linear.x = -200;
        //cout <<"right"<< robot_velocity.linear.y <<endl;
      break;
      case 'c'://right down
        robot_velocity.linear.y = -200;
        robot_velocity.linear.x = -200;
        //cout <<"right"<< robot_velocity.linear.y <<endl;
      break;
			case 'm':
				robot_velocity.angular.z = -0.4;
				break;
	  
      case 0x20:
        robot_velocity.angular.z = 0.4 ;
        //cout <<"rotate"<<robot_velocity.angular.z <<endl;
      break;
      case 'o':
			robot_velocity.linear.x = 0;
			robot_velocity.linear.y = 0;
			robot_velocity.angular.z = 0;
			isOver = 1;
		case 't':
				robot_velocity.linear.x= 200;
				robot_velocity.linear.y= -200;
				robot_velocity.angular.z = 0.5;
				break;
      default:
        robot_velocity.linear.x = 0;
        robot_velocity.linear.y = 0;
        robot_velocity.angular.z = 0;
      break;
    }
    pub_cmd_vel.publish(robot_velocity);
    ros::spinOnce();
    loop_rate.sleep();     
    pre_key = key; 
  }
  endwin();
  return 0;
}
