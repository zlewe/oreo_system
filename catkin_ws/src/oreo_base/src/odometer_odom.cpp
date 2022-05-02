#include "ros/ros.h"
#include <fstream>
#include "oreo_base/EncoderPair.h"
#include <mutex>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#define Rad2Deg 57.295779513
#define Deg2Rad (1/Rad2Deg)
using namespace std;
fstream odo_file;
#include "geometry_msgs/Pose2D.h"

class Odometer{
public:
  Odometer();
  ~Odometer();
  void UpdateOdometry(const oreo_base::EncoderPair::ConstPtr& msg);
  void GetOdometry(geometry_msgs::Pose2D* msg);
  void SetPrev_Begin_Time(double t){prev_begin_time = t ;prev_end_time = t;}
  void LoadParameters();
  double ComputeEncoderRate(int64_t encoder_counts, int64_t &diff, int64_t &prev, double begin_time, double &prev_begin_time);
  //double Get_x_vel();
 // double Get_y_vel();
  //double Get_ang_vel();
  
  
double Get_x_vel()
{
  return x_vel;
}
double Get_y_vel()
{
  return y_vel;  
}
double Get_ang_vel()
{
  return ang_vel;
}
private:
  void NormalizeDeg(double* ang);
  bool odo_updated_;
  bool param_loaded_;
  std::mutex odo_mutex_;
  geometry_msgs::Pose2D odometry_;
  int64_t prev_h_encoder_counts_;
  int64_t prev_r_encoder_counts_;
  int64_t prev_l_encoder_counts_;
  double robot_radius_, wheel_diameter_, geer_ratio_, right_wheel_counts_per_rev_, horizontal_wheel_counts_per_rev_ , left_wheel_counts_per_rev_;
  double a_balance_, t_balance_, l_balance_;
  double encoder_update_time_;
  double begin_time;
  double prev_begin_time;
  double prev_h_begin_time;
  double prev_r_begin_time;
  double prev_l_begin_time;
  double end_time;
  double prev_end_time;
  double ang_vel, x_vel, y_vel;
};

Odometer::Odometer():prev_h_encoder_counts_(0),prev_r_encoder_counts_(0),prev_l_encoder_counts_(0),odo_updated_(true),param_loaded_(false){}

Odometer::~Odometer(){};

void Odometer::LoadParameters() {
  param_loaded_ = true;
  if(!ros::ok())
  {
    ROS_ERROR("Odometer::LoadParameters(): Cannot access ROS system. Please make sure ROS system is initialized and is running.");
    param_loaded_ = false;
    return;
  }
  if (!ros::param::get("/oreo_base/robot_radius", robot_radius_))
  {
    robot_radius_ = 0.225;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/robot_radius cannot be retrieved, using default(%f m)", robot_radius_);
    param_loaded_ = false;
  }
  if (!ros::param::get("/oreo_base/wheel_diameter", wheel_diameter_))
  {
    wheel_diameter_ = 0.152;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/wheel_diameter cannot be retrieved, using default(%f m)", wheel_diameter_);
    param_loaded_ = false;
  }
  if (!ros::param::get("/oreo_base/geer_ratio", geer_ratio_))
  {
    geer_ratio_ = 43;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/geer_ratio cannot be retrieved, using default(%f)", geer_ratio_);
    param_loaded_ = false;
  }
  if (!ros::param::get("/oreo_base/horizontal_wheel_counts_per_rev", horizontal_wheel_counts_per_rev_))
  {
    horizontal_wheel_counts_per_rev_ = 176128;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/horizontal_wheel_counts_per_rev cannot be retrieved, using default(%f counts/rev)", horizontal_wheel_counts_per_rev_);
    param_loaded_ = false;
  }
  if (!ros::param::get("/oreo_base/right_wheel_counts_per_rev", right_wheel_counts_per_rev_))
  {
    right_wheel_counts_per_rev_ = 176128;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/right_wheel_counts_per_rev cannot be retrieved, using default(%f counts/rev)", right_wheel_counts_per_rev_);
    param_loaded_ = false;
  }
  if (!ros::param::get("/oreo_base/left_wheel_counts_per_rev", left_wheel_counts_per_rev_))
  {
    left_wheel_counts_per_rev_ = 176128;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/left_wheel_counts_per_rev cannot be retrieved, using default(%f counts/rev)", left_wheel_counts_per_rev_);
    param_loaded_ = false;
  }
  if (!ros::param::get("/oreo_base/a_balance", a_balance_))
  {
    a_balance_ = 1.01;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/a_balance cannot be retrieved, using default(%f)", a_balance_);
    param_loaded_ = false;
  }
  if (!ros::param::get("/oreo_base/t_balance", t_balance_))
  {
    t_balance_ = 1;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/t_balance cannot be retrieved, using default(%f)", t_balance_);
    param_loaded_ = false;
  }
  if (!ros::param::get("/oreo_base/l_balance", l_balance_))
  {
    l_balance_ = 1;
    ROS_WARN("Odometer::LoadParameters(): /oreo_base/l_balance cannot be retrieved, using default(%f)", l_balance_);
    param_loaded_ = false;
  }
}
//why? 360 or 180
void Odometer::NormalizeDeg(double * ang) {
  while ( *ang > 180 || *ang <= -180 ) {
    if (*ang > 180) { *ang -= 2 * 180; }
    if (*ang <= -180) { *ang += 2 * 180; }
  }
}

void Odometer::GetOdometry(geometry_msgs::Pose2D* msg) {
  odo_mutex_.lock();
  *msg = odometry_;
  odo_mutex_.unlock();
}

double Odometer::ComputeEncoderRate(int64_t encoder_counts, int64_t &diff, int64_t &prev, double begin_time, double &prev_begin_time){
  if (encoder_counts != -1){
    diff = encoder_counts-prev;
    
    double rate;
    rate = (double)diff/(begin_time-prev_begin_time);
    
    prev_begin_time = begin_time;
    prev = encoder_counts;

    return rate;
  }
  else{
    return 0.0;
  }
}

void Odometer::UpdateOdometry(const oreo_base::EncoderPair::ConstPtr& msg) 
{
  if (!param_loaded_) 
  {
    ROS_WARN("Odometer::UpdateOdometry(): Parameters haven't been initialized yet. Odometer may not be able to calculate correct values");
  }
  geometry_msgs::Pose2D prev_odo;
  odo_mutex_.lock();
  prev_odo = odometry_;
  odo_mutex_.unlock();

  //std::ofstream log("log.txt", std::ios::out);
  //std::ofstream odo_log("odo_log.txt", std::ios::out);

  double linear_d, /*ang_vel,*/ avg_theta , diff_count;
  int64_t h_encoder_counts_diff, r_encoder_counts_diff, l_encoder_counts_diff;

  ROS_INFO("[Subscribe Encoder] a: %" PRId64 ", b: %" PRId64 ", c: %" PRId64 ,msg->horizontal, msg->right , msg->left);
  int64_t h_encoder_counts = msg->horizontal;
  int64_t r_encoder_counts = msg->right;
  int64_t l_encoder_counts = msg->left;

  // if(h_encoder_counts != -1 || r_encoder_counts != -1 || l_encoder_counts != -1) 
  // {
  //   if(!odo_updated_) 
  //   {
  //     odo_updated_ = true;
  //     h_encoder_counts_diff = r_encoder_counts_diff = l_encoder_counts_diff = 0;
  //     prev_h_encoder_counts_ = 0;
  //     prev_r_encoder_counts_ = 0;
  //     prev_l_encoder_counts_ = 0;
  //   } 
  //   else 
  //   {
  //     //log << r_encoder_counts << ' ' << h_encoder_counts << std::endl;
  //     UpdateEncoder(h_encoder_counts, h_encoder_counts_diff, prev_h_encoder_counts_, msg.timestamp, prev_h_begin_time);
  //     UpdateEncoder(l_encoder_counts, l_encoder_counts_diff, prev_l_encoder_counts_, msg.timestamp, prev_l_begin_time);
  //     UpdateEncoder(r_encoder_counts, r_encoder_counts_diff, prev_r_encoder_counts_, msg.timestamp, prev_r_begin_time);
  //     // r_encoder_counts_diff = r_encoder_counts - prev_r_encoder_counts_;
  //     // h_encoder_counts_diff = h_encoder_counts - prev_h_encoder_counts_;
  //     // l_encoder_counts_diff = l_encoder_counts - prev_l_encoder_counts_;

  //     // prev_r_encoder_counts_ = r_encoder_counts;
  //     // prev_h_encoder_counts_ = h_encoder_counts;
  //     // prev_l_encoder_counts_ = l_encoder_counts;
  //   }
  // } 
  // else 
  // {
  //   odo_updated_ = false;
  // }

  double h_encoder_rate = ComputeEncoderRate(h_encoder_counts, h_encoder_counts_diff, prev_h_encoder_counts_, msg->timestamp, prev_h_begin_time);
  double l_encoder_rate = ComputeEncoderRate(l_encoder_counts, l_encoder_counts_diff, prev_l_encoder_counts_, msg->timestamp, prev_l_begin_time);
  double r_encoder_rate = ComputeEncoderRate(r_encoder_counts, r_encoder_counts_diff, prev_r_encoder_counts_, msg->timestamp, prev_r_begin_time);

  //kinematics
  begin_time = ros::Time::now().toSec();
  // encoder_update_time_ = begin_time - prev_begin_time;
  // double h_vel = (double)h_encoder_counts_diff / horizontal_wheel_counts_per_rev_ * wheel_diameter_ * M_PI / encoder_update_time_;
  // double r_vel = (double)r_encoder_counts_diff / right_wheel_counts_per_rev_ * wheel_diameter_ * M_PI / encoder_update_time_;
  // double l_vel = (double)l_encoder_counts_diff / left_wheel_counts_per_rev_ * wheel_diameter_ * M_PI / encoder_update_time_;
  double h_vel = (double)h_encoder_rate / horizontal_wheel_counts_per_rev_ * wheel_diameter_ * M_PI;
  double r_vel = (double)r_encoder_rate / right_wheel_counts_per_rev_ * wheel_diameter_ * M_PI;
  double l_vel = (double)l_encoder_rate / left_wheel_counts_per_rev_ * wheel_diameter_ * M_PI;
  
  x_vel = (r_vel - l_vel) / sqrt(3) * 0.97;
  y_vel = (r_vel + l_vel - 2 * h_vel) / 3 * 0.97;
  //odo_file <<h_vel <<" "<<r_vel<<" "<<l_vel<<endl;
/*
  if(h_vel == 0 || r_vel == 0 || l_vel == 0)
    ang_vel = 0;
  else
    ang_vel = (h_vel + r_vel + l_vel) / (-3) / robot_radius_ / M_PI * 180;
*/
    
  ang_vel = (h_vel + r_vel + l_vel) / (3) / robot_radius_ * 0.95;

  prev_begin_time = begin_time;
  
  end_time = ros::Time::now().toSec();
  diff_count = end_time - prev_end_time;
  //odo_file << "theta :" <<ang_vel * diff_count / M_PI * 180<<endl;
/*
  if( fabs(ang_vel * diff_count / M_PI * 180) < 0.01)
    ang_vel = 0; 
*/ 
  avg_theta = prev_odo.theta*Deg2Rad + (ang_vel * diff_count )/2;//not average /2
  prev_odo.x += x_vel * diff_count * cos(avg_theta) - y_vel * diff_count * sin(avg_theta);
  prev_odo.y += y_vel * diff_count * cos(avg_theta) + x_vel * diff_count * sin(avg_theta);
  //odo_file << "theta :" <<ang_vel * diff_count / M_PI * 180<<endl;
  prev_odo.theta += ang_vel * diff_count / M_PI * 180;
  NormalizeDeg( &prev_odo.theta );
  prev_end_time = ros::Time::now().toSec();
/*
  double ldis = (double)r_encoder_counts_diff / left_wheel_counts_per_rev_ * wheel_diameter_ * M_PI * t_balance_ * l_balance_;
  double rdis = (double)right_encoder_counts_diff / right_wheel_counts_per_rev_ * wheel_diameter_ * M_PI * t_balance_;
  delta_theta = ( rdis - ldis ) / 2 / robot_radius_* a_balance_ ;
  avg_theta = prev_odo.theta*Deg2Rad + delta_theta / 2;
  double r_plus_b = ( ldis + rdis ) / fabs( rdis - ldis ) * robot_radius_;
  if( fabs( r_plus_b ) < 10 ){
    linear_d = r_plus_b * fabs(sin( delta_theta / 2 )) * 2 ;
  }
  else{
    linear_d = (ldis + rdis) / 2;
  }

  prev_odo.x += linear_d * cos( avg_theta );
  prev_odo.y += linear_d * sin( avg_theta );
  prev_odo.theta = prev_odo.theta + delta_theta * Rad2Deg; // Convert to degree
  NormalizeDeg( &prev_odo.theta );
*/

  odo_mutex_.lock();
  odometry_ = prev_odo;
  odo_mutex_.unlock();

  //odo_log << prev_odo.x << ' ' << prev_odo.y << ' ' << prev_odo.theta << std::endl;
}









int main(int argc, char **argv) {
  ros::init(argc, argv, "odometer");
  ros::NodeHandle n;

  Odometer odo;
  odo.LoadParameters();
  
  ros::Publisher odometry_pub = n.advertise<geometry_msgs::Pose2D>("odometry", 1000);
  
  //insert for the need of navigation stack
  ros::Publisher odometry_odm_pub = n.advertise<nav_msgs::Odometry>("odometry_odm", 50);
  tf::TransformBroadcaster odom_broadcaster;
  
  ros::Subscriber encoder_sub = n.subscribe("encoder", 1, &Odometer::UpdateOdometry, &odo);
  odo.SetPrev_Begin_Time(ros::Time::now().toSec());
  ros::Rate loop_rate(100);
  
  odo_file.open("/home/robot/odo.txt",ios::out);
  odo_file << "hi Prof Fu" << endl;
  while (ros::ok()) {
    ros::spinOnce();
    geometry_msgs::Pose2D msg;
    odo.GetOdometry(&msg);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(msg.theta*3.14159/180);
    
    
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = msg.x;
    odom_trans.transform.translation.y = msg.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = msg.x;
    odom.pose.pose.position.y = msg.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = odo.Get_x_vel();
    odom.twist.twist.linear.y = odo.Get_y_vel();
    odom.twist.twist.angular.z = odo.Get_ang_vel();

    //publish the message
    odometry_odm_pub.publish(odom);
    
    
    
    
    
    
  
  
  
    odometry_pub.publish(msg);
    ROS_INFO("[Publish Odometry] x: %f y: %f theta: %f", msg.x, msg.y, msg.theta);
    odo_file <<"[Publish Odometry] x:"<<msg.x<<" y:" << msg.y <<" theta:" << msg.theta <<endl;
    
    loop_rate.sleep();
  }
  odo_file.close();
  return 0;
}

