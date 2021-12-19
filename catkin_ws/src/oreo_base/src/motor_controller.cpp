#include <fcntl.h>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <ctime>
#include <cmath>
#include <math.h>
#include <cstring>
#include <termios.h>

#include "ros/ros.h"
#include "oreo_base/EncoderPair.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

using namespace std;
fstream motor;
#define MAX_COM 6

#define Rad2Deg 57.295779513
#define Deg2Rad (1/Rad2Deg)
double HCOM;
double LCOM;
double RCOM;

double robot_radius_;  //0.1525 
double wheel_diameter_; //0.12366 #the wheel diameter is 5 inchs. 1 inch = 0.0254 meters.
double geer_ratio_; // 43 
double odo_theta;

static int l_com_fd, r_com_fd, h_com_fd;

void loadParameters()
{
  if(!ros::ok())
  {
    ROS_ERROR("loadParameters(): Cannot access ROS system. Please make sure ROS system is initialized and is running.");
    return;
  }
  //TODO: Fix variable names to fit coding style.
  if (!ros::param::get("/oreo_base/h_com", HCOM))
  {
    HCOM = 2;   
    ROS_WARN("loadParameters(): /oreo_base/h_com cannot be retrieved.");
  }
  if (!ros::param::get("/oreo_base/l_com", LCOM))
  {
    LCOM = 3;    
    ROS_WARN("loadParameters(): /oreo_base/l_com cannot be retrieved.");
  }
  if (!ros::param::get("/oreo_base/r_com", RCOM))
  {
    RCOM = 1;
    ROS_WARN("loadParameters(): /oreo_base/r_com cannot be retrieved.");
  }
  if (!ros::param::get("/oreo_base/robot_radius", robot_radius_))
  {
    robot_radius_ = 0.225;
    ROS_WARN("loadParameters(): /oreo_base/robot_radius cannot be retrieved.");
  }
  if (!ros::param::get("/oreo_base/wheel_diameter", wheel_diameter_))
  {
    wheel_diameter_ = 0.152;    
    ROS_WARN("loadParameters(): /oreo_base/wheel_diameter cannot be retrieved.");
  }
  if (!ros::param::get("/oreo_base/geer_ratio", geer_ratio_))
  {
    geer_ratio_ = 43;    
    ROS_WARN("loadParameters(): /oreo_base/geer_ratio cannot be retrieved.");
  }
}

int init_motor(int com_num, speed_t baud_rate)
{
	if(com_num > MAX_COM || com_num < 1){
		printf("Invalid Port\n");
		exit(-1);
	}

	std::ostringstream oss;
	oss << "/dev/ttyS" << (com_num-1);
	std::string device_name = oss.str();
	
	// open device //fd ->file pointer
	int fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
	if( fd < 0 ){
		perror("init_rs232 : Unable to open - ");
		exit(-1);
	}

	// disable the blocking behavior ->I/O will return matter the
	fcntl( fd, F_SETFL, FNDELAY );

	// enable blocking
	//fcntl( fd, F_SETFL, 0 );

	// set baud rate 
	struct termios options;
	tcgetattr( fd, &options );
	cfsetispeed( &options, baud_rate );
	cfsetospeed( &options, baud_rate );

	// set no parity
	options.c_cflag &= ~PARENB;

	// set data bits
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// set stop bits (1 bit
	options.c_cflag &= ~CSTOPB;

	// set raw
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	options.c_cflag |= (CLOCAL | CREAD);

	// set option
	tcsetattr(fd, TCSANOW, &options);

	return fd;
}

void close_motor(int fd) 
{
        close(fd);
}

void write_motor(int fd, const char *s, int len, int tries) 
{
	for(int i = 0 ; i < tries ; ++i){
		if( write(fd, s, len) < len )
			continue;
		else
			break;
	}
}

int read_motor(int fd, char *buffer, int len, int tries)
{
	
  int num_bytes;
	char* bufptr = buffer;
	for(int i = 0 ; i < tries ; ++i){ 
		while( ( num_bytes = read( fd, bufptr, buffer + len - bufptr - 1 ) ) > 0 ){
                          bufptr += num_bytes;
			if( bufptr[-1] == '\n' || bufptr[-1] == '\r' ){
                                *bufptr = '\0';
				return ( bufptr - buffer );
			}
		}
		// TODO sleep
		usleep(10000); // us
	}
	*bufptr = '\0';
	return (-1);
}
//change
void setVel2(double HVel , double LVel, double RVel) //rpm
{
	//gear ratio 43 or 26
	double VH = ( HVel / 1000 ) * 60 * geer_ratio_ / (M_PI * wheel_diameter_) ;
	double VL = ( LVel / 1000 ) * 60 * geer_ratio_ / (M_PI * wheel_diameter_) ;
	double VR = ( RVel / 1000 ) * 60 * geer_ratio_ / (M_PI * wheel_diameter_) ;
	ostringstream oss_l, oss_r, oss_h;
	oss_h << "V" << (int)VH << "\r";
	oss_l << "V" << (int)VL << "\r";
	oss_r << "V" << (int)VR << "\r";
	string sVH = oss_h.str();
	string sVL = oss_l.str();
	string sVR = oss_r.str();
  
	write_motor(h_com_fd, sVH.c_str(), sVH.length(), 1);
	write_motor(l_com_fd, sVL.c_str(), sVL.length(), 1);
	write_motor(r_com_fd, sVR.c_str(), sVR.length(), 1);

}

void setVel(double mmLinearVx, double mmLinearVy, double radAngularV) // mm/s, rad/s
{	
	
  double LVel = -1 * mmLinearVx * sin( (60) * Deg2Rad) + mmLinearVy * cos( (60) * Deg2Rad) + robot_radius_ * 1000 * radAngularV;
	double RVel = -1 * mmLinearVx * sin( (60) * Deg2Rad - 120 * Deg2Rad) + mmLinearVy * cos( (60) * Deg2Rad - 120 * Deg2Rad) + robot_radius_ * 1000 * radAngularV;
	double HVel = -1 * mmLinearVx * sin( (60) * Deg2Rad + 120 * Deg2Rad) + mmLinearVy * cos( (60) * Deg2Rad + 120 * Deg2Rad) + robot_radius_ * 1000 * radAngularV;

/*
  double V = sqrt(pow(mmLinearVx,2)+pow(mmLinearVy,2));
  double RVel = V*cos((30));
  double LVel = ;
  double HVel = ;
*/
/*
  double LVel = 0;
  double RVel = mmLinearVx;
  double HVel = 0;
*/
  motor << HVel/1000 << " " << RVel/1000 << " " <<LVel/1000 <<endl;
	setVel2( 1*HVel , LVel, RVel );
}

//!!!!!!!!!!!!!!!!!!!!!!!!!!!type
void cmd_vel_Callback(const geometry_msgs::Twist& v_twist)
{

  setVel(v_twist.linear.x,v_twist.linear.y,v_twist.angular.z);
  
  ROS_INFO("I heard: [%f %f]", v_twist.linear.x, v_twist.angular.z);
}
void odometer_Callback(const geometry_msgs::Pose2D& odo_data)
{
	odo_theta = odo_data.theta;
}
void initmotor()
{
    h_com_fd = init_motor( HCOM, B9600 );
    r_com_fd = init_motor( RCOM, B9600 );
	l_com_fd = init_motor( LCOM, B9600 );
	// set acc and dec
	// home
	// disable answering
	write_motor(h_com_fd, "EN\rAC20\rDEC40\r", strlen("EN\rAC20\rDEC40\r"), 2);
	write_motor(l_com_fd, "EN\rAC20\rDEC40\r", strlen("EN\rAC20\rDEC40\r"), 2);
	write_motor(r_com_fd, "EN\rAC20\rDEC40\r", strlen("EN\rAC20\rDEC40\r"), 2);
	write_motor(h_com_fd, "HO\r", strlen("HO\r"), 2);
	write_motor(l_com_fd, "HO\r", strlen("HO\r"), 2);
	write_motor(r_com_fd, "HO\r", strlen("HO\r"), 2);
	write_motor(h_com_fd, "answ0\r", strlen("answ0\r"), 2);	
	write_motor(l_com_fd, "answ0\r", strlen("answ0\r"), 2);
	write_motor(r_com_fd, "answ0\r", strlen("answ0\r"), 2);
}

int main(int argc, char **argv)
{    
    int i = 0;
    //publisher and subscriber initializing
    motor.open("/home/robot/motor.txt",ios::out);	
    ros::init(argc, argv, "motor_controller");
    
    ros::NodeHandle n;

    //load parameters from parameter server
    loadParameters();

    //initialize motor    
    char h_buffer[32]={0};
    char l_buffer[32]={0};
    char r_buffer[32]={0};
    
    initmotor();
    ROS_INFO("Done initializing motor!");
    //angle = 
    ros::Publisher encoders_pub = n.advertise<oreo_base::EncoderPair>("encoder", 1000);
    ros::Subscriber odometer_sub = n.subscribe("odometry", 1000, odometer_Callback);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmd_vel_Callback);
  
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
      oreo_base::EncoderPair e_pair;
      write_motor(h_com_fd, "POS\r", strlen("POS\r"), 2);
      write_motor(l_com_fd, "POS\r", strlen("POS\r"), 2);
      write_motor(r_com_fd, "POS\r", strlen("POS\r"), 2);
      loop_rate.sleep();
      int h_length = read_motor(h_com_fd, h_buffer, 31, 3);
      int l_length = read_motor(l_com_fd, l_buffer, 31, 3);
      int r_length = read_motor(r_com_fd, r_buffer, 31, 3);
      
      if((l_length == -1 && r_length == -1 && h_length == -1)||(l_length > 10 && r_length > 10 && h_length > 10)){
        //warning message (-1,-1) to make odometer know something get wrong in encoder
        e_pair.horizontal = -1;
        e_pair.left = -1;    
        e_pair.right = -1;
        setVel2(0.0,0.0,0.0); 
        ROS_INFO("init motor count: ", ++i);
        initmotor();
      }else{
      	e_pair.horizontal = atoi(h_buffer);
        e_pair.left = atoi(l_buffer);
        e_pair.right = atoi(r_buffer);
      }

      //cout << l_length << " " << r_length << endl;
      //cout << e_pair.left << " " << e_pair.right << endl;
      
      ROS_INFO("publish : %" PRId64 " %" PRId64 " %" PRId64, e_pair.horizontal , e_pair.left , e_pair.right);
      encoders_pub.publish(e_pair);

      ros::spinOnce();
      loop_rate.sleep();

    }
    
    close_motor(l_com_fd);
    close_motor(r_com_fd);
    close_motor(h_com_fd);
    
    return 0;
     
}
