#include <sys/ioctl.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <unistd.h>         
#include <fcntl.h>                
#include <time.h>
#include <math.h>


#define MAX_R_ANGLE -50
#define MAX_L_ANGLE 50

#define MAX_ROBOT_SPEED  250
#define MIN_ROBOT_SPEED  -250

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60)
#define RPS2RPM(x) ((x)*60)

//i2c <address>  
#define ADDRESS 0x05

//i2c <bus>  
static const char *deviceName = "/dev/i2c-0";

double speed_factor = 255;
double steer_factor = 20;
int file_I2C;


union
{
    short data;
    char byte_data[2];
} steering_angle, motor_speed;

unsigned char protocal_data[9] ={0,};

int open_I2C(void)
{
   int file;  
   
    if ((file = open( deviceName, O_RDWR ) ) < 0)  
    {  
        fprintf(stderr, "I2C: Failed %s\n", deviceName);  
        exit(1);  
    }  
    printf("I2C: Connected\n");  
 
    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);  
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0)  
    {  
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);  
        exit(1);  
    }
   
    return file;
}

void close_I2C(int fd)
{
   close(fd);
}


void cmd_callback(const geometry_msgs::Twist & cmd_vel)
{
    double angular_temp;
    double linear_temp;
   
    linear_temp  = cmd_vel.linear.x ;
    angular_temp = cmd_vel.angular.z ;


	  
    steering_angle.data  = (short)angular_temp;
       
    if(linear_temp >=  MAX_ROBOT_SPEED)
    {
		linear_temp = MAX_ROBOT_SPEED;
	}
    if(linear_temp <=  MIN_ROBOT_SPEED)
    {
		linear_temp = MIN_ROBOT_SPEED;
	}

    motor_speed.data = (short)linear_temp;
    
    if(angular_temp <= MAX_R_ANGLE)  
    {
		angular_temp = MAX_R_ANGLE;
	}
    if(angular_temp >= MAX_L_ANGLE)
    {
		angular_temp = MAX_L_ANGLE;
	}
}

int main(int argc, char **argv)
{
  file_I2C = open_I2C();
  
  if(file_I2C < 0)
  {
	  printf("Unable to open I2C\n");
	  return -1;
  }
  else
  {
	  printf("I2C is Connected\n");
  }
 
  ros::init(argc, argv, "car_control_node");
  ros::NodeHandle n;

  std::string cmd_vel_topic = "/cmd_vel";
 
  ros::Subscriber sub_car_control = n.subscribe(cmd_vel_topic, 20, cmd_callback);
 
  ros::Rate loop_rate(20);
 
  while(ros::ok())
  {
	protocal_data[0] = '#';
	protocal_data[1] = 'C';
	protocal_data[2] = steering_angle.byte_data[0];
	protocal_data[3] = steering_angle.byte_data[1];
	protocal_data[4] = motor_speed.byte_data[0];
	protocal_data[5] = motor_speed.byte_data[1];
	protocal_data[6] = 0;  
	protocal_data[7] = 0;    
	protocal_data[8] = '*';
   
	write(file_I2C, protocal_data, 9);
	
	printf("motor_speed: %d\n", motor_speed.data);
	printf("steering_angle: %d \n\n", steering_angle.data);
	
	ros::spinOnce();
	loop_rate.sleep();
  }

  return 0;
}
