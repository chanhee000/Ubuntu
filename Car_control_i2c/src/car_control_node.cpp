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


#define MAX_RIGHT_ANGLE -50
#define MAX_LEFT_ANGLE 50

#define MAX_SPEED  250
#define MIN_SPEED  -250

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60)
#define RPS2RPM(x) ((x)*60)

// I2C address  
#define ADDRESS 0x05

// I2C bus  
static const char *i2cDevice = "/dev/i2c-0";

double speedFactor = 255;
double steerFactor = 20;
int i2cFile;


union
{
    short data;
    char byteData[2];
} steeringAngle, motorSpeed;

unsigned char protocolData[9] ={0,};

int openI2C(void)
{
   int file;  
   
    if ((file = open(i2cDevice, O_RDWR)) < 0)  
    {  
        fprintf(stderr, "Failed %s\n", i2cDevice);  
        exit(1);  
    }  
    printf("Connected\n");  
 
    printf("Acquiring bus to 0x%x\n", ADDRESS);  
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0)  
    {  
        fprintf(stderr, "Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);  
        exit(1);  
    }
   
    return file;
}

void closeI2C(int fd)
{
   close(fd);
}


void cmdCallback(const geometry_msgs::Twist &cmdVel)
{
    double angularTemp;
    double linearTemp;
   
    linearTemp  = cmdVel.linear.x ;
    angularTemp = cmdVel.angular.z ;


	  
    steeringAngle.data  = (short)angularTemp;
       
    if(linearTemp >=  MAX_SPEED)
    {
		linearTemp = MAX_SPEED;
	}
    if(linearTemp <=  MIN_SPEED)
    {
		linearTemp = MIN_SPEED;
	}

    motorSpeed.data = (short)linearTemp;
    
    if(angularTemp <= MAX_RIGHT_ANGLE)  
    {
		angularTemp = MAX_RIGHT_ANGLE;
	}
    if(angularTemp >= MAX_LEFT_ANGLE)
    {
		angularTemp = MAX_LEFT_ANGLE;
	}
}

int main(int argc, char **argv)
{
  i2cFile = openI2C();
  
  if(i2cFile < 0)
  {
	  printf("open I2C\n");
	  return -1;
  }
  else
  {
	  printf("connected\n");
  }
 
  ros::init(argc, argv, "car_control_node");
  ros::NodeHandle n;

  std::string cmdVelTopic = "/cmd_vel";
 
  ros::Subscriber subCarControl = n.subscribe(cmdVelTopic, 20, cmdCallback);
 
  ros::Rate loopRate(20);
 
  while(ros::ok())
  {
	protocolData[0] = '#';
	protocolData[1] = 'C';
	protocolData[2] = steeringAngle.byteData[0];
	protocolData[3] = steeringAngle.byteData[1];
	protocolData[4] = motorSpeed.byteData[0];
	protocolData[5] = motorSpeed.byteData[1];
	protocolData[6] = 0;  
	protocolData[7] = 0;    
	protocolData[8] = '*';
   
	write(i2cFile, protocolData, 9);
	
	printf("Motor speed: %d\n", motorSpeed.data);
	printf("Steering angle: %d \n\n", steeringAngle.data);
	
	ros::spinOnce();
	loopRate.sleep();
  }

  return 0;
}
