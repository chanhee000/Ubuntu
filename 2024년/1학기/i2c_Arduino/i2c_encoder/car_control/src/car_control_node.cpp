#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// I2C address  
#define ADDRESS 0x05


#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60)
#define RPS2RPM(x) ((x)*60)


#define MAX_RIGHT_ANGLE -45
#define MAX_LEFT_ANGLE 45

#define MAX_SPEED  255
#define MIN_SPEED  -255


double speedFactor = 255;
double steerFactor = -255;

unsigned char protocolData[9] =  {'#','C',0,0,0,0,0,0,'*'};;
unsigned char encoderPos_long[6] = {0};

union
{
    short data;
    char byteData[2];
} steeringAngle, motorSpeed;

union
{
    short encoder_data;
    char encoder_byte[2];
} encoder_union;


int i2cFile;

int openI2C(void)
{
    int file;  
    const char *deviceName = "/dev/i2c-0";
    
    if ((file = open(deviceName, O_RDWR)) < 0)  
    {  
        fprintf(stderr, "Failed %s\n", deviceName);  
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
	int count   = 0;
	
    i2cFile = openI2C();
    

    ros::init(argc, argv, "car_control_node");
    ros::NodeHandle n;
    
    std::string cmdVelTopic = "/cmd_vel";
    
    ros::Subscriber subCarControl = n.subscribe(cmdVelTopic, 20, cmdCallback);
    
    ros::Rate loopRate(20);
    
    i2cFile = openI2C();
    if(i2cFile < 0)
    {
         printf("I2C를 열 수 없습니다.");
        return -1;
    }
    else
    {
        printf("I2C가 연결되었습니다.");
	}
    
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
        
       
        read(i2cFile, encoderPos_long, 6);
        
        if((encoderPos_long[0] == 'h' )&& (encoderPos_long[5] == 'k'))
        {
            encoder_union.encoder_byte[0] = encoderPos_long[1];
            encoder_union.encoder_byte[1] = encoderPos_long[2];
        }
        
        printf("encoder : %d\n",encoder_union.encoder_data);
        
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
