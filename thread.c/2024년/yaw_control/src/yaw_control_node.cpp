#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double roll, pitch, yaw;
double target_heading_yaw = 40;



double yaw_degree(double yaw_deg)
{
    if (yaw_deg >= 360)
    {
        yaw_deg -= 360;
    }
    else if (yaw_deg < 0)
    {
        yaw_deg += 360;
    }
    return yaw_deg;
}


geometry_msgs::Twist yaw_control(double Kp,double Kd,double Ki)
{
	geometry_msgs::Twist cmd_vel;
	
    double yaw_error_old = 0.0;
    
    double yaw_deg = RAD2DEG(yaw);
    yaw_deg = yaw_degree(yaw_deg);
    
    double yaw_error = target_heading_yaw - yaw_deg;
    double yaw_error_d = yaw_error - yaw_error_old;
    double error_s = 0.0;
    double Steering_Angle = Kp * yaw_error + Kd * yaw_error_d + Ki *error_s;
    
    cmd_vel.linear.x = 0.8;
    cmd_vel.linear.z = Steering_Angle;
    
  if (fabs(yaw_error) < 1.0)
	{
		cmd_vel.linear.x = 0.0; 
		cmd_vel.linear.z = 0.0;
	}
    else
    {
        cmd_vel.linear.x = 1.0;
        cmd_vel.linear.z = Steering_Angle; 
    }
    
    yaw_error_old = yaw_error;
}

void imu1Callback(const sensor_msgs::Imu::ConstPtr& msg) 
{

    tf2::Quaternion q
    (msg->orientation.x,
     msg->orientation.y,
     msg->orientation.z,
     msg->orientation.w);
     
    tf2::Matrix3x3 m(q);      

    m.getRPY(roll, pitch, yaw);
    
    double yaw_deg = yaw_degree(RAD2DEG(yaw));
    
    printf("degree:%.2f \n", yaw_deg);
     

}

int main(int argc, char **argv)
{
    int count = 0;
   
    geometry_msgs::Twist cmd_vel;
   
    ros::init(argc, argv, "yaw_control");
    ros::NodeHandle n;
    ros::Subscriber yaw_control_sub = n.subscribe("/imu", 1000, imu1Callback);
    ros::Publisher yaw_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);


    ros::Rate loop_rate(30.0);

	double Kp = 0.5;
	double Ki = 0.0;
	double Kd = 0.4;

    while (ros::ok())
    {
         geometry_msgs::Twist cmd_vel = yaw_control(Kp,Kd,Ki);
        yaw_cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
