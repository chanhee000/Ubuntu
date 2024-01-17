#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

double roll, pitch, yaw;



void imu1Callback(const sensor_msgs::Imu::ConstPtr& msg) 
{

    tf2::Quaternion q
    (msg->orientation.x,
     msg->orientation.y,
     msg->orientation.z,
     msg->orientation.w);
     
    tf2::Matrix3x3 m(q);      

    m.getRPY(roll, pitch, yaw);
     
    double yaw_deg = yaw * (180.0 / M_PI);
    
    if (yaw_deg > 360)
    {
      yaw_deg = yaw_deg - 360;
    }
    else if (yaw_deg < 0)
    {
      yaw_deg = yaw_deg + 360;
    }
    
    printf("degree:%.2f \n", yaw_deg);
}

int main(int argc, char **argv)
{
  int count = 0;
   
  ros::init(argc, argv, "yaw_control");
  ros::NodeHandle n;
  ros::Subscriber yaw_control_sub = n.subscribe("/imu", 1000, imu1Callback);
  ros::Rate loop_rate(30.0);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
