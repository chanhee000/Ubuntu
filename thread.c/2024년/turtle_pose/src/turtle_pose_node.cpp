#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <cmath>

#define RAD2DEG(x) ((x)*180.f/M_PI)
#define DEG2RAD(x) ((x)/180.f*M_PI)
geometry_msgs::Pose2D current_pos2d_msg;
float target_heading_angle_degree = 130.0;
float heading_angle_error = 0.0;


void cb_get_pose(const turtlesim::Pose& msg) 
{
  current_pos2d_msg.x = msg.x;
  current_pos2d_msg.y = msg.y;
  current_pos2d_msg.theta = msg.theta;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "sub_turtlesim_pose");
  ros::NodeHandle nh;

  nh.param("/target_heading_angle_degree", target_heading_angle_degree, target_heading_angle_degree);

  ros::Subscriber sub = nh.subscribe("/turtle1/pose", 0.1, cb_get_pose);
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 5);

  geometry_msgs::Twist cmd_vel_msg;

  ros::Rate loop_rate(30.0);

  while (ros::ok()) 
  {
    float heading1 = current_pos2d_msg.theta;
    ros::spinOnce();

    float heading2 = current_pos2d_msg.theta;
    ros::spinOnce();

    float heading = (heading1 + heading2) / 2;
    heading = 360 - heading;
    heading_angle_error = target_heading_angle_degree - RAD2DEG(current_pos2d_msg.theta);

    if (heading_angle_error > 180) 
    {
      heading_angle_error = heading_angle_error - 360;
    } 
    else if (heading_angle_error < -180) 
    {
      heading_angle_error = heading_angle_error + 360;
    }

    float speed = 0.1;
    float adjusted_angle= speed * heading_angle_error;
    
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = adjusted_angle;
    pub_cmd_vel.publish(cmd_vel_msg);

    ROS_INFO("x = %f, y = %f, theta = %f",current_pos2d_msg.x, current_pos2d_msg.y, RAD2DEG(current_pos2d_msg.theta));

    nh.getParam("/target_heading_angle_degree", target_heading_angle_degree);

    loop_rate.sleep();
  }

  return 0;
}
