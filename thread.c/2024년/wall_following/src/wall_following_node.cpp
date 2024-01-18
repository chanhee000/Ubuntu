#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front = 0.0;
double left = 0.0;
double right = 0.0;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  front = msg->range;
  printf("Front: %.2f\n", front);
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  right = msg->range;
  printf("Right: %.2f\n\n", right);
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  left = msg->range;
  printf("Left:  %.2f\n", left);
}

void PID_wall_following(geometry_msgs::Twist& pid, geometry_msgs::Twist& cmd_vel)
{
    double Kp = 0.5;
    double Ki = 0.0;
    double Kd = 0.4;

    double error = left - right;
    double error_old = 0.0;
    double error_d = error - error_old;
    double error_s = 0.0;
    error_s += error;

    double steering_control = Kp * error + Ki * error_s + Kd * error_d;

    if (front < 1.2)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = steering_control;
    }

	error_old = error;
    pid = cmd_vel;
}

int main(int argc, char **argv)
{
  int count = 0;
  
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Twist pid_cmd_vel;
  
  ros::init(argc, argv, "wall_following");
  ros::NodeHandle n;
  
  ros::Subscriber front_sonar_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);
  ros::Subscriber right_sonar_sub = n.subscribe("/range_front_right", 1000, Right_Sonar_Callback);
  ros::Subscriber left_sonar_sub = n.subscribe("/range_front_left", 1000, Left_Sonar_Callback);
  
  ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

  ros::Rate loop_rate(30.0);  
  
  while (ros::ok())
    {

        PID_wall_following(pid_cmd_vel, cmd_vel);
        sonar_cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

  return 0;
}
