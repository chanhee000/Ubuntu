#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"


bool AEB_control = false;

double sona_dis_front = 0.0;
double speed = 0.0;

geometry_msgs::Twist msg_cmd;
ros::Subscriber sona_range_frontsub;

void Sona_FrontCallback(const sensor_msgs::Range::ConstPtr& msg)
{
   sona_dis_front = msg->range;
}

int main(int argc, char **argv)
{
  int count = 0;
  //read_speed_data();
  
  ros::init(argc, argv, "AEB_system");
  ros::NodeHandle n;
  sona_range_frontsub       = n.subscribe("/range_front", 1000, Sona_FrontCallback);
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);
  ros::Rate loop_rate(30);
   
   

  while (ros::ok())
  {
   printf("range_front: %.3f\n", sona_dis_front);
   if (sona_dis_front > 1.4)
   {
      AEB_control = true;
   }
   else       
   {
      AEB_control = false;
   }

   if (AEB_control == true)
   {
      speed = 0.3;
      msg_cmd.linear.x = speed;
   }
   else
   {
      speed = 0.0;
      msg_cmd.linear.x = speed;
   }
   
   
    pub_cmd_vel.publish(msg_cmd);
  
    ros::spinOnce();
    ++count;
    
    loop_rate.sleep();
  }
  return 0;
}
