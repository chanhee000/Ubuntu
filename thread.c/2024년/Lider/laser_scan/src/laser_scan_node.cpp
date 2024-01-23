#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_2d_msgs/Point2D.h"


#define ROBOT_WIDTH 		0.2
#define ROBOT_WIDTH_TOR	 	0.1


#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)



int laser_point_no = 358;

nav_2d_msgs::Point2D obstacle_2d[358];



bool front_obstacle_detection(double detection_range, int count)//x방향 -projection
{
   int sum = 0;
   for(int i = 0; i < count; i++)
    {
      if(obstacle_2d[i].x <= detection_range)
      {
         if((obstacle_2d[i].y >= -(ROBOT_WIDTH + ROBOT_WIDTH_TOR))&&(obstacle_2d[i].y <= (ROBOT_WIDTH + ROBOT_WIDTH_TOR)))
         {
            sum++;
         }
      }
   }
   if(sum >= 10)
   {
      return true;
   }
   else
   {
      return false;
   }
}



void laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("LaserScan (val,angle)=(%f,%f", scan->range_min,scan->angle_min);
    int count = (int)(360. / RAD2DEG(scan->angle_increment));
    for(int i = 0; i < count; i++)
    {
      float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
      //printf("%.3d  %.2f  %.2lf \n",i, degree, scan->ranges[i]);
      
      obstacle_2d[i].x = scan->ranges[i]*cos(scan->angle_min + scan->angle_increment * i);
	  obstacle_2d[i].y = scan->ranges[i]*sin(scan->angle_min + scan->angle_increment * i);
	  
	  printf("%.3d (%.2f  %.2lf) = (%.2lf  %.2lf) \n",i, degree, scan->ranges[i],obstacle_2d[i].x,obstacle_2d[i].y);
   }
    printf("obstacle_detection_flag: %d \n",front_obstacle_detection(1.0,count));  
}


int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "laser_scan");
    ros::NodeHandle n;

    std::string laser_scan_topic = "/scan";
    
    ros::param::get("~laser_scan_topic", laser_scan_topic);
    
    ros::Subscriber sub_laser_scan = n.subscribe(laser_scan_topic, 1, laser_scan_Callback);
    
    ros::Rate loop_rate(5.0);// 5.0Hz
    
    while (ros::ok())
    {
      
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
