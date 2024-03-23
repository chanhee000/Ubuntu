#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "math.h"
#include "move_waypoint/target_waypoint.h"
#include "move_waypoint/Target_waypoint_line.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

enum type{
   RESET = -1,
   STOP,    
   RUN,     
   ARRIVAL    
};



typedef struct
{
   double a;
   double b;
   double c;
   double d;
} Line_Equation;


typedef struct
{
   double x;
   double y;
   double theta;
} Vector_2D;

#define NEW_WAYPOINT_TOR_XY     0.01
#define NEW_WAYPOINT_TOR_THETA  5.0

bool flag_new_waypoint     = false;
int  flag_waypoint_control = STOP;


geometry_msgs::Pose2D target_pose2D;   
geometry_msgs::Pose2D target_pose2D_old;
geometry_msgs::Pose2D start_pose2D;
geometry_msgs::Pose2D cur_pose2D;
geometry_msgs::Pose2D pose_msg;
geometry_msgs::Pose2D target_waypoint_new;

move_waypoint::Target_waypoint_line target_waypoint_line_old;
move_waypoint::Target_waypoint_line target_waypoint_line_new;

void move_target_callback(const move_waypoint::Target_waypoint_line::ConstPtr& target_msg)
{
   target_pose2D = target_msg->waypoint_target_pose2d;
   start_pose2D  = target_msg->waypoint_start_pose2d;
}

void target_waypoint_line_callback(const move_waypoint::Target_waypoint_line::ConstPtr& target_msg)
{
   if(target_waypoint_line_new == target_waypoint_line_old)
   {

   }
   else
   {

   }
}


Line_Equation find_line_equation(geometry_msgs::Pose2D m_target_pose2D, geometry_msgs::Pose2D m_start_pose2D, double *angle)
{
   Line_Equation line_equ;

   m_target_pose2D.x;
   m_target_pose2D.y;
   

   m_start_pose2D.x;
   m_start_pose2D.y;
     
   line_equ.a = (m_target_pose2D.y - m_start_pose2D.y) / (m_target_pose2D.x - m_start_pose2D.x);

   line_equ.b = m_target_pose2D.y - line_equ.a * m_target_pose2D.x;
 
   line_equ.c = (m_target_pose2D.x - m_start_pose2D.x) / (m_target_pose2D.y - m_start_pose2D.y);
   
   line_equ.d = m_target_pose2D.x - line_equ.c * m_target_pose2D.y;
   
   *angle = atan2( (m_target_pose2D.y - m_start_pose2D.y) , (m_target_pose2D.x - m_start_pose2D.x) );
   return line_equ;
}


Line_Equation find_cross_waypoint_line_equation(Line_Equation wp_line_equ, geometry_msgs::Pose2D cur_pose2D)
{
    Line_Equation cross_line_equ;
    
   cross_line_equ.a = -wp_line_equ.c;
    cross_line_equ.b = (cur_pose2D.y - (cross_line_equ.a * cur_pose2D.x));
    cross_line_equ.c = -wp_line_equ.a;
    cross_line_equ.d = (cur_pose2D.x - (cross_line_equ.c * cur_pose2D.y));
    
    return cross_line_equ;
}

Line_Equation find_waypoint_finish_line_equation(Line_Equation wp_line_equ, geometry_msgs::Pose2D m_target_pose2D, double *finish_angle)
{
   Line_Equation finish_line_equ;
   
    finish_line_equ.a = -wp_line_equ.c;
    finish_line_equ.b = m_target_pose2D.y - finish_line_equ.a * m_target_pose2D.x;
    finish_line_equ.c = -wp_line_equ.a; 
    finish_line_equ.d = m_target_pose2D.x - finish_line_equ.c * m_target_pose2D.y;
    
    *finish_angle = atan(finish_line_equ.a);
    
    return finish_line_equ;
}

geometry_msgs::Pose2D target_waypoint;
std_msgs::Float64 Waypoint_speed;

bool check_pass_finish_line(geometry_msgs::Pose2D m_target_pose2D, geometry_msgs::Pose2D m_start_pose2D, geometry_msgs::Pose2D m_cur_pose2D, double ahead_finish_distance, double *xte)
{
   bool pass = false;

   Vector_2D vector_a;
   Vector_2D vector_b;
   
   double vector_angle       = 0.0;
   double inner_product, cross_product, mag_vector_a, mag_vector_b;
   double distance_to_target = 0.0;
   
   vector_a.x = (m_start_pose2D.x - m_target_pose2D.x);
   vector_a.y = (m_start_pose2D.y - m_target_pose2D.y);
   //printf("vector_a = [%.2lf, %.2lf]\n", vector_a.x, vector_a.y);
   
   vector_b.x = (m_cur_pose2D.x - m_target_pose2D.x);
   vector_b.y = (m_cur_pose2D.y - m_target_pose2D.y);
   //printf("vector_b = [%.2lf, %.2lf]\n", vector_b.x, vector_b.y);
   
   mag_vector_a = sqrt(vector_a.x * vector_a.x + vector_a.y * vector_a.y);
   mag_vector_b = sqrt(vector_b.x * vector_b.x + vector_b.y * vector_b.y);
   
   inner_product = vector_a.x * vector_b.x + vector_a.y * vector_b.y;
   cross_product = vector_a.x * vector_b.y - vector_a.y * vector_b.x;
   
   vector_angle       = acos(inner_product / (mag_vector_a * mag_vector_b));
   distance_to_target = mag_vector_b * cos(vector_angle);
   *xte               = mag_vector_b * sin(vector_angle);
   
   if(cross_product < 0)//if(cross_product >= 0) {*xte = *xte;} else{*xte *= -1;}
   {
      *xte = -1;
   }
   else
   {
      *xte *= *xte;
   }
   
   //printf("vector_angle = %.2lf  , distance_to_target = %.2lf , xte = %.2lf\n", RAD2DEG(vector_angle), distance_to_target, *xte);
   
   if(distance_to_target <= ahead_finish_distance)
   {
      pass = true;
   }
   else
   {
      pass = false;
   }
   
   return pass;
}

int flag_control;

double tx;
double ty;
double tz;
void Pose_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)//current
{
    tx = msg->pose.pose.position.x;
    ty = msg->pose.pose.position.y;
    tz = msg->pose.pose.position.z;
    cur_pose2D.x = msg->pose.pose.position.x;
    cur_pose2D.y = msg->pose.pose.position.y;
    cur_pose2D.theta = msg->pose.pose.position.z;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll  = RAD2DEG(roll);
    pitch = RAD2DEG(pitch);
    yaw   = RAD2DEG(yaw);

    //printf("Received pose in frame : X: %.2f   Y: %.2f   Z: %.2f   - R: %.2f   P: %.2f   Y: %.2f\n", tx, ty, tz, roll, pitch, yaw);
}

/*void target_waypoint_new_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
   target_waypoint_new = *msg;
}*/
void move_base_simple_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   double diff_x     = target_pose2D_old.x     - msg->pose.position.x;
   double diff_y     = target_pose2D_old.y     - msg->pose.position.y;
   double diff_theta;
   double r,p,y;
   double theta;
   
   tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    
   m.getRPY(r,p,y);
   theta = RAD2DEG(y);
   diff_theta = target_pose2D_old.theta - theta;
   
   
   target_pose2D.x     = msg->pose.position.x;
   target_pose2D.y     = msg->pose.position.y;
   target_pose2D.theta = theta;
   
   
   if((fabs(diff_x) <= NEW_WAYPOINT_TOR_XY) && (fabs(diff_y) <= NEW_WAYPOINT_TOR_XY) && (fabs(diff_theta) <= NEW_WAYPOINT_TOR_THETA))
   {
	  flag_new_waypoint   = false;
	  printf("no new waypoint! \n");

   }
   
   else
   {
      target_pose2D.x     = msg->pose.position.x;
      target_pose2D.y     = msg->pose.position.y;
      target_pose2D.theta = theta;
      
      target_pose2D_old.x       = msg->pose.position.x;
      target_pose2D_old.y       = msg->pose.position.y;
      target_pose2D_old.theta   = theta;
      
      start_pose2D           = cur_pose2D;
      flag_waypoint_control  = RUN;
      
      //printf("target waypoint  : [%.2lf %.2lf %.2lf]\n",target_pose2D.x,target_pose2D.y,target_pose2D.theta);
      flag_new_waypoint   = true; 
   }

}



void target_waypoint_callback(const geometry_msgs::Pose2D& pose_msg)
{
   if(pose_msg != target_pose2D_old)
   {
      target_pose2D     = pose_msg;
      target_pose2D_old = pose_msg;
      start_pose2D      = start_pose2D;
      //printf("target_waypoint : %.2lf  %.2lf  %.2lf\n", target_pose2D.x, target_pose2D.y, target_pose2D.theta);
      flag_new_waypoint = true;
   }
   else
   {
      double diff_x     = target_pose2D_old.x     - cur_pose2D.x;
      double diff_y     = target_pose2D_old.y     - cur_pose2D.y;
      double diff_theta = target_pose2D_old.theta - cur_pose2D.theta;
      
      if((fabs(diff_x) <= NEW_WAYPOINT_TOR_XY) && (fabs(diff_y) <= NEW_WAYPOINT_TOR_XY) && (fabs(diff_theta) <= NEW_WAYPOINT_TOR_THETA))
      {
         target_pose2D     = pose_msg;
         target_pose2D_old = pose_msg;
         start_pose2D      = start_pose2D;
         flag_new_waypoint = true;
      }
      else
      {
         flag_new_waypoint = false;
      }
   }
   target_pose2D_old = cur_pose2D;
}

void flag_control_callback(const std_msgs::Int8::ConstPtr& msg)
{
   flag_control = msg->data;
}

void waypoint_speed_callback(const std_msgs::Float64::ConstPtr& msg)
{
   Waypoint_speed = *msg;
}

int main(int argc, char **argv)
{
    Line_Equation waypoint_line;
    Line_Equation cross_waypoint_line;
    Line_Equation waypoint_finish_line;
    int count = 0;
    
    enum type type1;
    enum type type2;
    enum type type3;
    enum type type4;
    type1 = RESET;
    type2 = STOP;      
    type3 = RUN;  
    type4 = ARRIVAL;
   
    ros::init(argc, argv, "move_waypoint_node");
    ros::NodeHandle n;
   
   
    std::string poseupdate_topic            = "/poseupdate";
    std::string target_wp_topic             = "/target_wp";
    std::string flag_control_topic          = "/flag/control";
    std::string status_topic                = "/status/waypoint_move";
    std::string target_yaw_radian_topic     = "/target_yaw/radian";
    std::string target_yaw_degree_topic     = "/target_yaw/degree";
    std::string type_cmd_vel_topic          = "/type_cmd_vel";
    std::string target_speed_topic          = "/target_speed";
    std::string waypoint_line_topic         = "/waypoint_line";
    std::string control_speed_yaw_topic     = "/control_speed_yaw";
    std::string target_waypoint_new_topic   = "/target_waypoint_new";
    std::string move_base_simple_goal_topic = "/move_base_simple/goal";
    std::string move_target_topic	        = "/move_waypoint/move";
    std::string target_waypoint_line_topic  = "/wp/target_waypoint_line";
    //std::string cmd_vel_topic           = "/ackermann_steering_controller/cmd_vel";
    
    
    
    ros::param::get("~poseupdate_topic",          poseupdate_topic);
    ros::param::get("~target_wp_topic",           target_wp_topic);
    ros::param::get("~flag_control_topic",        flag_control_topic);
    ros::param::get("~status_topic",              status_topic);  
    ros::param::get("~target_yaw_radian_topic",   target_yaw_radian_topic);
    ros::param::get("~type_cmd_vel_topic",        type_cmd_vel_topic);
    ros::param::get("~target_speed_topic",        target_speed_topic);
    ros::param::get("~control_speed_yaw_topic",   control_speed_yaw_topic);
    ros::param::get("~target_yaw_degree",         target_yaw_degree_topic);
    ros::param::get("~target_waypoint_new_topic", target_waypoint_new_topic);
    ros::param::get("~move_base_simple_goal_topic", move_base_simple_goal_topic);
    ros::param::get("~move_target_topic", move_target_topic);
    //ros::param::get("~cmd_vel_topic",           cmd_vel_topic);
    
    
    
    
    ros::Subscriber sub_poseupdate                      = n.subscribe(poseupdate_topic,10, Pose_Callback);
    ros::Subscriber sub_flag_control                    = n.subscribe(flag_control_topic,10, flag_control_callback);
    ros::Subscriber sub_target_wp                       = n.subscribe(target_wp_topic, 10, target_waypoint_callback);
    ros::Subscriber sub_move_base_simple_goal           = n.subscribe(move_base_simple_goal_topic, 10, move_base_simple_goal_callback);
    ros::Subscriber sub_move_move_target                = n.subscribe(move_target_topic, 10, move_target_callback);
    ros::Subscriber sub_target_waypoint_line_topic      = n.subscribe(target_waypoint_line_topic, 10, target_waypoint_line_callback);
    //ros::Subscriber sub_target_waypoint_new = n.subscribe(target_waypoint_new_topic, 10, target_waypoint_new_callback);
    
    
    
    
   
    ros::Publisher status_pub               = n.advertise<std_msgs::Int8>(status_topic, 1);
    ros::Publisher target_yaw_radian_pub    = n.advertise<std_msgs::Float64>(target_yaw_radian_topic, 1);
    ros::Publisher type_cmd_vel_pub         = n.advertise<std_msgs::Int8>(type_cmd_vel_topic, 1);
    ros::Publisher target_speed_topic_pub   = n.advertise<std_msgs::Float64>(target_speed_topic, 1);
    ros::Publisher control_speed_topic_pub  = n.advertise<std_msgs::Float64>(control_speed_yaw_topic, 1);
    ros::Publisher target_yaw_degree_pub    = n.advertise<std_msgs::Float64>(target_yaw_degree_topic, 1);
    //ros::Publisher cmd_vel_pub            = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    
    
    
    ros::Rate loop_rate(100.0);
    
    while (ros::ok())
    {   
        std_msgs::Int8 status_waypoint_move;
        std_msgs::Int8 type_cmd_vel;
        //geometry_msgs::Twist cmd_vel;
        std_msgs::Float64 target_yaw_radian;
        std_msgs::Float64 target_speed;
        std_msgs::Float64 target_yaw_degree;
        std_msgs::Float64 waypoint_speed;
        std_msgs::Float64 control_speed_yaw;

      double angle;
      double finish_angle;
      double ahead_finish_distance = 0.0;
      double xte;
      double K_xte = 0.2;
      double target_move_speed = 0.25;
      double target_yaw_angle = 0.0;
      
      std_msgs::Float64 target_speed_msg;
        geometry_msgs::Pose2D m_target;
        geometry_msgs::Pose2D m_start;
        geometry_msgs::Pose2D m_current;
        geometry_msgs::Pose2D m_rand;

      //flag_control = type3;//RUN
      //flag_control = type2;//STOP
      //status_waypoint_move.data = STOP;
      
      /*
      if(isnan(waypoint_finish_line.a) == 1)
      {
         printf("infinity\n");
      }*/
      if(flag_control == type3)
      {
         m_target.x  = target_pose2D.x;      m_target.y  = target_pose2D.y;
         m_start.x   = start_pose2D.x;        m_start.y   = start_pose2D.y;
         m_current.x = tx;               m_current.y = ty;
         
         printf("Target_Pose : [ %.2lf  %.2lf]\n", m_target.x,  m_target.y);    
         printf("Start_ Pose : [ %.2lf  %.2lf]\n", m_start.x,  m_start.y);
         printf("Current_Pose: [ %.2lf  %.2lf]\n", m_current.x, m_current.y);
         
         waypoint_line = find_line_equation(m_target, m_start, &angle);
         cross_waypoint_line = find_cross_waypoint_line_equation(waypoint_line,m_current);
         waypoint_finish_line = find_waypoint_finish_line_equation(waypoint_line, m_target, &finish_angle);
         
         printf("status_waypoint_move = %d\n", status_waypoint_move.data);
         printf("wp_line   =  %.2lf   %.2lf   %.2lf   %.2lf   %.2lf\n", waypoint_line.a, waypoint_line.b, waypoint_line.c, waypoint_line.d, RAD2DEG(angle));
         printf("wp_f_line =  %.2lf   %.2lf   %.2lf   %.2lf   %.2lf\n\n", waypoint_finish_line.a, waypoint_finish_line.b, waypoint_finish_line.c, waypoint_finish_line.d, RAD2DEG(finish_angle));
      
         if(check_pass_finish_line(m_target, m_start, m_current, 0, &xte) == true)
         {
            status_waypoint_move.data = type4;
            status_pub.publish(status_waypoint_move);
         
            control_speed_yaw.data = 0.0; // linear.x 대신에 data에 값을 할당
            control_speed_topic_pub.publish(control_speed_yaw);
            
            target_yaw_angle  = angle;
            target_yaw_radian.data = target_yaw_angle;
            target_yaw_radian_pub.publish(target_yaw_radian);
            
            target_yaw_degree.data = RAD2DEG(target_yaw_angle);
            target_yaw_degree_pub.publish(target_yaw_degree);

            printf("target_speed = %.2lf\n", target_speed_msg.data);
         }
         else//도달x
         {
            //printf("flag_control : %d\n", flag_control);
         
            status_waypoint_move.data = type3;
            status_pub.publish(status_waypoint_move);
      
            control_speed_yaw.data = target_move_speed;
            control_speed_topic_pub.publish(control_speed_yaw);
            
            target_yaw_angle  = angle + xte * K_xte;
            target_yaw_radian.data = target_yaw_angle;
            target_yaw_radian_pub.publish(target_yaw_radian);
         
            target_yaw_degree.data = RAD2DEG(target_yaw_angle);
            target_yaw_degree_pub.publish(target_yaw_degree);
         
            //printf("target_speed = %.2lf\n", target_speed.data); // data를 출력
            //printf("target_angle = %.2lf   target_angle_degree = %.2lf(degree)\n", target_yaw_angle, RAD2DEG(target_yaw_angle));
            
         }
      }
      else if(flag_control == type2)
      {
         status_waypoint_move.data = type2;
         status_pub.publish(status_waypoint_move);
         
         control_speed_yaw.data = 0.0;
            control_speed_topic_pub.publish(control_speed_yaw);
            
         printf("-----------------------\n\n");
         printf("          STOP\n");
         printf("-----------------------\n\n");
      }
      else if(flag_control == type1)
      { 
         status_waypoint_move.data = type1;
         status_pub.publish(status_waypoint_move);
         
         control_speed_yaw.data = 0.0;
            control_speed_topic_pub.publish(control_speed_yaw);
            
         printf("-----------------------\n\n");
         printf("          RESET\n");
         printf("-----------------------\n\n");
      }
      else
      {
      }
      
      
      
      //printf("target_waypoint_new : %.2lf   %.2lf   %.2lf\n", target_waypoint_new.x, target_waypoint_new.y, target_waypoint_new.theta);
      
      
       type_cmd_vel.data = 1;
   
        
      status_pub.publish(status_waypoint_move);
      type_cmd_vel_pub.publish(type_cmd_vel);
      target_speed_topic_pub.publish(target_speed_msg);
      //cmd_vel_pub.publish(cmd_vel);
      //control_speed_topic_pub.publish(control_speed_yaw);
      //target_speed_topic_pub.publish(target_speed);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
