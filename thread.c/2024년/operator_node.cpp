#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

double x = 0.0;
std::string sign = "";
double y = 0.0;

bool a_received = false;
bool b_received = false;

ros::Publisher result_pub;

void xCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x = msg->data;
    a_received = true;
}

void signCallback(const std_msgs::String::ConstPtr& msg)
{
    sign = msg->data;
}

void yCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y = msg->data;
    b_received = true;

    if (a_received && b_received) 
    {
        double result = 0.0;

        if (sign == "+") 
        {
            result = x + y;
        } 
        else if (sign == "-") 
        {
            result = x - y;
        } 
        else if (sign == "x") 
        {
            result = x * y;
        } 
        else if (sign == "/") 
        {
            if (y != 0.0) 
            {
                result = x / y;
            } 
            else 
            {
				
                return;
            }
        } 
        else 
        {
            return;
        }

        ROS_INFO_STREAM(x << " " << sign << " " << y << " = " << result);

        std_msgs::Float32 result_msg;
        result_msg.data = result;
        result_pub.publish(result_msg);

        x = 0.0;
        sign = "";
        y = 0.0;

        a_received = false;
        b_received = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calculator_node");
    ros::NodeHandle nh;

    ros::Publisher x_pub = nh.advertise<std_msgs::Float32>("/float32/x", 1);
    ros::Publisher sign_pub = nh.advertise<std_msgs::String>("/string/sign", 1);
    ros::Publisher y_pub = nh.advertise<std_msgs::Float32>("/float32/y", 1);

    result_pub = nh.advertise<std_msgs::Float32>("/result", 1);

    ros::Subscriber x_sub = nh.subscribe("/float32/x", 1, xCallback);
    ros::Subscriber sign_sub = nh.subscribe("/string/operator", 1,signCallback);
    ros::Subscriber y_sub = nh.subscribe("/float32/y", 1, yCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
