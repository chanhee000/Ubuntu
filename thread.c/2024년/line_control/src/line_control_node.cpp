#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define TSL1401CL_SIZE 320
#define THRESHOLD 0.01  
#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];

float Line_Center = 147;
float OFFSET = 0;


void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (tsl1401cl_data[i] > threshold)
        {
            ThresholdData[i] = 180;
        }
        else
        {
            ThresholdData[i] = 0;
        }
    }
}

void tsl1401cl_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
}

int find_line_center()
{
    int centroid = 0;
    int sum = 0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        sum += LineSensor_threshold_Data[i];
        centroid += LineSensor_threshold_Data[i] * i;
    }

    centroid = (sum != 0) ? centroid / sum : 0;

    return centroid;
}

void lane_control(geometry_msgs::Twist &cmd_vel)
{
	float Kp = 0.3;
	float Ki = 0.0;
	float Kd = 0.01;
	
	float error = 0.0;
	float error_d = 0.0;
	float error_old = 0.0;
	float Steering_Angle = 0.0;
	
    if (find_line_center() == 0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        error = Line_Center - find_line_center() + OFFSET;
        error_d = error - error_old;
        Steering_Angle = Kp * error + Kd * error_d + Ki * 0;

        cmd_vel.linear.x = 0.7;
        cmd_vel.angular.z = Steering_Angle / 130;

        error_old = error;
    }
}

int main(int argc, char **argv)
{
    int count = 0;

    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist msg_cmd;

    ros::init(argc, argv, "line_control");
    ros::NodeHandle nh;

    ros::Subscriber tsl1401cl_sub = nh.subscribe("/tsl1401cl", 10, tsl1401cl_Callback);
    ros::Publisher tst1401cl_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);
    while (ros::ok())
    {

        printf("Threshold Data: \n");

        msg_cmd.linear.x = 0.7;

        for (int i = 0; i < TSL1401CL_SIZE; i++)
        {
            printf("%d ", LineSensor_threshold_Data[i]);
        }
        printf("\n");

        double centroid = find_line_center();
        printf("Line Centroid: %f\n", centroid);

        lane_control(cmd_vel);

        tst1401cl_cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
