#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include "std_msgs/Empty.h"
#include <time.h>

sensor_msgs::LaserScan scan;
geometry_msgs::Twist twist; // 指令する速度、角速度

ros::Time start;

bool flag = true;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
    float nandeyanen = scan_->range_max;
    scan.header = scan_->header;
    scan.angle_min = scan_->angle_min;
    scan.angle_max = scan_->angle_max;
    scan.angle_increment = scan_->angle_increment;
    scan.time_increment = scan_->time_increment;
    scan.scan_time = scan_->scan_time;
    scan.range_min = scan_->range_min;
    scan.range_max = scan_->range_max;
    scan.ranges = scan_->ranges;
    scan.intensities = scan_->intensities;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_avoid");
    ros::NodeHandle nh;
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_pub", 10);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    ros::Rate loop_rate(10);
    

    ros::spinOnce();
    int count = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        //ROS_INFO("1");


        // int angle_center = scan.ranges.size() / 2;
        // std::cout << "scan.ranges[" << angle_center << "] : " << scan.ranges[angle_center] << std::endl;
        // if(scan.ranges[angle_center] < 0.4){
        //     twist.linear.x = 0.0;
        //     twist.angular.z = 0.0;
        //     }

    

        if (scan.ranges.size() != 0)
        {
            //ROS_INFO("2");
            int angle_center = scan.ranges.size() / 2;
            //std::cout << "scan.ranges[" << angle_center << "] : " << scan.ranges[angle_center] << std::endl;
            if(scan.ranges[angle_center] < 0.8){
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                ROS_INFO("3");
                }

            }
        }


        twist_pub.publish(twist);

        loop_rate.sleep();
}
