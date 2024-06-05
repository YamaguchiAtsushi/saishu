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
    ros::init(argc, argv, "saishu_detect");
    ros::NodeHandle nh;
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("obstacle_robot", 10);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    ros::Rate loop_rate(10);
    

    ros::Time::waitForValid();
    ros::Time ros_begin = ros::Time::now();
    ros::WallTime wall_begin = ros::WallTime::now();

    ros::spinOnce();
    int count = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        if (scan.ranges.size() != 0)
        {
            int angle_center = scan.ranges.size() / 2;
            std::cout << "scan.ranges[" << angle_center << "] : " << scan.ranges[angle_center] << std::endl;
            if(scan.ranges[angle_center] < 0.4){
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;

                if (flag == true){
                    ros::Time start = ros::Time::now();
                    flag = false;
                }

                ros::Time now = ros::Time::now();

                ros::Duration ros_duration =  now - start;

                if (now - start < ros::Duration(3.0)){
                    continue;
                }else{
                    
                }




                // ROS_INFO("8");

            }
        }

        twist_pub.publish(twist);

        loop_rate.sleep();
    }
}