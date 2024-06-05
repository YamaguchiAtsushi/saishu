#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <map>
#include <math.h>

// オドメトリから得られる現在の位置と姿勢
double robot_x, robot_y;
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;

geometry_msgs::Twist twist; // 指令する速度、角速度

std::map<std::string, double> params_; // パラメータをここに格納
geometry_msgs::PoseStamped goal; // 目標地点

// オドメトリのコールバック
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_x = msg->pose.pose.position.x;
	robot_y = msg->pose.pose.position.y;
	robot_r = msg->pose.pose.orientation;
}

//　goalで指定した位置に近いかの判定を行う
int near_position(geometry_msgs::PoseStamped goal)
{
	double difx = robot_x - goal.pose.position.x;
	double dify = robot_y - goal.pose.position.y;
	return (sqrt(difx * difx + dify * dify) < 0.4);
}



int main(int argc, char** argv) {
    // ROS ノードの初期化
    ros::init(argc, argv, "simple_goal_publisher");

    // ROS ノードハンドルの作成
    ros::NodeHandle nh;

	// Subscriber, Publisherの定義
	  ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 1000, odom_callback);
	  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    ros::Publisher goal_publisher = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    ros::Rate loop_rate(100);

    // odometryの値の初期化
    robot_x = 0.0;
    robot_y = 0.0;
    robot_r.x = 0.0;
    robot_r.y = 0.0;
    robot_r.z = 0.0;
    robot_r.w = 1.0;

    //goal_msg.header.frame_id = "map"; // 地図座標系で指定
    goal.pose.position.x = 2.0;  // X 座標
    goal.pose.position.y = 2.0;  // Y 座標
    //goal.pose.orientation.w = 1.0; // 向きはクォータニオン形式で指定 (ここでは0度)


    while (ros::ok())
    {
      ros::spinOnce();

      // メッセージをパブリッシュ
      goal_publisher.publish(goal);

      if(near_position(goal)){
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
      }


      loop_rate.sleep();
    }
    return 0;
}