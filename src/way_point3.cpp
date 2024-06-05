#include <ros/ros.h>
 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>  // add goal navi
 
// Global
int target_pos = 0;
int robot_state = 0; // 0: idle 1:moving 2:goal
 
class Goal {
public:
    Goal(double px, double py, double pz, double ow);
    ~Goal();
 
private:
    ros::Publisher pub;
    ros::NodeHandle nh;
};
 
Goal::Goal(double px, double py, double pz, double ow){
    pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
 
    ros::Rate one_sec(1);
    one_sec.sleep();
     
    ros::Time time = ros::Time::now();
    geometry_msgs::PoseStamped goal_point;
 
    goal_point.pose.position.x = px;
    goal_point.pose.position.y = py;
//    goal_point.pose.position.z =  pz;
    goal_point.pose.orientation.z =  pz;
    goal_point.pose.orientation.w = ow;
    goal_point.header.stamp = time;
    goal_point.header.frame_id = "map";
 
    pub.publish(goal_point);
 
}
 
Goal::~Goal(){
 
}
 
 
 
void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
 
    int status_id = 0;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9
 
    if (!status->status_list.empty()){
    actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
    status_id = goalStatus.status;
    }
 
    ROS_INFO("status=%d\n",status_id);
    if(status_id==1){
    //移動中
        ROS_INFO("Target = %d MOVING!!\n",target_pos);
        robot_state = 1;
    }
 
    if((status_id==3)||(status_id==0)){
    //ゴールに到達・もしくはゴールに到達して待機中。
    
        if(status_id==3){
            if(robot_state!=1){
                return;
            }
            robot_state = 2;
            ROS_INFO("Target = %d Goal reached!!!!!\n",target_pos);
            if(target_pos==1) { // target1到着時
                target_pos++;
                Goal goal_ob(1.5, -0.5, -0.3, 1.0);    // target2へ
            }
            else if(target_pos==2) {    // target2到着時
                target_pos++;
                Goal goal_ob(1.9, -1.2, -0.7, 0.7);    // add goal navi
            }
            else if(target_pos==3) {
                target_pos++;
                Goal goal_ob(2.0, -1.79, 0, 1.0);    // add goal navi
            }
            /*
            else if(target_pos==4) {
                target_pos++;
                Goal goal_ob(3.0, -1.8, 0, 1.0);    // add goal navi
            }
            else if(target_pos==5) {
                target_pos++;
                Goal goal_ob(4.0, -1.69, 0, 1.0);    // add goal navi
            }
            */
            else if(target_pos==4){
                target_pos++;
                Goal goal_ob(5.0, -1.69, 0, 1.0);    // add goal navi
            }
        
            else if(target_pos==5){
                target_pos++;
                Goal goal_ob(7.5, -1.69, 1.5, 1.0);    // add goal navi
            }
            else if(target_pos==6){
                ROS_INFO("Final Goal reached!!!!!\n");
            }
            
        }
        else {
            ROS_INFO("Idle!\n");
            robot_state = 0;
        }
 
    }
 
}
 
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "way_point3");
 
    ros::NodeHandle nh;
    ros::Subscriber switch_sub;
 
    ros::Subscriber move_base_status_sub;
    move_base_status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &navStatusCallBack);
    target_pos = 1;
    Goal goal_ob(1.13, -0.04, 0, 1.0);    // add goal navi
//    goal_ob(1.13, -0.04, 0, 1.0);    // add goal navi
 
    ros::spin();
 
    return 0;
}