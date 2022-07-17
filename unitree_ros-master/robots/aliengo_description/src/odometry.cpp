#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

geometry_msgs::Pose pose_AlienGo;
bool start;
void odom_call_back(const nav_msgs::Odometry::ConstPtr& msg){
    start = 1;
    pose_AlienGo = msg->pose.pose;
}

int main(int argc,char** argv){
    ros::init(argc,argv,"odometry_publisher");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("odom", 1000, odom_call_back);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate r(20);
    ros::Time current_time = ros::Time::now();
    while(ros::ok())
    {
        if(start){
            current_time = ros::Time::now();
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "world";
            odom_trans.child_frame_id = "base";

            odom_trans.transform.translation.x = pose_AlienGo.position.x;
            odom_trans.transform.translation.y = pose_AlienGo.position.y;
            odom_trans.transform.translation.z = pose_AlienGo.position.z;
            odom_trans.transform.rotation = pose_AlienGo.orientation;
            odom_broadcaster.sendTransform(odom_trans);
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;

}