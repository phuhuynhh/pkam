#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


geometry_msgs::PoseStamped v_pose;
nav_msgs::Odometry v_odom;

bool initialized = false;


void vision_pose_callback(const nav_msgs::OdometryConstPtr& msg)
{
	initialized = true;
    v_odom = *msg;
    // ROS_INFO("Got data : %f, %f, %f", v_odom.pose.pose.position.x, v_odom.pose.pose.position.y, v_odom.pose.pose.position.z);
	
	v_pose.pose.position.x = v_odom.pose.pose.position.x;
	v_pose.pose.position.y = v_odom.pose.pose.position.y;
	v_pose.pose.position.z = v_odom.pose.pose.position.z;


	v_pose.pose.orientation.x = v_odom.pose.pose.orientation.x;
    v_pose.pose.orientation.y = v_odom.pose.pose.orientation.y;
    v_pose.pose.orientation.z = v_odom.pose.pose.orientation.z;
    v_pose.pose.orientation.w = v_odom.pose.pose.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_sub");
    ros::NodeHandle nh;

    ros::Publisher mavros_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10);
    ros::Subscriber vision_pose_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/camera_pose",10, vision_pose_callback);
    ros::Rate rate(10);
    while(ros::ok())
    {
        //Public setpoint_pos_ENU to MAVROS.
		if (initialized){
			v_pose.header.stamp = ros::Time::now();
			ROS_INFO("send pose : %f, %f, %f", v_pose.pose.position.x, v_pose.pose.position.y, v_pose.pose.position.z);
			mavros_pub.publish(v_pose);
		}
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
