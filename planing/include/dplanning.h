
#ifndef D_PLANNING_H
#define D_PLANNING_H

#include "planning_client.h"

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <visualization_msgs/Marker.h>

class PlanningClient;

class DPlanning {
public:	

	enum class PLAING_TYPE {
		SIMPLE = 0,
		A_START = 1,
		RRT = 2,
		NEWTON_EULER = 3,
		//MORE
	};
	static PLAING_TYPE planning_type;


	DPlanning(PlanningClient *ros_client);
	DPlanning(PlanningClient *ros_client,ros::Rate *rate);

	static constexpr bool  KEEP_ALIVE = true;
	static constexpr float ROS_RATE = 20.0;

	// The setpoint publishing rate MUST be faster than 2Hz
	ros::Rate *rate_;
	tf2_ros::Buffer tfBuffer_;



	//d_ ~ mean Drone message.
	mavros_msgs::State d_current_state;
	geometry_msgs::PoseStamped d_local_position;
	sensor_msgs::NavSatFix d_global_position;

	// Saving init pose to home.
	// geometry_msgs::PoseStamped target_position;


	void publishVisualize();
	void removeVisualize();
	// Subsriber Callback.
	void state_callback(const mavros_msgs::State::ConstPtr &msg);
	void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
	void global_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
	void get_target_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);



	void public_local_position();
	void run();

private:
	PlanningClient *ros_client;


	bool approaching = false;
	bool endpoint_active = false;

	visualization_msgs::Marker points, line_strip;


	double vx = 0.5;
	double vy = 0.5;
	double vz = 0.5;

	// Use this param for ros_client->setpoint_pos_local_pub()
	geometry_msgs::PoseStamped setpoint_pos_ENU; 
	geometry_msgs::PoseStamped startpoint_pos_ENU; 
	geometry_msgs::PoseStamped endpoint_pos_ENU; //

	// Target_position
	ros::Time start_time;


		
	double currentYaw();
	double getYaw(const geometry_msgs::Quaternion &msg);
	double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);
};
	

#endif