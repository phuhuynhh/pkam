
#ifndef D_CONTROLLER_H
#define D_CONTROLLER_H

#include "control_client.h"

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
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/SetBool.h>

class ControlClient;

class DController {
public:	

	enum class MISSION_STATE {
		WAIT = 0,
		IDLE = 1,
		TAKE_OFF = 2,
		COMMAND = 3,
		RETURN_HOME = 4,
		LAND = 5,
	};
	static MISSION_STATE mission_state;

	static  int global_num;

	DController(ControlClient *ros_client);
	DController(ControlClient *ros_client,ros::Rate *rate);

	static constexpr bool  KEEP_ALIVE = true;
	static constexpr float TAKEOFF_ALTITUDE = 2.5;
	static constexpr int   MAX_ATTEMPTS = 300;
	static constexpr float SAFETY_ALTITUDE_GPS = 3.0;
	static constexpr float ROS_RATE = 20.0;

	// The setpoint publishing rate MUST be faster than 2Hz
	ros::Rate *rate_;
	tf2_ros::Buffer tfBuffer_;

	//d_ ~ mean Drone message.


	mavros_msgs::State d_current_state;
	geometry_msgs::PoseStamped d_local_position;
	sensor_msgs::NavSatFix d_global_position;
	geometry_msgs::TransformStamped d_transformStamped;

	//Saving init pose to home.
	geometry_msgs::PoseStamped d_home_pose;
	bool received_home_pose;

	geometry_msgs::PoseArray target_position;



	//Subsriber Callback.
	void state_callback(const mavros_msgs::State::ConstPtr &msg);
	void extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg);
	void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
	void global_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
	void getpoint_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

	//Timer Callback, test fail => TODO.
	void offboardStatusLooper(const ros::TimerEvent &event);

	void init();
	void run();
	bool is_mission_finished();
	void offboardMode();
	void takeOff();
	void land();
	void testFlightHorizontal();
	void testFlightVertical();


	// void flyToGlobal(double latitude, double longitude, double altitude, double yaw);
	void flyToLocal(double x, double y, double z);
	// void flyToLocalNoCollision(double x, double y, double z);


	void public_local_position();

private:
	ControlClient *ros_client;


	bool approaching = true;
	bool endpoint_active = false;

	uint8_t d_landed_state = 0;
	uint8_t close_enough = 0;

	// Use this param for ros_client->setpoint_pos_local_pub()
	geometry_msgs::PoseStamped setpoint_pos_ENU; 

	// Use this param for Trajectory Generetor via : ros_client->endpoint_pos_pub() 
	// pass to path-planning node.
	geometry_msgs::PoseStamped endpoint_pos_ENU; //
	geometry_msgs::PoseStamped gps_init_pos; // is needed ?? 

	ros::Time last_request;

	mavros_msgs::SetMode offboard_setmode;
	mavros_msgs::CommandBool arm_cmd;

		
	double currentYaw();
	double getYaw(const geometry_msgs::Quaternion &msg);
	double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);
};
	

#endif