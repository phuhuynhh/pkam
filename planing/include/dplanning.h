
#ifndef D_PLANNING_H
#define D_PLANNING_H

#include "planning_client.h"
#include "grid/Grid3D.h"
#include "path_planning/APF.h"

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
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <visualization_msgs/Marker.h>

class PlanningClient;
class Grid3D;
class APF;

class DPlanning {
public:

	// all the steering and path planning work with this grid
	// this data structure take octomap as input and create ready to use grid
	Grid3D grid = Grid3D(400, 400, 400, 0.4);
	APF apf = APF(&grid);

	enum class PLANNING_TYPE {
		TAKE_OFF,
		A_START,
		RRT,
		NEWTON_EULER,
		POTENTIAL_FIELD,
		//MORE
	};


	DPlanning(PlanningClient *ros_client);
	DPlanning(PlanningClient *ros_client,ros::Rate *rate);

	static constexpr bool  KEEP_ALIVE = true;
	static constexpr float DEFAULT_VELOCITY = 0.3f;
	static constexpr float ROS_RATE = 30.0;

	// The setpoint publishing rate MUST be faster than 2Hz
	ros::Rate *rate_;
	tf2_ros::Buffer tfBuffer_;



	//d_ ~ mean Drone message.
	mavros_msgs::State d_current_state;
	geometry_msgs::PoseStamped d_local_position;
	sensor_msgs::NavSatFix d_global_position;

	// Saving init pose to home.
	// geometry_msgs::PoseStamped target_position;



	void draw_velocitty();
	void draw_global_trajectory();
	void remove_global_trajectory();
	// Subsriber Callback.
	void state_callback(const mavros_msgs::State::ConstPtr &msg);
	void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
	void global_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
	void get_target_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void octomap_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);


	void public_local_position();
	void run();
	void update_grid_map();

private:
	std::string m_worldFrameId = "/map";
	PLANNING_TYPE planning_type = PLANNING_TYPE::TAKE_OFF;
	PlanningClient *ros_client;

	bool approaching = false;
	bool endpoint_active = false;
	bool octomap_activate = false;

	visualization_msgs::Marker points, velocity_vector, global_trajectory_line;


	double vx = 0.5;
	double vy = 0.5;
	double vz = 0.5;

	// Use this param for ros_client->setpoint_pos_local_pub()
	geometry_msgs::PoseStamped setpoint_pos_ENU;
	geometry_msgs::PoseStamped startpoint_pos_ENU;
	geometry_msgs::PoseStamped endpoint_pos_ENU; //
	sensor_msgs::PointCloud2 octomap_cloud;

	// Target_position
	ros::Time start_time;

	ros::Time pre_time;

	double currentYaw();
	double getYaw(const geometry_msgs::Quaternion &msg);
	double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);
};


#endif
