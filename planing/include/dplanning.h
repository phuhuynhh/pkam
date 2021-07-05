
#ifndef D_PLANNING_H
#define D_PLANNING_H

#include "planning_client.h"
#include "grid/Grid3D.h"


#include "path_planning/APF.h"
#include "path_planning/Astar.h"
#include "path_planning/planning_setup.h"


#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>
#include <cmath>
#include <string>
#include <chrono>


#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>



#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>



#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>





class PlanningClient;
class Grid3D;
class APF;
class Astar;
class EuclideanDistanceRingBuffer;
class RaycastRingBuffer;
class RingBufferBase;
	
using namespace message_filters;


class DPlanning {
public:

	// all the steering and path planning work with this grid
	// this data structure take octomap as input and create ready to use grid
	Grid3D *grid;
	APF *apf;
	Astar* astar;
	ros::NodeHandle* nh_;
	int planning = 1;

	enum class PLANNING_STEP {
		TAKE_OFF,
		GLOBAL_PLANNING,
		FOLLOW_TRAJECTORY,
		LOCAL_PLANNING,
		IDLE,
		ASTAR_PLANNING,
		//MORE
	};


	DPlanning(PlanningClient *ros_client);
	DPlanning(PlanningClient *ros_client,ros::Rate *rate);

	// typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
	// typedef Synchronizer<MySyncPolicy> Sync;
	// boost::shared_ptr<Sync> sync;




	static constexpr bool  KEEP_ALIVE = true;
	static constexpr float DEFAULT_VELOCITY = 0.3f;
	static constexpr float ROS_RATE = 20.0;

	// The setpoint publishing rate MUST be faster than 2Hz
	ros::Rate *rate_;
	tf2_ros::Buffer tfBuffer_;



	//d_ ~ mean Drone message.
	mavros_msgs::State d_current_state;
	geometry_msgs::PoseStamped d_local_position;
	geometry_msgs::PoseStamped d_previous_position;

	nav_msgs::Odometry d_local_odometry;

	sensor_msgs::NavSatFix d_global_position;
	octomap_msgs::Octomap::ConstPtr octomap_msgs;


	visualization_msgs::MarkerArray global_trajectory;
	visualization_msgs::MarkerArray d_way_points;

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
  	void bin_octomap_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
	void full_octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg);



	//local map callback
	void occ_trigger_callback(const std_msgs::Bool::ConstPtr &msg);
	void apf_force_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
	

	void public_local_position();
	void run();
	void update_grid_map();

private:
	std::string m_worldFrameId = "/map";
	PLANNING_STEP planning_type = PLANNING_STEP::TAKE_OFF;
	PlanningClient *ros_client;

	bool approaching = false;
	bool endpoint_active = false;
	bool octomap_activate = false;

	visualization_msgs::Marker points, velocity_vector, global_trajectory_line;
	double vx = 0.5;
	double vy = 0.5;
	double vz = 0.5;
	double travel_cost = 0.0;

	// Use this param for ros_client->setpoint_pos_local_pub()
	geometry_msgs::PoseStamped setpoint_pos_ENU;
	mavros_msgs::PositionTarget setpoint_raw;


	geometry_msgs::PoseStamped startpoint_pos_ENU;
	geometry_msgs::PoseStamped endpoint_pos_ENU; 
	sensor_msgs::PointCloud2 octomap_cloud;


	// Space bounds
  	double _min_bounds[3];
  	double _max_bounds[3];
	  // goal state
  	double _prev_goal[7];

	// Target_position
	ros::Time start_time;
	ros::Time pre_time;

	long long worst_duration = 0;
	long long best_duration = 100000000000000;

	ompl::base::StateSpacePtr space;
	ompl::base::SpaceInformationPtr si;
	ompl::base::ProblemDefinitionPtr pdef;
	mav_trajectory_generation::Trajectory trajectory;
	mav_msgs::EigenTrajectoryPoint::Vector states;

	double currentYaw();
	double getYaw(const geometry_msgs::Quaternion &msg);
	double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);
	bool isStateValid(const ompl::base::State *state);
};


#endif
