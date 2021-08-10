#include "planning_client.h"

using namespace message_filters;


PlanningClient::PlanningClient(ros::NodeHandle &node_handle){
	this->nh_ = &node_handle;
}


void PlanningClient::init(DPlanning *const drone_planning){

	//For Mavros Subcriber Data.
	state_sub = nh_->subscribe<mavros_msgs::State>("/mavros/state", 10, &DPlanning::state_callback, drone_planning);
	local_pos_sub = nh_->subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &DPlanning::local_position_callback, drone_planning);
	global_pos_sub = nh_->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &DPlanning::global_position_callback, drone_planning);
	
	local_octomap_sub = nh_->subscribe<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers", 10, &DPlanning::bin_octomap_callback, drone_planning);
	octomap_sub = nh_->subscribe<octomap_msgs::Octomap>("/octomap_full", 10, &DPlanning::full_octomap_callback, drone_planning);

	//for local map
	// odom_sub = nh_->subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10,&DPlanning::local_odom_callback, drone_planning);
	// pointcloud_sub = nh_->subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points",10,&DPlanning::pointcloud2_callback, drone_planning);

	occ_trigger_sub = nh_->subscribe<std_msgs::Bool>("/mapping/has_occupied",10,&DPlanning::occ_trigger_callback, drone_planning);
	apf_force_sub = nh_->subscribe<geometry_msgs::PoseStamped>("/mapping/distance_Sgrad",10,&DPlanning::apf_force_callback, drone_planning);
	local_waypoint_sub = nh_->subscribe<geometry_msgs::PoseArray>("/mapping/local_waypoints",10,&DPlanning::local_waypoint_callback, drone_planning);
	global_trigger_sub = nh_->subscribe<std_msgs::Bool>("/mapping/global_trigger",10, &DPlanning::global_trigger_callback, drone_planning);

	local_waypoint_sub1 = nh_->subscribe<geometry_msgs::PoseArray>("/mapping/local_waypoints11",2,&DPlanning::local_waypoint_callback1,drone_planning);
	local_waypoint_sub2 = nh_->subscribe<geometry_msgs::PoseArray>("/mapping/local_waypoints22",2,&DPlanning::local_waypoint_callback2,drone_planning);
	local_waypoint_sub3 = nh_->subscribe<geometry_msgs::PoseArray>("/mapping/local_waypoints33",2,&DPlanning::local_waypoint_callback3,drone_planning);
	local_waypoint_sub4 = nh_->subscribe<geometry_msgs::PoseArray>("/mapping/local_waypoints44",2,&DPlanning::local_waypoint_callback4,drone_planning);

	//For Planning process
	/**
	* Trajectory get endpoint_position and create Movement Array
	* from current_position from mavros/local_position/pose and endpoint_position
	* with velocity and acceleration and start_time.
	* calculate current_target_position with dt (current_time - start_time).
	* (x = xo + vo*t + (a* t^2)/2).
	* quaternion (TODO).
	* with this propose we can change velocity and acceleration run time and add pid controller.
	*/
	getpoint_target_sub = nh_->subscribe<geometry_msgs::PoseStamped>("/planning/endpoint_position", 10, &DPlanning::get_target_position_callback, drone_planning);
	//Set point for drone Movement.
	setpoint_pos_pub = nh_->advertise<geometry_msgs::PoseStamped>("/planning/setpoint_position", 10);
	//TODO : Need subcrible vel,acc for adding constraint.
	raw_reference_pub = nh_->advertise<mavros_msgs::PositionTarget>("/planning/setpoint_raw", 10);

	//local publisher
	traj_subset_pub = nh_->advertise<geometry_msgs::PoseArray>("/planning/traj_subset",10);
	local_target_pub = nh_->advertise<geometry_msgs::PoseStamped>("/planning/local_target",10);
	local_astar_active_pub = nh_->advertise<std_msgs::Bool>("/planning/local_astar_active",10);
	local_apf_active_pub = nh_->advertise<std_msgs::Bool>("/planning/local_apf_active",10);
	local_rrt_active_pub = nh_->advertise<std_msgs::Bool>("/planning/local_rrt_active",10);

	//visulization publsiher
	globalmarker_start_pub = nh_->advertise<visualization_msgs::Marker>("/planning/global_start_marker", 10);
	globalmarker_target_pub = nh_->advertise<visualization_msgs::Marker>("/planning/global_target_marker", 10);

	localmarker_start_pub = nh_->advertise<visualization_msgs::Marker>("/planning/local_start_marker", 10);
	localmarker_target_pub = nh_->advertise<visualization_msgs::Marker>("/planning/local_target_marker", 10);

	global_waypoints_pub1 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/global_waypoints1", 10);
	global_waypoints_pub2 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/global_waypoints2", 10);
	global_waypoints_pub3 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/global_waypoints3", 10);
	global_waypoints_pub4 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/global_waypoints4", 10);

	global_trajectory_pub1 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/global_trajectory1", 10);
	global_trajectory_pub2 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/global_trajectory2", 10);
	global_trajectory_pub3 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/global_trajectory3", 10);
	global_trajectory_pub4 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/global_trajectory4", 10);

	local_trajectory_pub1 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/local_trajectory1", 10);
	local_trajectory_pub2 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/local_trajectory2", 10);
	local_trajectory_pub3 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/local_trajectory3", 10);
	local_trajectory_pub4 = nh_->advertise<visualization_msgs::MarkerArray>("/planning/local_trajectory4", 10);

	// message_filters::Subscriber<nav_msgs::Odometry> odom_sub(*nh, "/mavros/local_position/odom", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(*nh, "/camera/depth/color/points", 1);
    // typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pcl_sub);
    // sync.registerCallback(boost::bind(&DPLanning::odomCloudCallback,drone_planning, _1, _2));


	// Visualizer Marker
	grid_marker_pub = nh_->advertise<visualization_msgs::MarkerArray>("/planning/grid", 10);
	way_points_pub = nh_->advertise<visualization_msgs::MarkerArray>("/planning/way_points", 10);
	local_way_points_pub = nh_->advertise<visualization_msgs::MarkerArray>("/planning/way_points_local", 10);
	global_traj_pub = nh_->advertise<visualization_msgs::MarkerArray>("/planning/generated_trajectory", 10);
	local_traj_pub = nh_->advertise<visualization_msgs::MarkerArray>("/planning/generated_trajectory_local", 10);
	vel_marker_pub = nh_->advertise<visualization_msgs::Marker>("/planning/velocity", 10);

}

//Not need spinOnce() in here, in planning_node already had.
void PlanningClient::publish_position_to_controller(const geometry_msgs::PoseStamped& setpoint_pos_ENU){
	setpoint_pos_pub.publish(setpoint_pos_ENU);

	printf("next position (x,y,z) : (%f, %f, %f) \n",
		setpoint_pos_ENU.pose.position.x,  setpoint_pos_ENU.pose.position.y,  setpoint_pos_ENU.pose.position.z);
}

void PlanningClient::publish_raw_position_target(const mavros_msgs::PositionTarget& raw_pos_target){
	raw_reference_pub.publish(raw_pos_target);
}
