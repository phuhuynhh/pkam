#include "planning_client.h"


PlanningClient::PlanningClient(int &argc, char **argv)
{
	this->nh_ = new ros::NodeHandle();

	avoidCollision_ = false;
}


void PlanningClient::init(DPlanning *const drone_planing){

	//For Mavros Subcriber Data.
	state_sub = nh_->subscribe<mavros_msgs::State>("/mavros/state", 10, &DPlanning::state_callback, drone_planing);
	local_pos_sub = nh_->subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &DPlanning::local_position_callback, drone_planing);
	global_pos_sub = nh_->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &DPlanning::global_position_callback, drone_planing);
	

	local_octomap_sub = nh_->subscribe<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers", 10, &DPlanning::octomap_callback, drone_planing);
	octomap_sub = nh_->subscribe<octomap_msgs::Octomap>("/octomap_full", 10, &DPlanning::full_octomap_callback, drone_planing);

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
	getpoint_target_sub = nh_->subscribe<geometry_msgs::PoseStamped>("/planning/endpoint_position", 10, &DPlanning::get_target_position_callback, drone_planing);
	//Set point for drone Movement.
	setpoint_pos_pub = nh_->advertise<geometry_msgs::PoseStamped>("/planning/setpoint_position", 10);
	//TODO : Need subcrible vel,acc for adding constraint.
	raw_reference_pub = nh_->advertise<mavros_msgs::PositionTarget>("/planning/setpoint_raw", 10);

	// Visualizer Marker
	grid_marker_pub = nh_->advertise<visualization_msgs::MarkerArray>("/planning/grid", 10);
	way_points_pub = nh_->advertise<visualization_msgs::MarkerArray>("/planning/way_points", 10);
	global_traj_pub = nh_->advertise<visualization_msgs::MarkerArray>("/planning/generated_trajectory", 10);
	vel_marker_pub = nh_->advertise<visualization_msgs::Marker>("/planning/velocity", 10);

}\

//Not need spinOnce() in here, in planning_node already had.
void PlanningClient::publish_position_to_controller(const geometry_msgs::PoseStamped& setpoint_pos_ENU){
	setpoint_pos_pub.publish(setpoint_pos_ENU);

	printf("next position (x,y,z) : (%f, %f, %f) \n",
		setpoint_pos_ENU.pose.position.x,  setpoint_pos_ENU.pose.position.y,  setpoint_pos_ENU.pose.position.z);
}

void PlanningClient::publish_raw_position_target(const mavros_msgs::PositionTarget& raw_pos_target){
	raw_reference_pub.publish(raw_pos_target);
}
