#include "control_client.h"

ControlClient::ControlClient(int &argc, char **argv)
{
	this->nh_ = new ros::NodeHandle();


	avoidCollision_ = true;
}


void ControlClient::init(DController *const drone_control)
{
  	//For Mavros Subcriber Data.
	state_sub = nh_->subscribe<mavros_msgs::State>("/mavros/state", 10, &DController::state_callback, drone_control);
	extended_state_sub = nh_->subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, &DController::extended_state_callback, drone_control);
	local_pos_sub = nh_->subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &DController::local_position_callback, drone_control);
	global_pos_sub = nh_->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &DController::global_position_callback, drone_control);

  	//For position control from Planning package.
	getpoint_pos_sub = nh_->subscribe<geometry_msgs::PoseStamped>("/planning/setpoint_position", 10, &DController::getpoint_position_callback, drone_control);
	setpoint_pos_local_pub = nh_->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	

	//For PositionTarget Raw control form planning (flowing trajectory)
	getpoint_raw_sub = nh_->subscribe<mavros_msgs::PositionTarget>("/planning/setpoint_raw", 10, &DController::getpoint_raw_callback, drone_control);
	setpoint_raw_pub = nh_->advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
	




	/**
	* Trajectory get endpoint_position and create Movement Array
	* from current_position from mavros/local_position/pose and endpoint_position
	* with velocity and acceleration and start_time.
	* calculate current_target_position with dt (current_time - start_time).
	* (x = xo + vo*t + (a* t^2)/2).
	* quaternion (TODO).
	* with this propose we can change velocity and acceleration run time and add pid controller.
	*/
	//Goal target for planning.
	endpoint_pos_pub = nh_->advertise<geometry_msgs::PoseStamped>("/planning/endpoint_position", 10);


  	//For mavros Service.
	arming_client = nh_->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	land_client = nh_->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	set_mode_client = nh_->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");


  	//For Commander and Status

  	// offboard_status_timer = nh_->createTimer(ros::Duration(1), &DController::offboardStatusLooper,this->drone_control);

  	// The tf module of mavros does not work currently
  	// /mavros/setpoint_position/tf/ could also be used in approachMarker function
  	//nh_->setParam("/mavros/local_position/tf/frame_id", "map");
  	//nh_->setParam("/mavros/local_position/tf/child_frame_id", "drone");
  	//nh_->setParam("/mavros/local_position/tf/send", true);
}

void ControlClient::create_status_timer(){

}

void ControlClient::publishTrajectoryEndpoint(const geometry_msgs::PoseStamped &setpoint_pos_ENU)
{
	if(avoidCollision_)
	{
		endpoint_pos_pub.publish(setpoint_pos_ENU);
		ros::spinOnce();
	}
	else
	{
		ROS_INFO("Collision avoidance has not been enabled");
	}
}

void ControlClient::setParam(const std::string &key, double d)
{
	nh_->setParam(key, d);
}
