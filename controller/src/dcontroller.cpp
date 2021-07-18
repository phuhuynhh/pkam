#include "dcontroller.h"


DController::DController(ControlClient *ros_client,ros::Rate *rate)
{
	this->ros_client = ros_client;
	this->ros_client->init(this);

  	// The setpoint publishing rate MUST be faster than 2Hz
	this->rate_ = new ros::Rate(ROS_RATE);

	static tf2_ros::TransformListener tfListener(tfBuffer_);
}


void DController::init(){
	ROS_INFO("Drone Controller initializing,..");
  	// Wait for FCU connection
	while(ros::ok() && !d_current_state.connected)
	{
		ros::spinOnce();
		rate_->sleep();
		ROS_INFO("connecting to FCU...");
	}

  	// Wait for ROS
	for(int i = 0; ros::ok() && i < 4 * ROS_RATE; ++i)
	{
		ros::spinOnce();
		rate_->sleep();
	}

	while(received_home_pose==false)
	{
		ros::spinOnce();
		rate_->sleep();
		std::cout << "Waiting for Local Position" << std::endl;
	}

	setpoint_pos_ENU = gps_init_pos = d_local_position;

 	// Send a few setpoints before starting, otherwise px4 will not switch to OFFBOARD mode
	for(int i = 30; ros::ok() && i > 0; --i)
	{
		ros_client->setpoint_pos_local_pub.publish(setpoint_pos_ENU);
		ros::spinOnce();
		rate_->sleep();
	}


	last_request = ros::Time::now();
	std::cout << "last_request is ARMED"<< std::endl;
}

void DController::run(){

}

bool DController::is_mission_finished(){
	if (distance(d_local_position, endpoint_pos_ENU) < 0.2f){
		setpoint_pos_ENU = endpoint_pos_ENU;
		return true;
	}
	return false;

}

void DController::flyToLocal(double x, double y, double z){
	endpoint_pos_ENU.pose.position.x = x;
	endpoint_pos_ENU.pose.position.y = y;
	endpoint_pos_ENU.pose.position.z = z;
	endpoint_pos_ENU.pose.orientation = tf::createQuaternionMsgFromYaw(currentYaw());


	// ros_client->publishTrajectoryEndpoint(endpoint_pos_ENU);
	ros_client->endpoint_pos_pub.publish(endpoint_pos_ENU);
}



void DController::offboardMode()
{
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	if( d_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
	{
		if( ros_client->set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		{
			ROS_INFO("Offboard enabled");
		}
		last_request = ros::Time::now();
	}
	else
	{
		if( !d_current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
		{
			ROS_INFO("Armd request");
			if( ros_client->arming_client.call(arm_cmd) && arm_cmd.response.success)
			{
				ROS_INFO("Vehicle armed");
				mission_state  = DController::MISSION_STATE::IDLE;
			}
			last_request = ros::Time::now();
		}
	}
	return;
}

void DController::takeOff()
{
  // ROS_INFO("Taking off. Current position: E: %f, N: %f, U: %f", d_local_position.pose.position.x,d_local_position.pose.position.y, d_local_position.pose.position.z);

  // Take off
	endpoint_pos_ENU = d_local_position;
	endpoint_pos_ENU.pose.position.z = TAKEOFF_ALTITUDE;
	ros_client->endpoint_pos_pub.publish(endpoint_pos_ENU);
	mission_state = MISSION_STATE::TAKE_OFF;

	return;
}

void DController::land()
{
	int i;
	mavros_msgs::CommandTOL land_cmd;
	land_cmd.request.yaw = 0;
	land_cmd.request.latitude = NAN; //Land at current location
	land_cmd.request.longitude = NAN;
	land_cmd.request.altitude = 0;

	ROS_INFO("Trying to land");
	while(!(ros_client->land_client.call(land_cmd) && land_cmd.response.success))
	{
		switch (control_type)
		{
		case (CONTROL_TYPE::POSITION):
			public_local_position();
			break;
		default:
			public_raw_target();
			break;
		}
		ros::spinOnce();
		ROS_WARN("Retrying to land");
		rate_->sleep();
	}

	// Wait until proper landing (or a maximum of 15 seconds)
	for(i = 0; ros::ok() && d_landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && i < MAX_ATTEMPTS; ++i)
	{
		ros::spinOnce();
		rate_->sleep();
	}
	if(i == MAX_ATTEMPTS)
		ROS_WARN("Landing failed, aborting");
	else
		ROS_INFO("Landing success");


	return;
}


void DController::public_local_position(){
	ROS_INFO("Current position: E: %f, N: %f, U: %f", d_local_position.pose.position.x,
		d_local_position.pose.position.y, d_local_position.pose.position.z);
	ROS_INFO("Set position: E: %f, N: %f, U: %f", setpoint_pos_ENU.pose.position.x,
		setpoint_pos_ENU.pose.position.y, setpoint_pos_ENU.pose.position.z);


	ros_client->setpoint_pos_local_pub.publish(setpoint_pos_ENU);
}

void DController::public_raw_target(){
	ros_client->setpoint_raw_pub.publish(setpoint_raw_target);
}



//  Callback for subcriber.

void DController::state_callback(const mavros_msgs::State::ConstPtr &msg)
{
	d_current_state = *msg;
}

void DController::extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
	d_landed_state = msg->landed_state;
}

void DController::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	d_local_position = *msg;

	if (!received_home_pose)
	{
		received_home_pose = true;
		d_home_pose = *msg;
		ROS_INFO_STREAM("Home pose initialized to: " << d_home_pose);
	}

  //For debug logger.
	static int cnt = 0;
	cnt++;
	if(cnt % 100 == 0)
	{
    // ROS_INFO("Mavros local position: E: %f, N: %f, U: %f, yaw: %f", d_local_position.pose.position.x,
    //          d_local_position.pose.position.y, d_local_position.pose.position.z, currentYaw());
	}
}

void DController::global_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	d_global_position = *msg;
	static int cnt = 0;

	cnt++;
	if(cnt % 100 == 0)
	{
    // ROS_INFO("GPS: lat: %f, long: %f, alt: %f", msg->latitude, msg->longitude, msg->altitude);
	}
}

void DController::getpoint_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	// if(approaching && !endpoint_active) return;
	control_type = CONTROL_TYPE::POSITION;
	setpoint_pos_ENU = *msg;
}

void DController::getpoint_raw_callback(const mavros_msgs::PositionTarget::ConstPtr &msg){
	control_type = CONTROL_TYPE::RAW;
	setpoint_raw_target = *msg;
}


void DController::offboardStatusLooper(const ros::TimerEvent &event){
	if( d_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
	{
		ROS_INFO("%s",d_current_state.mode.c_str());
		if( ros_client->set_mode_client.call(offboard_setmode) && offboard_setmode.response.mode_sent)
		{
			ROS_INFO("Offboard enabled");

		}
		last_request = ros::Time::now();
	}
	else
	{
		if( !d_current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
		{
			if( ros_client->arming_client.call(arm_cmd) && arm_cmd.response.success)
			{
				ROS_INFO("Vehicle armed");
			}
			last_request = ros::Time::now();
		}
	}
};

//
//Util Method.
//
double DController::currentYaw()
{
  //Calculate yaw current orientation
	double roll, pitch, yaw;
	tf::Quaternion q;

	tf::quaternionMsgToTF(d_local_position.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	return yaw;
}

double DController::getYaw(const geometry_msgs::Quaternion &msg)
{
  //Calculate yaw current orientation
	double roll, pitch, yaw;
	tf::Quaternion q;

	tf::quaternionMsgToTF(msg, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	return yaw;
}

double DController::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
{
	tf::Point t1, t2;
	tf::pointMsgToTF(p1.pose.position, t1);
	tf::pointMsgToTF(p2.pose.position, t2);

	return t1.distance(t2);
}
