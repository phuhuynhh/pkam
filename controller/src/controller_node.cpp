#include "dcontroller.h"
#include "control_client.h"
#include "std_msgs/Int8.h"




void commander_callback(const std_msgs::Int8::ConstPtr& msg)
{
	char c = msg->data;
	ROS_INFO("I heard: [%i] : %c", msg->data, c);
}

int DController::global_num = 0;
DController::MISSION_STATE DController::mission_state  = DController::MISSION_STATE::WAIT;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_controller_client");
	//For ros_client class and create service.
	ros::NodeHandle node_handle("~");
	ros::Rate *rate = new ros::Rate(20.0);
	ros::Subscriber sub = node_handle.subscribe("/control_commander", 10, commander_callback);

	//Create Client & Controller and init().
	ControlClient ros_client(argc, argv);


	DController drone_control(&ros_client, rate);

	drone_control.init();
	// Main thread in 20Hz.
	while(ros::ok())
	{

		//keep offboardMode has activated and motors have armed.
		drone_control.offboardMode();
		switch (DController::mission_state){
			case (DController::MISSION_STATE::WAIT):
				break;
			case (DController::MISSION_STATE::IDLE):
				drone_control.takeOff();
				break;
			case (DController::MISSION_STATE::TAKE_OFF):
				if (drone_control.is_mission_finished()){
					ROS_INFO("TakeOff Finished.");
					drone_control.mission_state  = DController::MISSION_STATE::COMMAND;

				}
				break;
			case (DController::MISSION_STATE::COMMAND):
					drone_control.flyToLocal(-5,-5,2.5); // (5,5,3) was a test case for A*
				if (drone_control.is_mission_finished()){
					drone_control.mission_state  = DController::MISSION_STATE::LAND;
				}
				break;
			case (DController::MISSION_STATE::RETURN_HOME):
				break;
			case (DController::MISSION_STATE::LAND):
				drone_control.land();
				break;
		}

		//Public setpoint_pos_ENU to MAVROS.
		drone_control.public_local_position();
		ros::spinOnce();
		rate->sleep();
	}

	return 0;
}
