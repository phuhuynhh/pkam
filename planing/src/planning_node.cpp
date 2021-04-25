#include "dplanning.h"
#include "planning_client.h"
#include "std_msgs/Int8.h"
#include <visualization_msgs/Marker.h>


//Commander callback for 
void commander_callback(const std_msgs::Int8::ConstPtr& msg)
{
	char c = msg->data;
	ROS_INFO("I heard: [%i] : %c", msg->data, c);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_planning_client");
	//For ros_client class and create service.
	ros::NodeHandle node_handle("~");
	ros::Rate *rate = new ros::Rate(20.0);
	ros::Subscriber sub = node_handle.subscribe("/planning_commander", 10, commander_callback);

	//Create Client & Controller and init().
	PlanningClient ros_client(argc, argv);
	DPlanning drone_planning(&ros_client);

	// Main thread in 20Hz.
	while(ros::ok())
	{
		drone_planning.run();
		ros::spinOnce();
		rate->sleep();
	}

	return 0;
}
