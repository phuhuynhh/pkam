#include "dplanning.h"
#include "planning_client.h"

#include <ros/ros.h>
#include <math.h> /* isnan, sqrt */

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "std_msgs/Int8.h"
#include <visualization_msgs/Marker.h>

class DPlanning;
class PlanningClient;

PlanningClient *ros_client;
DPlanning *drone_planning;

//Commander callback for
void commander_callback(const std_msgs::Int8::ConstPtr &msg)
{
	char c = msg->data;
	ROS_INFO("I heard: [%i] : %c", msg->data, c);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_planning_client");
	//For ros_client class and create service.
	ros::NodeHandle node_handle("~");
	ros::Subscriber sub = node_handle.subscribe("/planning_commander", 10, commander_callback);

	//Create Client & Controller and init().
	ros_client = new PlanningClient(node_handle);
	drone_planning = new DPlanning(ros_client);

	drone_planning->run();
	// ros::AsyncSpinner spinner(4);
	// spinner.start();

	// Main thread in 20Hz.

	// ros::waitForShutdown();

	ros::spin();


	return 0;
}
//
