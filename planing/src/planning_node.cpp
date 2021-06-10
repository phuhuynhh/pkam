#include "dplanning.h"
#include "planning_client.h"

#include <ewok/ed_ring_buffer.hpp>
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

bool initialized = false;
const double resolution = 0.3;
static const int POW = 4;
static const int N = (1 << POW);

//Commander callback for
void commander_callback(const std_msgs::Int8::ConstPtr &msg)
{
	char c = msg->data;
	ROS_INFO("I heard: [%i] : %c", msg->data, c);
}

void odomCloudCallback(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::PointCloud2ConstPtr &cloud);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_planning_client");
	//For ros_client class and create service.
	ros::NodeHandle node_handle("~");
	ros::Rate *rate = new ros::Rate(20.0);
	ros::Subscriber sub = node_handle.subscribe("/planning_commander", 10, commander_callback);

	//Create Client & Controller and init().
	ros_client = new PlanningClient(node_handle);
	drone_planning = new DPlanning(ros_client);

	drone_planning->run();

	
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(node_handle, "/mavros/local_position/odom", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(node_handle, "/camera/depth/color/points", 1);
	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pcl_sub);
	sync.registerCallback(boost::bind(&odomCloudCallback, _1, _2));

	// ros::AsyncSpinner spinner(4);
	// spinner.start();

	// Main thread in 20Hz.

	// ros::waitForShutdown();

	ros::spin();


	return 0;
}

void odomCloudCallback(const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::PointCloud2ConstPtr &cloud)
{
	// double elp = ros::Time::now().toSec() - _last_time.toSec();
	// if(elp < (1 / map_rate)) return;
	ROS_INFO("LocalMap");

	tf2::Quaternion q_orig, q_rot, q_new;
	// Get the original orientation of 'commanded_pose'
	tf2::convert(odom->pose.pose.orientation, q_orig);

	// for simulation iris rotate
	q_rot.setRPY(-1.5, 0, -1.57);

	q_new = q_rot * q_orig; // Calculate the new orientation
	q_new.normalize();

	Eigen::Quaternionf q;
	q.w() = q_new.getW();
	q.x() = q_new.getX();
	q.y() = q_new.getY();
	q.z() = q_new.getZ();

	// create transform matrix
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block(0, 0, 3, 3) = Eigen::Matrix3f(q);
	transform(0, 3) = odom->pose.pose.position.x;
	transform(1, 3) = odom->pose.pose.position.y;
	transform(2, 3) = odom->pose.pose.position.z;

	// convert cloud to pcl form
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*cloud, *cloud_in);
	// transform to world frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

	// compute ewol pointcloud and origin
	Eigen::Vector3f origin = (transform * Eigen::Vector4f(0, 0, 0, 1)).head<3>();

	ewok::EuclideanDistanceRingBuffer<POW, int16_t, float, uint8_t>::PointCloud cloud_ew;

	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points = cloud_out->points;
	printf("point cloud size : %d", points.size());
	for (int i = 0; i < points.size(); ++i)
	{
		if (isnan(points.at(i).x) || isnan(points.at(i).y) || isnan(points.at(i).z))
		{
			continue;
		}
		// printf("(%f, %f, %f)",points.at(i).x, points.at(i).y, points.at(i).z);
		cloud_ew.push_back(Eigen::Vector4f(points.at(i).x, points.at(i).y, points.at(i).z, 0));
	}

	// initialize the ringbuffer map
	if (!initialized)
	{
		Eigen::Vector3i idx;
		drone_planning->rrb->getIdx(origin, idx);
		ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());
		drone_planning->rrb->setOffset(idx);
		initialized = true;
	}
	else
	{
		Eigen::Vector3i origin_idx, offset, diff;
		drone_planning->rrb->getIdx(origin, origin_idx);
		offset = drone_planning->rrb->getVolumeCenter();
		diff = origin_idx - offset;
		if (diff.array().any())
			drone_planning->rrb->moveVolume(diff.head<3>());
	}

	drone_planning->rrb->insertPointCloud(cloud_ew, origin);
	drone_planning->rrb->updateDistance();

	// visualize ringbuffer
	visualization_msgs::Marker m_occ, m_free, m_dist, m_norm;
	drone_planning->rrb->getMarkerOccupied(m_occ);
	drone_planning->rrb->getMarkerFree(m_free);
	drone_planning->rrb->getMarkerDistance(m_dist, 0.5);

	ros_client->occ_marker_pub.publish(m_occ);
	ros_client->free_marker_pub.publish(m_free);
	ros_client->dist_marker_pub.publish(m_dist);

	// _last_time = ros::Time::now();
}
//
