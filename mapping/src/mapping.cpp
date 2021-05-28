#include "mapping.h"

#include "ewok/ed_ring_buffer.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <cmath>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

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

#include <string>
#include <chrono>

using namespace message_filters;

Mapping::Mapping(){
	this->nh_ = new ros::NodeHandle("~");
	this->rrb = new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0);

	this->init();
}

void Mapping::init(){
// ringbuffer visualizer
	occ_marker_pub = nh_->advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5, true);
	free_marker_pub = nh_->advertise<visualization_msgs::Marker>("ring_buffer/free", 5, true);
	dist_marker_pub = nh_->advertise<visualization_msgs::Marker>("ring_buffer/distance", 5, true);
	norm_marker_pub = nh_->advertise<visualization_msgs::Marker>("ring_buffer/normal", 5, true);

    // synchronized subscriber for pointcloud and odometry
	message_filters::Subscriber<geometry_msgs::PoseStamped> odom_sub(*nh_, "/mavros/local_position/pose", 1);

	message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(*nh_, "/camera/depth/color/points", 1);
	typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pcl_sub);
	sync.registerCallback(boost::bind(&Mapping::originCloudCallback, _1, _2));
}

void Mapping::originCloudCallback(Mapping *this,const geometry_msgs::PoseStamped::ConstPtr &pose, const sensor_msgs::PointCloud2::ConstPtr &cloud){
	double elp = ros::Time::now().toSec() - _last_time.toSec();
	d_local_position = * pose;
	// if(elp < (1 / map_rate)) return;

	ROS_INFO("Updating ringbuffer map");
    // get orientation and translation
	Eigen::Quaternionf q;
	q.w() = d_local_position.pose.orientation.w;
	q.x() = d_local_position.pose.orientation.x;
	q.y() = d_local_position.pose.orientation.y;
	q.z() = d_local_position.pose.orientation.z;

    // create transform matrix
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block(0, 0, 3, 3) = Eigen::Matrix3f(q);
	transform(0, 3) = d_local_position.pose.position.x;
	transform(1, 3) = d_local_position.pose.position.y;
	transform(2, 3) = d_local_position.pose.position.z;
    // std::cout << transform.matrix() << "\n\n";

    // convert cloud to pcl form
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*cloud, *cloud_in);

    // down-sample
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	// pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	// sor.setInputCloud(cloud_out);
	// float res = 0.1f;
	// sor.setLeafSize(res, res, res);
	// sor.filter(*cloud_filtered);

    // compute ewol pointcloud and origin
	Eigen::Vector3f origin = (transform * Eigen::Vector4f(0, 0, 0, 1)).head<3>();
	ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud_ew;
	// std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > points = cloud_filtered->points;
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > points = cloud_in->points;
	for(int i = 0; i < points.size(); ++i)
	{
		cloud_ew.push_back(Eigen::Vector4f(points.at(i).x, points.at(i).y, points.at(i).z, 0));
	}

    // initialize the ringbuffer map
	if(!initialized)
	{
		Eigen::Vector3i idx;
		rrb->getIdx(origin, idx);
		ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());
		rrb->setOffset(idx);
		initialized = true;
	}
	else
	{
		Eigen::Vector3i origin_idx, offset, diff;
		rrb->getIdx(origin, origin_idx);
		offset = rrb->getVolumeCenter();
		diff = origin_idx - offset;
		if(diff.array().any()) rrb->moveVolume(diff.head<3>());
	}

    // insert point cloud to ringbuffer
	rrb->insertPointCloud(cloud_ew, origin);
	rrb->updateDistance();

    // visualize ringbuffer
	visualization_msgs::Marker m_occ, m_free, m_dist, m_norm;
	rrb->getMarkerOccupied(m_occ);
	rrb->getMarkerFree(m_free);
	rrb->getMarkerDistance(m_dist, 0.5);
	// rrb->getMarkerNormal(m_norm);

	occ_marker_pub.publish(m_occ);
	free_marker_pub.publish(m_free);
	dist_marker_pub.publish(m_dist);
	norm_marker_pub.publish(m_norm);

	_last_time = ros::Time::now();
}