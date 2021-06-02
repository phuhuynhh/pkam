#include <ewok/ed_nor_ring_buffer.h>
#include <ewok/raycast_ring_buffer.h>
#include <ros/ros.h>

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
using namespace message_filters;

// global declaration
ros::Time _last_time;

bool initialized = false;
const double resolution = 0.1;
static const int POW = 6;
static const int N = (1 << POW);
ewok::EuclideanDistanceNormalRingBuffer<POW> rrb(0.1, 1.0);

ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, norm_marker_pub;
ros::Publisher cloud2_pub, center_pub;

double map_rate, pub_rate;

void odomCloudCallback(const geometry_msgs::PoseStampedConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    double elp = ros::Time::now().toSec() - _last_time.toSec();
    ROS_INFO("%f", elp);    
}

void timerCallback(const ros::TimerEvent& e)
{
    if(!initialized) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3d center;
    // rrb.getBufferAsCloud(cloud, center);

    // convert to ROS message and publish
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(cloud, cloud2);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = center(0);
    pose.pose.position.y = center(1);
    pose.pose.position.z = center(2);

    // message publish should have the same time stamp
    ros::Time stamp = ros::Time::now();
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    cloud2.header.stamp = stamp;
    cloud2.header.frame_id = "map";

    cloud2_pub.publish(cloud2);
    center_pub.publish(pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realtime_example");
    ros::NodeHandle nh;

    // ringbuffer visualizer
    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5, true);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5, true);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5, true);
    // norm_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/normal", 5, true);
    cloud2_pub = nh.advertise<sensor_msgs::PointCloud2>("ring_buffer/cloud2", 1, true);
    center_pub = nh.advertise<geometry_msgs::PoseStamped>("ring_buffer/center", 1, true);

    // synchronized subscriber for pointcloud and odometry
    message_filters::Subscriber<geometry_msgs::PoseStamped> odom_sub(nh, "/mavros/local_position/pose", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/camera/depth/color/points", 1);
    typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pcl_sub);
    sync.registerCallback(boost::bind(&odomCloudCallback, _1, _2));

    // get parameter
    // nh.getParam("/ring_buffer/map_rate", map_rate);
    // nh.getParam("/ring_buffer/publish_rate", pub_rate);

    // std::cout << map_rate << "," << pub_rate << std::endl;

    // timer for publish ringbuffer as pointcloud
    ros::Timer timer = nh.createTimer(ros::Duration(1 / 5), timerCallback);

    ros::Duration(0.5).sleep();
    _last_time = ros::Time::now();
    std::cout << "Start mapping!" << std::endl;

    ros::spin();

    return 0;
}