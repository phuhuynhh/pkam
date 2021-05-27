#include <ewok/ed_ring_buffer.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

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
ewok::EuclideanDistanceRingBuffer<POW> rrb(resolution, 1.0);

ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, norm_marker_pub;
ros::Publisher cloud2_pub, center_pub;

double map_rate, pub_rate;

void odomCloudCallback(const geometry_msgs::PoseStamped::ConstPtr &odom, const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    double elp = ros::Time::now().toSec() - _last_time.toSec();
    ROS_INFO("%f", elp);
    // if(elp < (1 / map_rate)) return;


    /*

    ROS_INFO("Updating ringbuffer map");
    // get orientation and translation
    Eigen::Quaternionf q;
    q.w() = odom->pose.orientation.w;
    q.x() = odom->pose.orientation.x;
    q.y() = odom->pose.orientation.y;
    q.z() = odom->pose.orientation.z;

    std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << std::endl << q.vec() << std::endl;

    // create transform matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block(0, 0, 3, 3) = Eigen::Matrix3f(q);
    transform(0, 3) = odom->pose.position.x;
    transform(1, 3) = odom->pose.position.y;
    transform(2, 3) = odom->pose.position.z;
    // std::cout << transform.matrix() << "\n\n";

    // convert cloud to pcl form
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *cloud_in);

    /*

    // transform to world frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform.inverse());

    // down-sample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_out);
    float res = 0.1f;
    sor.setLeafSize(res, res, res);
    sor.filter(*cloud_filtered);
    */

    /*
    // compute ewol pointcloud and origin
    Eigen::Vector3f origin = (transform * Eigen::Vector4f(0, 0, 0, 1)).head<3>();
    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud_ew;
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > points = cloud_in->points;
    for(int i = 0; i < points.size(); ++i)
    {
        cloud_ew.push_back(Eigen::Vector4f(points.at(i).x, points.at(i).y, points.at(i).z, 0));
    }

    // initialize the ringbuffer map
    if(!initialized)
    {
        Eigen::Vector3i idx;
        rrb.getIdx(origin, idx);
        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());
        rrb.setOffset(idx);
        initialized = true;
    }
    else
    {
        Eigen::Vector3i origin_idx, offset, diff;
        rrb.getIdx(origin, origin_idx);
        offset = rrb.getVolumeCenter();
        diff = origin_idx - offset;
        if(diff.array().any()) rrb.moveVolume(diff.head<3>());
        ROS_INFO_STREAM("Origin: " << origin.transpose() << " Moving " << diff.transpose());
    }

    */

    /*
    // insert point cloud to ringbuffer
    rrb.insertPointCloud(cloud_ew, origin);
    rrb.updateDistance();

    // visualize ringbuffer
    visualization_msgs::Marker m_occ, m_free, m_dist, m_norm;
    rrb.getMarkerOccupied(m_occ);
    // rrb.getMarkerFree(m_free);
    // rrb.getMarkerDistance(m_dist, 0.5);
    // rrb.getMarkerNormal(m_norm);

    occ_marker_pub.publish(m_occ);
    // free_marker_pub.publish(m_free);
    // dist_marker_pub.publish(m_dist);
    // norm_marker_pub.publish(m_norm);

    _last_time = ros::Time::now();
    ROS_INFO("End.");
    */


    
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



    typedef sync_policies::ExactTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;
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