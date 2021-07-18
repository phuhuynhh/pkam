#include <ewok/ed_ring_buffer.hpp>
#include <ros/ros.h>
#include <math.h>       /* isnan, sqrt */

#include <std_msgs/Bool.h>


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
using namespace message_filters;

// global declaration
ros::Time _last_time;

//LocalMap
bool initialized = false;
const double resolution = 0.4;
static const int POW = 4;
static const int N = (1 << POW);
ewok::EuclideanDistanceRingBuffer<POW> rrb(resolution, 1.0);


//Publisher
ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, norm_marker_pub;
ros::Publisher occ_trigger_pub, apf_force_pub; // can has more.

double map_rate, pub_rate;
std::string m_worldFrameId = "/map";

void odomCloudCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    double elp = ros::Time::now().toSec() - _last_time.toSec();
    // if(elp < (1 / map_rate)) return;
    
    tf2::Quaternion q_orig, q_rot, q_new;
    // Get the original orientation of 'commanded_pose'
    tf2::convert(odom->pose.pose.orientation , q_orig);

    // for simulation iris rotate
    q_rot.setRPY(-1.5, 0, -1.57);

    q_new = q_rot*q_orig;  // Calculate the new orientation
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
    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud_ew;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points = cloud_out->points;
    // printf("point cloud size : %d", points.size());

    for(int i = 0; i < points.size(); ++i)
    {
        if (isnan(points.at(i).x) || isnan(points.at(i).y) || isnan(points.at(i).z)){
            continue;
        }
        // printf("(%f, %f, %f)",points.at(i).x, points.at(i).y, points.at(i).z);  
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
    }
    // insert point cloud to ringbuffer
    // ROS_INFO("Insert Point Cloud");
    rrb.insertPointCloud(cloud_ew, origin);
    // ROS_INFO("Update Distance");
    rrb.updateDistance();
    // visualize ringbuffer
    visualization_msgs::Marker m_occ, m_free, m_dist, m_norm;
    rrb.getMarkerOccupied(m_occ);
    rrb.getMarkerFree(m_free);
    rrb.getMarkerDistance(m_dist, 0.5);

    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);
    dist_marker_pub.publish(m_dist);

    _last_time = ros::Time::now();
}

void timerCallback(const ros::TimerEvent& e)
{
    if(!initialized) return;
    ROS_INFO("Timer Trigger");
    //TODO : Khang VO
    //Calculate when drone met occupied -> send occ_trigger_pub = true.
    //Calculate APF and send this force to apf_force_pub via PoseStamped.

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_mapping");
    ros::NodeHandle nh;

    //Local Mapping Controller 
    occ_trigger_pub = nh.advertise<std_msgs::Bool>("/mapping/has_occupied", 1, true);
    apf_force_pub = nh.advertise<geometry_msgs::PoseStamped>("/mapping/potential_force", 1, true);


    // ringbuffer visualizer
    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("/local_map_visualizer/occupied", 5, true);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("/local_map_visualizer/free", 5, true);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("/local_map_visualizer/distance", 5, true);
    norm_marker_pub = nh.advertise<visualization_msgs::Marker>("/local_map_visualizer/normal", 5, true);

    // synchronized subscriber for pointcloud and odometry
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/mavros/local_position/odom", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/camera/depth/color/points", 1);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pcl_sub);
    sync.registerCallback(boost::bind(&odomCloudCallback, _1, _2));

    // get parameter
    nh.getParam("/dmapping/map_rate", map_rate);
    nh.getParam("/dmapping/publish_rate", pub_rate);
    std::cout << map_rate << "," << pub_rate << std::endl;

    // timer for publish ringbuffer as pointcloud
    ros::Timer timer = nh.createTimer(ros::Duration(1 / pub_rate), timerCallback);

    ros::Duration(0.5).sleep();
    _last_time = ros::Time::now();
    std::cout << "Start mapping!" << std::endl;

    ros::spin();

    return 0;
}