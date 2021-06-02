#ifndef MAPPING_H
#define MAPPING_H


#include "ewok/ed_ring_buffer.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>


class Mapping
{
public:
    Mapping();

    // global declaration
    ros::Time _last_time;

    bool initialized = false;
    const double resolution = 0.1;
    static const int POW = 6;
    static const int N = (1 << POW);

    ewok::EuclideanDistanceRingBuffer<POW> *rrb;


    void init();

    void draw_global_trajectory();
    void setParam(const std::string &key, double d);
    void odomCloudCallback(const geometry_msgs::PoseStampedConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud);
    bool avoidCollision_ = true;

    ros::NodeHandle *nh_;

    geometry_msgs::PoseStamped d_local_position;

    ros::Subscriber local_pos_sub;
    ros::Subscriber pcl_sub;

    // Visualizer Public
    ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, norm_marker_pub;

};

#endif /* ROS_CLIENT_H */
