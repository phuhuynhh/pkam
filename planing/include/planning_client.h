#ifndef PLANNING_CLIENT_H
#define PLANNING_CLIENT_H

#include "dplanning.h"

#include "std_msgs/Int8.h"
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/NavSatFix.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>



using namespace message_filters;
class DPlanning; // Forward declaration because of circular reference

class PlanningClient
{
  public:
    PlanningClient();
    PlanningClient(ros::NodeHandle &node_handle);


    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber global_pos_sub;
    ros::Subscriber local_octomap_sub;
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber pointcloud_sub;


    //localmap
    ros::Subscriber occ_trigger_sub;
    ros::Subscriber apf_force_sub;

    // end point for path-planning.
    ros::Subscriber getpoint_target_sub;

    // pulbic point to move
    ros::Publisher setpoint_pos_pub;
    //setPointRaw.
    ros::Publisher raw_reference_pub;

    // Visualize Marker Public.
    ros::Publisher grid_marker_pub;
    ros::Publisher global_traj_pub;
    ros::Publisher vel_marker_pub;
    ros::Publisher way_points_pub;

    //Local-Map visualize marker
    ros::Publisher occ_marker_pub;
    ros::Publisher free_marker_pub;
    ros::Publisher dist_marker_pub;



    DPlanning *drone_planning;
    void init(DPlanning *const drone_planning);




    void create_status_timer();
    void publish_position_to_controller(const geometry_msgs::PoseStamped& setpoint_pos_ENU);
    void publish_raw_position_target(const mavros_msgs::PositionTarget& raw_pos_target);
    void draw_global_trajectory();
    void setParam(const std::string &key, double d);
    bool avoidCollision_ = true;

    ros::NodeHandle *nh_;
};

#endif /* ROS_CLIENT_H */
