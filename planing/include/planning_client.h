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
    ros::Subscriber local_waypoint_sub;
    ros::Subscriber global_trigger_sub;

    ros::Subscriber local_waypoint_sub1;
    ros::Subscriber local_waypoint_sub2;
    ros::Subscriber local_waypoint_sub3;
    ros::Subscriber local_waypoint_sub4; 

    // end point for path-planning.
    ros::Subscriber getpoint_target_sub;

    // pulbic point to move
    ros::Publisher setpoint_pos_pub;
    //setPointRaw.
    ros::Publisher raw_reference_pub;

    // Visualize Marker Public.
    ros::Publisher grid_marker_pub;
    ros::Publisher global_traj_pub;
    ros::Publisher local_traj_pub;
    ros::Publisher vel_marker_pub;
    ros::Publisher way_points_pub;
    ros::Publisher local_way_points_pub;


    // Visualize Trajectory
    ros::Publisher globalmarker_start_pub;
    ros::Publisher globalmarker_target_pub;

    ros::Publisher localmarker_start_pub;
    ros::Publisher localmarker_target_pub;

    ros::Publisher global_waypoints_pub1;
    ros::Publisher global_waypoints_pub2;
    ros::Publisher global_waypoints_pub3;
    ros::Publisher global_waypoints_pub4;

    ros::Publisher global_trajectory_pub1;
    ros::Publisher global_trajectory_pub2;
    ros::Publisher global_trajectory_pub3;
    ros::Publisher global_trajectory_pub4;

    ros::Publisher local_trajectory_pub1;
    ros::Publisher local_trajectory_pub2;
    ros::Publisher local_trajectory_pub3;
    ros::Publisher local_trajectory_pub4;

    //Local-Map visualize marker
    ros::Publisher occ_marker_pub;
    ros::Publisher free_marker_pub;
    ros::Publisher dist_marker_pub;

    //Local map publisher
    //Publisher send tracjetory pose to ring buffer
    ros::Publisher traj_subset_pub;
    //Publish local goal to ringbuffer;
    ros::Publisher local_target_pub;
    ros::Publisher local_astar_active_pub;
    ros::Publisher local_apf_active_pub;
    ros::Publisher local_rrt_active_pub;

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
