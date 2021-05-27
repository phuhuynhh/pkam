#ifndef PLANNING_CLIENT_H
#define PLANNING_CLIENT_H

#include "dplanning.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/Octomap.h>


class DPlanning; // Forward declaration because of circular reference

class PlanningClient
{
  public:
    PlanningClient(int &argc, char **argv);
    PlanningClient(int &argc, char **argv, ros::NodeHandle *node_handle);


    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber global_pos_sub;
    ros::Subscriber local_octomap_sub;
    ros::Subscriber octomap_sub;

    // end point for path-planning.
    ros::Subscriber getpoint_target_sub;

    // pulbic point to move
    ros::Publisher setpoint_pos_pub;

    // Visualize Marker Public.
    ros::Publisher grid_marker_pub;
    ros::Publisher global_traj_marker_pub;
    ros::Publisher global_traj_pub;
    ros::Publisher vel_marker_pub;



    DPlanning *drone_planning;
    void init(DPlanning *const drone_planning);




    void create_status_timer();
    void publish_position_to_controller(const geometry_msgs::PoseStamped& setpoint_pos_ENU);
    void draw_global_trajectory();
    void setParam(const std::string &key, double d);
    bool avoidCollision_ = true;

    ros::NodeHandle *nh_;
};

#endif /* ROS_CLIENT_H */
