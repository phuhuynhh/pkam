#ifndef PLANNING_CLIENT_H
#define PLANNING_CLIENT_H

#include "dplanning.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class DPlanning; // Forward declaration because of circular reference

class PlanningClient
{
  public:
    PlanningClient(int &argc, char **argv);
    PlanningClient(int &argc, char **argv, ros::NodeHandle *node_handle);

    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber global_pos_sub;

    // end point for path-planning.
    ros::Subscriber getpoint_target_sub;
    // pulbic point to move
    ros::Publisher setpoint_pos_pub;

    ros::Publisher traj_marker_pub;



    void init(DPlanning *const drone_planning);

    DPlanning *drone_planning;

    void create_status_timer();
    void publish_position_to_controller(const geometry_msgs::PoseStamped& setpoint_pos_ENU);
    void publishVisualize();
    void setParam(const std::string &key, double d);
    bool avoidCollision_ = true;

  private:
    ros::NodeHandle *nh_;
};

#endif /* ROS_CLIENT_H */
