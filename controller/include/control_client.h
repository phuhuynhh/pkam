#ifndef CONTROL_CLIENT_H
#define CONTROL_CLIENT_H

#include "dcontroller.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class DController; // Forward declaration because of circular reference

class ControlClient
{
  public:
    ControlClient(int &argc, char **argv);
    ControlClient(int &argc, char **argv, ros::NodeHandle *node_handle);

    ros::Subscriber state_sub;
    ros::Subscriber extended_state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber global_pos_sub;

    // for path planning.
    ros::Subscriber getpoint_pos_sub;
    ros::Publisher setpoint_pos_local_pub;
    // get end point for command.
    ros::Publisher endpoint_pos_pub;


    ros::ServiceClient arming_client;
    ros::ServiceClient land_client;
    ros::ServiceClient set_mode_client;


    ros::Timer offboard_status_timer;
    void init(DController *const drone_control);

    DController *drone_control;

    void create_status_timer();
    void publishTrajectoryEndpoint(const geometry_msgs::PoseStamped& setpoint_pos_ENU);
    void setParam(const std::string &key, double d);
    bool avoidCollision_;

  private:
    ros::NodeHandle *nh_;
};

#endif /* ROS_CLIENT_H */
