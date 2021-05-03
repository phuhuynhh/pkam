#include "dplanning.h"

#include <cmath>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>


DPlanning::DPlanning(PlanningClient * ros_client){
	this->ros_client = ros_client;
	this->grid.Initilize(octomap::point3d(0.0,0.0,0.0));
	this->ros_client->init(this);

	// The setpoint publishing rate MUST be faster than 2Hz
	this->rate_ = new ros::Rate(ROS_RATE);

	static tf2_ros::TransformListener tfListener(tfBuffer_);
}

void DPlanning::run(){
	if (endpoint_active){
		ROS_INFO("Distance : %f", distance(d_local_position,endpoint_pos_ENU));
		if (distance(d_local_position,endpoint_pos_ENU) < 0.5){
			endpoint_active = false;
			ROS_INFO("Finished.");
			removeVisualize();
			this->planning_type = PLANNING_TYPE::POTENTIAL_FIELD;
		}
		double dt= ros::Time::now().toSec() - start_time.toSec();

		switch(this->planning_type){
			case PLANNING_TYPE::SIMPLE:
			{
				geometry_msgs::Point start, end;
				start.x = startpoint_pos_ENU.pose.position.x;
				start.y = startpoint_pos_ENU.pose.position.y;
				start.z = startpoint_pos_ENU.pose.position.z;

				end.x = endpoint_pos_ENU.pose.position.x;
				end.y = endpoint_pos_ENU.pose.position.y;
				end.z = endpoint_pos_ENU.pose.position.z;

				points.points.push_back(start);
				line_strip.points.push_back(start);

				float stamped = distance(startpoint_pos_ENU, endpoint_pos_ENU) / 0.2f;

				vx = std::abs(end.x - start.x) / stamped;
				vy = std::abs(end.y - start.y) / stamped;
				vz = std::abs(end.z - start.z) / stamped;

				setpoint_pos_ENU.pose.position.x = startpoint_pos_ENU.pose.position.x + vx * dt;
				setpoint_pos_ENU.pose.position.y = startpoint_pos_ENU.pose.position.y + vy * dt;
				setpoint_pos_ENU.pose.position.z = startpoint_pos_ENU.pose.position.z + vz * dt;

				publishVisualize();
				ros_client->publish_position_to_controller(setpoint_pos_ENU);
				break;
			}
			case PLANNING_TYPE::POTENTIAL_FIELD:
			{
				octomap::point3d v = this->apf.calculate_velocity(
					octomap::point3d(startpoint_pos_ENU.pose.position.x, startpoint_pos_ENU.pose.position.y, startpoint_pos_ENU.pose.position.z),
					octomap::point3d(endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z)
				);

				setpoint_pos_ENU.pose.position.x = startpoint_pos_ENU.pose.position.x + v.x() * dt;
				setpoint_pos_ENU.pose.position.y = startpoint_pos_ENU.pose.position.y + v.y() * dt;
				setpoint_pos_ENU.pose.position.z = startpoint_pos_ENU.pose.position.z + v.z() * dt;

				// publishVisualize();
				ros_client->publish_position_to_controller(setpoint_pos_ENU);
			}
		}
	}
}


void DPlanning::publishVisualize(){
	points.header.frame_id = line_strip.header.frame_id = "map";
	points.header.stamp = line_strip.header.stamp = ros::Time::now();
	points.ns = line_strip.ns = "points_and_lines";
	points.action = line_strip.action  = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

	points.id = 0;
	line_strip.id = 1;

	points.type = visualization_msgs::Marker::POINTS;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;


	// POINTS markers use x and y scale for width/height respectively
	points.scale.x = 0.2f;
	points.scale.y = 0.2f;

	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	line_strip.scale.x = 0.1f;

	// Points are green
	points.color.g = 1.0f;
	points.color.a = 1.0f;

	// Line strip is blue
	line_strip.color.b = 1.0f;
	line_strip.color.a = 1.0f;


	geometry_msgs::Point start, end;
	start.x = startpoint_pos_ENU.pose.position.x;
	start.y = startpoint_pos_ENU.pose.position.y;
	start.z = startpoint_pos_ENU.pose.position.z;

	end.x = endpoint_pos_ENU.pose.position.x;
	end.y = endpoint_pos_ENU.pose.position.y;
	end.z = endpoint_pos_ENU.pose.position.z;

	points.points.push_back(start);
	line_strip.points.push_back(start);

	float stamped = distance(startpoint_pos_ENU, endpoint_pos_ENU) / 0.2f;

	vx = std::abs(end.x - start.x) / stamped;
	vy = std::abs(end.y - start.y) / stamped;
	vz = std::abs(end.z - start.z) / stamped;
	ROS_INFO("stamped time %f : v(x,y,z) : (%f, %f, %f)", stamped,vx,vy,vz);
	for (int i = 0; i < (int) stamped + 1; i++){
		geometry_msgs::Point point_stamped;
		point_stamped.x = start.x + vx * i;
		point_stamped.y = start.y + vy * i;
		point_stamped.z = start.z + vz * i;
		points.points.push_back(point_stamped);
	}





	points.points.push_back(end);
	line_strip.points.push_back(end);


	ros_client->traj_marker_pub.publish(points);
	ros_client->traj_marker_pub.publish(line_strip);
};

void DPlanning::removeVisualize(){
	points.header.stamp = line_strip.header.stamp = ros::Time::now();
	points.action = line_strip.action  = visualization_msgs::Marker::DELETEALL;

	ros_client->traj_marker_pub.publish(points);
	ros_client->traj_marker_pub.publish(line_strip);
}

//
//  Callback for subcriber.
//

void DPlanning::state_callback(const mavros_msgs::State::ConstPtr &msg)
{
	d_current_state = *msg;
}

void DPlanning::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	d_local_position = *msg;
	if (endpoint_active){

	}
	/*
  	// For debug logger.
	static int cnt = 0;
	cnt++;
	if(cnt % 100 == 0)
	{
    // ROS_INFO("Mavros local position: E: %f, N: %f, U: %f, yaw: %f", d_local_position.pose.position.x,
    //          d_local_position.pose.position.y, d_local_position.pose.position.z, currentYaw());
	}
	*/
}

void DPlanning::global_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	d_global_position = *msg;

	/*
	// For debug logger.
	static int cnt = 0;

	cnt++;
	if(cnt % 100 == 0)
	{
    // ROS_INFO("GPS: lat: %f, long: %f, alt: %f", msg->latitude, msg->longitude, msg->altitude);
	}
	*/
}

void DPlanning::get_target_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
	if (!endpoint_active){
		endpoint_active = true;
		endpoint_pos_ENU = *msg;
		ROS_INFO("Requested trajectory start (x,y,z) : %f %f %f, stop: %f %f %f",
			d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z,
			endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z);

		start_time = ros::Time::now();
		startpoint_pos_ENU = d_local_position;



		// publishVisualize();
	}
}

void DPlanning::octomap_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
	    tf::TransformListener m_tfListener;

	    pcl::PointCloud<pcl::PointXYZ> pc;
	    pcl::fromROSMsg(*msg, pc);

	    visualization_msgs::MarkerArray mkarr;

	    tf::StampedTransform sensorToWorldTf;
	    try {
	      m_tfListener.lookupTransform(m_worldFrameId, msg->header.frame_id, msg->header.stamp, sensorToWorldTf);
	    } catch(tf::TransformException& ex){
	      ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
	      return;
	    }

	    Eigen::Matrix4f sensorToWorld;
	    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
	    pcl::transformPointCloud(pc, pc, sensorToWorld);

	    this->grid.insertOctomapCloud(pc);
	    // int marr_index = 0;
			//
	    // for(std::set<int>::const_iterator it = grid.all_occupied_nodes.begin(); it != grid.all_occupied_nodes.end(); ++it){
	    //   octomap::point3d position = grid.toPosition(*it);
	    //   visualization_msgs::Marker mk;
	    //   mk.id = marr_index;
	    //   mk.type = mk.CUBE;
	    //   marr_index += 1;
	    //   mk.header.frame_id = "map";
	    //   mk.pose.position.x = position.x();
	    //   mk.pose.position.y = position.y();
	    //   mk.pose.position.z = position.z();
	    //   mk.color.r = 1.0;
	    //   mk.color.a = 1.0;
	    //   mk.scale.x = 0.2;
	    //   mk.scale.y = 0.2;
	    //   mk.scale.z = 0.2;
	    //   mkarr.markers.push_back(mk);
	    // }
	    // pub.publish(mkarr);

}






//
//Util Method.
//
double DPlanning::currentYaw()
{
  //Calculate yaw current orientation
	double roll, pitch, yaw;
	tf::Quaternion q;

	tf::quaternionMsgToTF(d_local_position.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	return yaw;
}

double DPlanning::getYaw(const geometry_msgs::Quaternion &msg)
{
  //Calculate yaw current orientation
	double roll, pitch, yaw;
	tf::Quaternion q;

	tf::quaternionMsgToTF(msg, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	return yaw;
}


double DPlanning::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
{
	tf::Point t1, t2;
	tf::pointMsgToTF(p1.pose.position, t1);
	tf::pointMsgToTF(p2.pose.position, t2);

	return t1.distance(t2);
}
