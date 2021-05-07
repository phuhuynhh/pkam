#include "dplanning.h"

#include <cmath>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


DPlanning::DPlanning(PlanningClient * ros_client){
	this->ros_client = ros_client;
	this->grid.Initilize(octomap::point3d(0.0,0.0,0.0));
	this->ros_client->init(this);

	// The setpoint publishing rate MUST be faster than 2Hz
	this->rate_ = new ros::Rate(ROS_RATE);

	static tf2_ros::TransformListener tfListener(tfBuffer_);
}

void DPlanning::run(){
	if (octomap_activate){
		update_grid_map();
	}	

	if (endpoint_active){
		if (distance(d_local_position,endpoint_pos_ENU) < 0.2f){
			printf("Distance : %f \n", distance(d_local_position,endpoint_pos_ENU));
			endpoint_active = false;
			printf("Finished.");
			remove_global_trajectory();
			this->planning_type = PLANNING_TYPE::POTENTIAL_FIELD;
		}
		double dt = ros::Time::now().toSec() - pre_time.toSec();
		pre_time = ros::Time::now();

		switch(this->planning_type){
			case PLANNING_TYPE::TAKE_OFF:
			{

				float stamped = distance(d_local_position, endpoint_pos_ENU) / 0.5;

				vx = (float)(endpoint_pos_ENU.pose.position.x - (float)d_local_position.pose.position.x) / stamped;
				vy = (float)(endpoint_pos_ENU.pose.position.y - (float)d_local_position.pose.position.y) / stamped;
				vz = (float)(endpoint_pos_ENU.pose.position.z - (float)d_local_position.pose.position.z) / stamped;

				setpoint_pos_ENU.pose.position.x = d_local_position.pose.position.x + vx;
				setpoint_pos_ENU.pose.position.y = d_local_position.pose.position.y + vy;
				setpoint_pos_ENU.pose.position.z = d_local_position.pose.position.z + vz;




				//After calculate setpoint_pos_ENU, send for controller
				ros_client->publish_position_to_controller(setpoint_pos_ENU);
				break;
			}
			case PLANNING_TYPE::POTENTIAL_FIELD:
			{
				octomap::point3d v = this->apf.calculate_velocity(
					octomap::point3d(d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z),
					octomap::point3d(endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z)
					);


				vx = v.x();
				vy = v.y();
				vz = v.z();

				setpoint_pos_ENU.pose.position.x = d_local_position.pose.position.x + vx;
				setpoint_pos_ENU.pose.position.y = d_local_position.pose.position.y + vy;
				setpoint_pos_ENU.pose.position.z = d_local_position.pose.position.z + vz;


		//After calculate setpoint_pos_ENU, send for controller
				ros_client->publish_position_to_controller(setpoint_pos_ENU);
				break;
			}
		}

		// printf("velocity : (%f, %f, %f)\n", vx, vy, vz);
		draw_velocitty();

	}
}

void DPlanning::update_grid_map(){
	//update the grid
	tf::TransformListener m_tfListener;

	pcl::PointCloud<pcl::PointXYZ> temp_cloud;
	pcl::fromROSMsg(octomap_cloud,temp_cloud);

	visualization_msgs::MarkerArray mkarr;

	tf::StampedTransform sensorToWorldTf;
	try {
		m_tfListener.lookupTransform(m_worldFrameId, this->octomap_cloud.header.frame_id, this->octomap_cloud.header.stamp, sensorToWorldTf);
	} catch(tf::TransformException& ex){
		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
	pcl::transformPointCloud(temp_cloud, temp_cloud, sensorToWorld);

	this->grid.insertOctomapCloud(temp_cloud);
}


void DPlanning::draw_global_trajectory(){
	global_trajectory_line.header.frame_id = "map";
	global_trajectory_line.header.stamp = ros::Time::now();
	global_trajectory_line.ns = "global_trajectory";

	remove_global_trajectory();

	global_trajectory_line.action  = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = global_trajectory_line.pose.orientation.w = 1.0;

	global_trajectory_line.id = 1;

	global_trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	global_trajectory_line.scale.x = 0.08f;


	// Line strip is blue
	global_trajectory_line.color.b = 1.0f;
	global_trajectory_line.color.a = 1.0f;


	geometry_msgs::Point start, end;
	start.x = startpoint_pos_ENU.pose.position.x;
	start.y = startpoint_pos_ENU.pose.position.y;
	start.z = startpoint_pos_ENU.pose.position.z;
	global_trajectory_line.points.push_back(start);


	end.x = endpoint_pos_ENU.pose.position.x;
	end.y = endpoint_pos_ENU.pose.position.y;
	end.z = endpoint_pos_ENU.pose.position.z;
	global_trajectory_line.points.push_back(end);





	ros_client->global_traj_marker_pub.publish(global_trajectory_line);
};

void DPlanning::remove_global_trajectory(){
	global_trajectory_line.header.stamp = ros::Time::now();
	global_trajectory_line.action  = visualization_msgs::Marker::DELETEALL;
	ros_client->global_traj_marker_pub.publish(global_trajectory_line);

}

void DPlanning::draw_velocitty(){
	velocity_vector.header.frame_id = "map";
	velocity_vector.header.stamp = ros::Time::now();
	velocity_vector.ns = "velocity";

	velocity_vector.action=  visualization_msgs::Marker::DELETEALL;
	ros_client->vel_marker_pub.publish(velocity_vector);

	velocity_vector.header.stamp = ros::Time::now();
	velocity_vector.action  = visualization_msgs::Marker::ADD;
	velocity_vector.pose.orientation.w = 1.0;

	velocity_vector.id = 2;

	velocity_vector.type = visualization_msgs::Marker::ARROW;
	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	velocity_vector.scale.x = 0.2f;


	// Line strip is blue
	velocity_vector.color.r = 1.0f;
	velocity_vector.color.a = 1.0f;

	velocity_vector.points.clear();

	geometry_msgs::Point start, end;
	start.x = d_local_position.pose.position.x;
	start.y = d_local_position.pose.position.y;
	start.z = d_local_position.pose.position.z;
	velocity_vector.points.push_back(start);


	end.x = start.x + vx;
	end.y = start.y + vy;
	end.z = start.z + vz;
	velocity_vector.points.push_back(end);

	ros_client->vel_marker_pub.publish(velocity_vector);
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
}

void DPlanning::global_position_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	d_global_position = *msg;
}

void DPlanning::get_target_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
	if (!endpoint_active){
		endpoint_active = true;
		endpoint_pos_ENU = *msg;
		printf("Requested trajectory : \n start (x,y,z) : %f %f %f \n stop (x,y,z): %f %f %f",
			d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z,
			endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z);

		start_time = ros::Time::now();
		pre_time = ros::Time::now();
		startpoint_pos_ENU = d_local_position;

		draw_global_trajectory();
	}
}

void DPlanning::octomap_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
	if (!octomap_activate){
		octomap_activate = true;
	}
	this->octomap_cloud = *msg;
}

/*
void draw_grid(){
	int marr_index = 0;
	int index = 0;
	for(std::vector<int>::const_iterator it = this->grid.occupied_nodes.begin(); it != this->grid.occupied_nodes.end(); ++it){
		if(*it == 1){
			octomap::point3d position = this->grid.toPosition(index);
			visualization_msgs::Marker mk;
			mk.id = marr_index;
			mk.type = mk.CUBE;
			marr_index += 1;
			mk.header.frame_id = "map";
			mk.pose.position.x = position.x();
			mk.pose.position.y = position.y();
			mk.pose.position.z = position.z();
			mk.color.r = 1.0;
			mk.color.a = 1.0;
			mk.color.g = 0.0;
			mk.color.b = 0.0;
			mk.scale.x = 0.2;
			mk.scale.y = 0.2;
			mk.scale.z = 0.2;
			mkarr.markers.push_back(mk);
		}
		index++;
	}
	this->ros_client->grid_pub.publish(mkarr);
}

*/

/*
void drawPoint(){
		points.header.frame_id = global_trajectory_line.header.frame_id = "map";
	points.header.stamp = global_trajectory_line.header.stamp = ros::Time::now();
	points.ns = global_trajectory_line.ns = "points_and_lines";
	points.action = global_trajectory_line.action  = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = global_trajectory_line.pose.orientation.w = 1.0;

	points.id = 0;

	points.type = visualization_msgs::Marker::POINTS;


	// POINTS markers use x and y scale for width/height respectively
	points.scale.x = 0.2f;
	points.scale.y = 0.2f;

	// Points are green
	points.color.g = 1.0f;
	points.color.a = 1.0f;



	geometry_msgs::Point start, end;
	start.x = startpoint_pos_ENU.pose.position.x;
	start.y = startpoint_pos_ENU.pose.position.y;
	start.z = startpoint_pos_ENU.pose.position.z;

	end.x = endpoint_pos_ENU.pose.position.x;
	end.y = endpoint_pos_ENU.pose.position.y;
	end.z = endpoint_pos_ENU.pose.position.z;

	points.points.push_back(start);
	

	float stamped = distance(startpoint_pos_ENU, endpoint_pos_ENU) / 5.0;

	vx = std::abs(end.x - start.x) / stamped;
	vy = std::abs(end.y - start.y) / stamped;
	vz = std::abs(end.z - start.z) / stamped;
	for (int i = 0; i < (int) stamped + 1; i++){
		geometry_msgs::Point point_stamped;
		point_stamped.x = start.x + vx * i;
		point_stamped.y = start.y + vy * i;
		point_stamped.z = start.z + vz * i;
		points.points.push_back(point_stamped);
	}

	points.points.push_back(end);
	ros_client->global_traj_marker_pub.publish(points);
}

*/

/*

tf::Point t1, t2;
				tf::pointMsgToTF(startpoint_pos_ENU.pose.position, t1);
				tf::pointMsgToTF(endpoint_pos_ENU.pose.position, t2);
				// Test yaw control
				double roll, pitch, yaw;
				tf::Quaternion q;

				tf::quaternionMsgToTF(startpoint_pos_ENU.pose.orientation, q);
				tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

				double x,y,z,xzLen;
				xzLen = cos(pitch);
				x = xzLen * cos (yaw);
				y = sin(pitch);
				z = xzLen * sin (-yaw);


				tf::Point d1,d2;
				d1.setX(x);
				d1.setY(y);
				d1.setZ(0);
				d1.normalize();
				d2 = t2.normalize();
				d1.setZ(0);
				d2.setZ(0);
				printf("\n currentYaw : %f ", currentYaw());
				printf("\nAngle of (%f,%f,%f) and (%f,%f,%f) :  %f", d1.x(), d1.y(),d1.z(),d2.x(),d2.y(),d2.z(), d1.angle(d2));
		//Calculate yaw current orientation
				double _angle = d1.angle(d2);
				double new_yaw = yaw + _angle;
				printf("\n newYaw : %f \n", new_yaw);

				if (_angle>=M_PI)  _angle-=2*M_PI;
            	if (_angle<=-M_PI) _angle+=2*M_PI;
				double stamped = _angle / 0.2f;
				double v_yaw = _angle  / stamped;

				tf::Quaternion _new_q;
				geometry_msgs::Quaternion _new_quaternion;

				_new_q.setRPY( roll, pitch, yaw + v_yaw * dt );
				printf("new quaternion : (%f,%f,%f,%f)", _new_q.x(), _new_q.y(), _new_q.z(), _new_q.w() );
				tf::quaternionTFToMsg(_new_q,_new_quaternion);
*/





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
