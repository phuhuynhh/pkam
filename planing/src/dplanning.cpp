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

#include <octomap/octomap.h>
#include "path_planning/Astar.h"
#include <string>
#include <chrono>

#include "path_planning/planning_setup.h"
#include "ompl/geometric/PathGeometric.h"

#include <octomap_msgs/Octomap.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

DPlanning::DPlanning(PlanningClient * ros_client){
	this->ros_client = ros_client;
	this->ros_client->init(this);

	// The setpoint publishing rate MUST be faster than 2Hz
	this->rate_ = new ros::Rate(ROS_RATE);

	static tf2_ros::TransformListener tfListener(tfBuffer_);

	this->nh_ = new ros::NodeHandle("~");
	this->nh_->getParam("/dplanning/planning", this->planning);
	global_trajectory.header.frame_id = "map";

	switch(this->planning){
		case 3:
		{
			this->grid = new Grid3D(400,400,100,0.8);
			break;
		}
		case 2:
		{
			this->grid = new Grid3D(400,400,100,0.8);
			break;
		}
		case 1:{
			this->grid = new Grid3D(400,400,100,0.4);
			break;
		}
		default:
		{
			this->grid = new Grid3D(400,400,100,0.4);
			break;
		}
	}
	this->grid->Initilize(octomap::point3d(0.0,0.0,0.0));
	_prev_goal[0] = 0.0;
  	_prev_goal[1] = 7.0;
  	_prev_goal[2] = 2.18;
  	_prev_goal[3] = 1;
  	_prev_goal[4] = 0;
  	_prev_goal[5] = 0;
  	_prev_goal[6] = 0;
 }

void DPlanning::run(){

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

	this->grid->insertOctomapCloud(temp_cloud);

	if (endpoint_active){
		if (distance(d_local_position,endpoint_pos_ENU) < 0.2){
			double travel_time = ros::Time::now().toSec() - start_time.toSec();
			ROS_INFO("Distance : %f", distance(d_local_position,endpoint_pos_ENU));
			ROS_INFO("Time : %f", travel_time);
			ROS_INFO("Travel distance : %f", travel_cost);
			ROS_INFO("Best exec time : %d", best_duration);
			ROS_INFO("Worst exec time : %d", worst_duration);
			endpoint_active = false;
			ROS_INFO("Finished.");
			// this->removeVisualize();
			switch(this->planning){
				case 3:
				{
					this->planning_type = PLANNING_TYPE::RRT;
					break;
				}
				case 2:
				{
					this->planning_type = PLANNING_TYPE::ASTAR;
					break;
				}
				case 1:{
					this->planning_type = PLANNING_TYPE::POTENTIAL_FIELD;
					break;
				}
				default:
				{
					this->planning_type = PLANNING_TYPE::POTENTIAL_FIELD;
					break;
				}
			}
			delete astar;
			return;
		}
		double dt = ros::Time::now().toSec() - pre_time.toSec();
		pre_time = ros::Time::now();
		travel_cost += distance(d_local_position, d_previous_position);
		d_previous_position = d_local_position;

		global_trajectory.poses.push_back(d_local_position);
		ros_client->global_traj_pub.publish(global_trajectory);

		switch(this->planning_type){
			case PLANNING_TYPE::TAKE_OFF:
			{
				geometry_msgs::Point start, end;
				start.x = d_local_position.pose.position.x;
				start.y = d_local_position.pose.position.y;
				start.z = d_local_position.pose.position.z;

				end.x = endpoint_pos_ENU.pose.position.x;
				end.y = endpoint_pos_ENU.pose.position.y;
				end.z = endpoint_pos_ENU.pose.position.z;

				// points.points.push_back(start);
				// line_strip.points.push_back(start);

				float stamped = distance(d_local_position, endpoint_pos_ENU)/0.4;

				vx = (end.x - (float)d_local_position.pose.position.x) / stamped;
				vy = (end.y - (float)d_local_position.pose.position.y) / stamped;
				vz = (end.z - (float)d_local_position.pose.position.z) / stamped;

				setpoint_pos_ENU.pose.position.x = d_local_position.pose.position.x + vx;
				setpoint_pos_ENU.pose.position.y = d_local_position.pose.position.y + vy;
				setpoint_pos_ENU.pose.position.z = d_local_position.pose.position.z + vz;

				// publishVisualize();
				ros_client->publish_position_to_controller(setpoint_pos_ENU);

				break;
			}
			case PLANNING_TYPE::POTENTIAL_FIELD:
			{
				auto start = std::chrono::high_resolution_clock::now();
				octomap::point3d v = this->apf->calculate_velocity(
					octomap::point3d(d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z),
					octomap::point3d(endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z)
				);
				auto end = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

				if(duration.count() >  worst_duration) worst_duration = duration.count();
				if(duration.count() <  best_duration) best_duration = duration.count();

				ROS_INFO("APF run time: %d", duration);

				setpoint_pos_ENU.pose.position.x = d_local_position.pose.position.x + v.x();
				setpoint_pos_ENU.pose.position.y = d_local_position.pose.position.y + v.y();
				setpoint_pos_ENU.pose.position.z = d_local_position.pose.position.z + v.z();

				// publishVisualize();
				ros_client->publish_position_to_controller(setpoint_pos_ENU);


				break;
			}
			case PLANNING_TYPE::ASTAR:
			{
				std::vector<int> path;

				auto start = std::chrono::high_resolution_clock::now();
				bool find = astar->find_path(octomap::point3d(d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z),
					path, 1000);
				auto end = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
				ROS_INFO("Astar run time: %d", duration);

				if(duration.count() >  worst_duration) worst_duration = duration.count();
				if(duration.count() <  best_duration) best_duration = duration.count();

				if(find){
						if(path.size() == 0){
							ros_client->publish_position_to_controller(endpoint_pos_ENU);
						}
						else{
							octomap::point3d newpos = grid->toPosition(path[0]);
							octomap::point3d nowpos(d_local_position.pose.position.x,
							d_local_position.pose.position.y,
						d_local_position.pose.position.z);
							octomap::point3d nextpos = newpos - nowpos;
							nextpos.normalize();
							nextpos = nextpos*0.5;
							nextpos = nowpos + nextpos;
							setpoint_pos_ENU.pose.position.x = nextpos.x();
							setpoint_pos_ENU.pose.position.y = nextpos.y();
							setpoint_pos_ENU.pose.position.z = nextpos.z();
							ros_client->publish_position_to_controller(setpoint_pos_ENU);

							int marr_index = 0;

							for(std::vector<int>::const_iterator it = path.begin(); it != path.end(); ++it){
								octomap::point3d position = grid->toPosition(*it);
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
								mk.scale.x = 0.2;
								mk.scale.y = 0.2;
								mk.scale.z = 0.2;
								mkarr.markers.push_back(mk);
							}
							ros_client->grid_marker_pub.publish(mkarr);
						}
					}
					else{
						ROS_INFO("ASTAR FAILED TO GENERATE PATH");
					}
					break;
			}
			case PLANNING_TYPE::RRT:
			{
				// ros_client->publish_position_to_controller(d_local_position);
				//
				// PlanningSetup planning_setup;
				// planning_setup.setOctomapValidator(this->octomap_msgs);
				//
				// ompl::base::RealVectorBounds bounds(3);
				//
				// bounds.setLow(0, -20);
    		// bounds.setLow(1, -20);
    		// bounds.setLow(2, -20);
				//
    		// bounds.setHigh(0, 20);
    		// bounds.setHigh(1, 20);
    		// bounds.setHigh(2, 20);
				// planning_setup.getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
				//
				// // planning_setup.setStateValidityCheckingResolution(0.1);
				//
				// ompl::base::ScopedState<ompl::base::SE3StateSpace> start_ompl(
		 		// 			planning_setup.getSpaceInformation());
 	 			// ompl::base::ScopedState<ompl::base::SE3StateSpace> goal_ompl(
		 		// 			planning_setup.getSpaceInformation());
				//
				// start_ompl->setXYZ(d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z);
				// start_ompl->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
				//
				// goal_ompl->setXYZ(endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z);
				// goal_ompl->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
				//
				// planning_setup.setStartState(start_ompl);
				// planning_setup.setGoalState(goal_ompl);
				//
				// planning_setup.setDefaultObjective();
				//
				// planning_setup.setRrtStar();
				// planning_setup.setup();
				// planning_setup.solve(0.05);
				//
				// if(planning_setup.haveSolutionPath()){
				// 	int marr_index = 0;
				//
				// 	og::PathGeometric path = planning_setup.getSolutionPath();
				// 	for(std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++){
				// 		const ob::SE3StateSpace::StateType *state = path.getState(path_idx)->as<ob::SE3StateSpace::StateType>();
				// 		const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>(0);
				// 		const ob::SO3StateSpace::StateType *rot = state->as<ob::SO3StateSpace::StateType>(1);
				//
				// 		visualization_msgs::Marker mk;
				// 		mk.id = marr_index;
				// 		mk.type = mk.CUBE;
				// 		marr_index += 1;
				// 		mk.header.frame_id = "map";
				// 		mk.pose.position.x = pos->values[0];
				// 		mk.pose.position.y = pos->values[1];
				// 		mk.pose.position.z = pos->values[2];
				// 		mk.color.r = 1.0;
				// 		mk.color.a = 1.0;
				// 		mk.scale.x = 0.2;
				// 		mk.scale.y = 0.2;
				// 		mk.scale.z = 0.2;
				// 		mkarr.markers.push_back(mk);
				// 	}
				//
				// 	ros_client->grid_marker_pub.publish(mkarr);
				// }
				// else{
				// 	ROS_INFO("FAILED TO FIND PATH WITH OMPL-RRT");
				// }

				ob::StateSpacePtr space = ob::StateSpacePtr(new ob::SE3StateSpace());

				// create a start state
				ob::ScopedState<ob::SE3StateSpace> start(space);

				// create a goal state
				ob::ScopedState<ob::SE3StateSpace> goal(space);

				// set the bounds for the R^3 part of SE(3)
				ob::RealVectorBounds bounds(3);

				bounds.setLow(0, _min_bounds[0]);   // x min
  				bounds.setHigh(0, _max_bounds[0]);  // x max
  				bounds.setLow(1, _min_bounds[1]);   // y min
  				bounds.setHigh(1, _max_bounds[1]);  // y max
  				bounds.setLow(2, _min_bounds[2]);   // z min
  				bounds.setHigh(2, _max_bounds[2]);  // z max

				space->as<ob::SE3StateSpace>()->setBounds(bounds);

				// construct an instance of  space information from this state space
				ob::SpaceInformationPtr si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

				start->setXYZ(d_local_position.pose.position.x,
											d_local_position.pose.position.y,
											d_local_position.pose.position.z);
				start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
				// start.random();

				goal->setXYZ(	endpoint_pos_ENU.pose.position.x,
											endpoint_pos_ENU.pose.position.y,
											endpoint_pos_ENU.pose.position.z);
				goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
				// goal.random();

			    // set state validity checking for this space
				std::shared_ptr<OctomapStateValidator> validity_checker(new OctomapStateValidator(si, this->octomap_msgs));
				si->setStateValidityChecker(validity_checker);
				si->setStateValidityCheckingResolution(0.01);
				si->setMotionValidator(std::make_shared<OctomapMotionValidator>(si, this->octomap_msgs));
				si->setup();							
				// create a problem instance
				ob::ProblemDefinitionPtr pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

				// set the start and goal states
				pdef->setStartAndGoalStates(start, goal);

			    // set Optimizattion objective
				ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
				obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);

				pdef->setOptimizationObjective(obj);

				ob::PlannerPtr plan(new og::InformedRRTstar(si));

	    	// set the problem we are trying to solve for the planner
				plan->setProblemDefinition(pdef);

			    // perform setup steps for the planner
				plan->setup();

			    // attempt to solve the problem within one second of planning time
				ob::PlannerStatus solved = plan->solve(2);

				if(solved){
					int marr_index = 0;

					mav_trajectory_generation::NonlinearOptimizationParameters parameters;
					mav_trajectory_generation ::Vertex::Vector vertices;
					const int dimension = 3;
					const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
					// we have 2 vertices:
					// Start = current position
					// end = desired position and velocity
					mav_trajectory_generation::Vertex start(dimension), end(dimension);	


					og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
					for(std::size_t path_idx = 0; path_idx < path->getStateCount(); path_idx++){
						const ob::SE3StateSpace::StateType *se3state = path->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

						// extract the first component of the state and cast it to what we expect
						const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

						// extract the second component of the state and cast it to what we expect
						const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

						if(path_idx == 0){
							start.makeStartOrEnd(Eigen::Vector3d(pos->values[0],pos->values[1],pos->values[2]), derivative_to_optimize);
							// set start point's velocity to be constrained to current velocity
  							start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,Eigen::Vector3d::Zero());
							vertices.push_back(start);  
						}
						else if(path_idx == path->getStateCount() - 1){
							end.makeStartOrEnd(Eigen::Vector3d(pos->values[0],pos->values[1],pos->values[2]), derivative_to_optimize);
							// set start point's velocity to be constrained to current velocity
							end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,Eigen::Vector3d(1,1,0.1));
							vertices.push_back(end);
						}
						else{
							mav_trajectory_generation::Vertex vertex(dimension);
							vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(pos->values[0],pos->values[1],pos->values[2]));
							vertices.push_back(vertex);
						}

						visualization_msgs::Marker mk;
						mk.id = marr_index;
						mk.type = mk.CUBE;
						marr_index += 1;
						mk.header.frame_id = "map";
						mk.pose.position.x = pos->values[0];
						mk.pose.position.y = pos->values[1];
						mk.pose.position.z = pos->values[2];
						mk.color.r = 1.0;
						mk.color.a = 1.0;
						mk.scale.x = 0.2;
						mk.scale.y = 0.2;
						mk.scale.z = 0.2;
						mkarr.markers.push_back(mk);
					}

					// // setimate initial segment times
  					// std::vector<double> segment_times;
  					// segment_times = estimateSegmentTimes(vertices, 2.0, 2.0);
					// 	// set up optimization problem
					// const int N = 10;
					// mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
					// opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

					//  // constrain velocity and acceleration
  					// opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 2.0);
  					// opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, 2.0);

  					// // solve trajectory
  					// opt.optimize();

 					// mav_trajectory_generation::Trajectory trajectory;
					// opt.getTrajectory(&trajectory);
					// ros_client->grid_marker_pub.publish(mkarr);

					// mav_msgs::EigenTrajectoryPoint::Vector states;
					// // Whole trajectory:
					// double sampling_interval = 0.01;
					// bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

					// double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
					// std::string frame_id = "map";

					// // From Trajectory class:
					// mav_trajectory_generation::drawMavTrajectory(trajectory, 0.0, frame_id, &mkarr);
					ros_client->grid_marker_pub.publish(mkarr);
					break;
				
				}
				else{
					ROS_INFO("FAILED TO FIND PATH WITH OMPL-RRT");
				}


				break;
			}
		}
	}
}

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

		if (msg->pose.position.x > _min_bounds[0] && msg->pose.position.x < _max_bounds[0] &&
      		msg->pose.position.y > _min_bounds[1] && msg->pose.position.y < _max_bounds[1] &&
      		msg->pose.position.z > _min_bounds[2] && msg->pose.position.z < _max_bounds[2])
			  {
				endpoint_pos_ENU.pose.position.x = msg->pose.position.x;
				endpoint_pos_ENU.pose.position.y = msg->pose.position.y;
				endpoint_pos_ENU.pose.position.z = msg->pose.position.z;
			  }
		else{
			endpoint_pos_ENU.pose.position.x = _prev_goal[0];
			endpoint_pos_ENU.pose.position.y = _prev_goal[1];
			endpoint_pos_ENU.pose.position.z = _prev_goal[2];
		}



		ROS_INFO("Requested trajectory : \n start (x,y,z) : %f %f %f \n stop (x,y,z): %f %f %f",
			d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z,
			endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z);

		start_time = ros::Time::now();
		pre_time = ros::Time::now();
		startpoint_pos_ENU = d_local_position;
		d_previous_position = d_local_position;
		travel_cost = 0.0;
		astar = new Astar(
			octomap::point3d(endpoint_pos_ENU.pose.position.x,
				endpoint_pos_ENU.pose.position.y,
				endpoint_pos_ENU.pose.position.z),
			this->grid);
		apf = new APF(this->grid);
		// publishVisualize();

		endpoint_active = true;
		best_duration = 1000000000000;
		worst_duration = 0;
		global_trajectory.poses.clear();
	}
}

void DPlanning::octomap_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
	this->octomap_cloud = *msg;
}

void DPlanning::full_octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg){
	this->octomap_msgs = msg;
	// ROS_INFO("OCTOMAP CALLBACK 1");
  	// // convert ColorOcTree to OcTree
  	// octomap::OcTree* tree_oct = reinterpret_cast<octomap::OcTree*>(tree_coloct);
  	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
  	
  	tree_oct->getMetricMin(_min_bounds[0], _min_bounds[1], _min_bounds[2]);
  	tree_oct->getMetricMax(_max_bounds[0], _max_bounds[1], _max_bounds[2]);
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

bool DPlanning::isStateValid(const ompl::base::State *state)
	{
	  //   // cast the abstract state type to the type we expect
		// const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
		//
	  //   // extract the first component of the state and cast it to what we expect
		// const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
		//
	  //   // extract the second component of the state and cast it to what we expect
		// const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
		//
		// fcl::CollisionObject treeObj((tree_obj));
		// fcl::CollisionObject aircraftObject(Quadcopter);
		//
	  //   // check validity of state defined by pos & rot
		// fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
		// fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
		// aircraftObject.setTransform(rotation, translation);
		// fcl::CollisionRequest requestType(1,false,1,false);
		// fcl::CollisionResult collisionResult;
		// fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
		//
		// return(!collisionResult.isCollision());
		return true;
	}
