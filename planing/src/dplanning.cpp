#include "dplanning.h"




namespace ob = ompl::base;
namespace og = ompl::geometric;



DPlanning::DPlanning(PlanningClient *ros_client)
{
	this->ros_client = ros_client;
	this->ros_client->init(this);

	// The setpoint publishing rate MUST be faster than 2Hz
	this->rate_ = new ros::Rate(ROS_RATE);

	static tf2_ros::TransformListener tfListener(tfBuffer_);

	this->nh_ = new ros::NodeHandle("~");
	this->nh_->getParam("/dplanning/planning", this->planning);

	// synchronized subscriber for pointcloud and odometry
    // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(*nh_, "/mavros/local_position/odom", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(*nh_, "/camera/depth/color/points", 1);
	// sync.reset(new Sync(MySyncPolicy(10), odom_sub, pcl_sub));      
	// sync->registerCallback(boost::bind(&DPlanning::odomCloudCallback, this, _1, _2));
}

void DPlanning::run()
{

	// Main thread in 20Hz.
	while(ros::ok())
	{
	//update the grid
		tf::TransformListener m_tfListener;

		pcl::PointCloud<pcl::PointXYZ> temp_cloud;

		if (octomap_activate)
		{
			pcl::fromROSMsg(octomap_cloud, temp_cloud);

			tf::StampedTransform sensorToWorldTf;
			try
			{
				m_tfListener.lookupTransform(m_worldFrameId, this->octomap_cloud.header.frame_id, this->octomap_cloud.header.stamp, sensorToWorldTf);
			}
		catch (tf::TransformException &ex)
			{
				ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
				return;
			}

			Eigen::Matrix4f sensorToWorld;
			pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
			pcl::transformPointCloud(temp_cloud, temp_cloud, sensorToWorld);

		// this->grid->insertOctomapCloud(temp_cloud);
		}
		if (endpoint_active)
		{
			travel_cost += distance(d_local_position, d_previous_position);
			d_previous_position = d_local_position;

		// global_trajectory.poses.push_back(d_local_position);
		// ros_client->global_traj_pub.publish(global_trajectory);

			switch (this->planning_type)
			{
				case PLANNING_STEP::TAKE_OFF:
				{
					if (distance(d_local_position, endpoint_pos_ENU) < 0.2)
					{
						double travel_time = ros::Time::now().toSec() - start_time.toSec();
						ROS_INFO("Distance : %f", distance(d_local_position, endpoint_pos_ENU));
						ROS_INFO("Time : %f", travel_time);
						ROS_INFO("Travel distance : %f", travel_cost);
						ROS_INFO("Best exec time : %d", best_duration);
						ROS_INFO("Worst exec time : %d", worst_duration);
						endpoint_active = false;
						ROS_INFO("Finished.");
						this->planning_type = PLANNING_STEP::GLOBAL_PLANNING;
						break;
					}
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

						publishVisualize();

					//Force take-off.
					ros_client->publish_position_to_controller(endpoint_pos_ENU);

					break;
				}
				case PLANNING_STEP::LOCAL_PLANNING:
				{
					// auto start = std::chrono::high_resolution_clock::now();
					// octomap::point3d v = this->apf->calculate_velocity(
					// 	octomap::point3d(d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z),
					// 	octomap::point3d(endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z)
					// );
					// auto end = std::chrono::high_resolution_clock::now();
					// auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

					// if(duration.count() >  worst_duration) worst_duration = duration.count();
					// if(duration.count() <  best_duration) best_duration = duration.count();

					// ROS_INFO("APF run time: %d", duration);

					// setpoint_pos_ENU.pose.position.x = d_local_position.pose.position.x + v.x();
					// setpoint_pos_ENU.pose.position.y = d_local_position.pose.position.y + v.y();
					// setpoint_pos_ENU.pose.position.z = d_local_position.pose.position.z + v.z();

					// // publishVisualize();
					// ros_client->publish_position_to_controller(setpoint_pos_ENU);

					break;
				}
				case PLANNING_STEP::GLOBAL_PLANNING:
				{
					global_trajectory.markers.clear();
					d_way_points.markers.clear();
					space = ob::StateSpacePtr(new ob::SE3StateSpace());

					// create a start state
					ob::ScopedState<ob::SE3StateSpace> start(space);
					// create a goal state
					ob::ScopedState<ob::SE3StateSpace> goal(space);
					// set the bounds for the R^3 part of SE(3)
					ob::RealVectorBounds bounds(3);

					//TODO : Assign Khang VO.
					// config pipeline, ref : https://github.com/kosmastsk/path_planning/blob/master/src/path_planning.cpp
					bounds.setLow(0, _min_bounds[0]);
					bounds.setHigh(0, _max_bounds[0]);
					bounds.setLow(1,  _min_bounds[1]);
					bounds.setHigh(1,  _max_bounds[1]);
					bounds.setLow(2, 0);
					bounds.setHigh(2, 3);

					space->as<ob::SE3StateSpace>()->setBounds(bounds);

					// construct an instance of  space information from this state space
					si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
					start->setXYZ(d_local_position.pose.position.x,
						d_local_position.pose.position.y,
						d_local_position.pose.position.z);
					start->as<ob::SO3StateSpace::StateType>(1)->setIdentity(); // start.random();
					goal->setXYZ(endpoint_pos_ENU.pose.position.x,
						endpoint_pos_ENU.pose.position.y,
						endpoint_pos_ENU.pose.position.z);
					goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
					// goal.random();

					// set state validity checking for this space
					std::shared_ptr<OctomapStateValidator> validity_checker(new OctomapStateValidator(si, this->octomap_msgs));
					si->setStateValidityChecker(validity_checker);

					// create a problem instance
					pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
					// set the start and goal states
					pdef->setStartAndGoalStates(start, goal);
					// set Optimizattion objective
					ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
					obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
					pdef->setOptimizationObjective(obj);

					ob::PlannerPtr plan(new og::RRTstar(si));
					// set the problem we are trying to solve for the planner
					plan->setProblemDefinition(pdef);
					// perform setup steps for the planner
					plan->setup();
					// attempt to solve the problem within one second of planning time
					ob::PlannerStatus solved = plan->solve(2);

					if (solved)
					{
						int marr_index = 0;

						// Path smoothing using bspline
						// ompl::geometric::PathSimplifier *pathBSpline = new ompl::geometric::PathSimplifier(si);
						// ompl::geometric::PathGeometric *_path_smooth = new ompl::geometric::PathGeometric(
						// 	dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));

						// ROS_WARN("Path smoothness : %f\n", _path_smooth->smoothness());
						// // Using 5, as is the default value of the function
						// // If the path is not smooth, the value of smoothness() will be closer to 1
						// int bspline_steps = ceil(3 * _path_smooth->smoothness());

						// pathBSpline->smoothBSpline(*_path_smooth, bspline_steps);
						// ROS_INFO("Smoothed Path\n");
						// _path_smooth->print(std::cout);

						mav_trajectory_generation::NonlinearOptimizationParameters parameters;
						mav_trajectory_generation ::Vertex::Vector vertices;
						const int dimension = 3;
						const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
						// we have 2 vertices:
						// Start = current position
						// end = desired position and velocity
						mav_trajectory_generation::Vertex start(dimension), end(dimension);

						og::PathGeometric *path = pdef->getSolutionPath()->as<og::PathGeometric>();
						for (std::size_t path_idx = 0; path_idx < path->getStateCount(); path_idx++)
						{
							const ob::SE3StateSpace::StateType *se3state = path->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

							// extract the first component of the state and cast it to what we expect
							const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

							// extract the second component of the state and cast it to what we expect
							const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

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
							this->d_way_points.markers.push_back(mk);

							if (path_idx == 0)
							{
								start.makeStartOrEnd(Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2]), derivative_to_optimize);
								// set start point's velocity to be constrained to current velocity
								start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
								vertices.push_back(start);
							}
							else if (path_idx == path->getStateCount() - 1)
							{
								end.makeStartOrEnd(Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2]), derivative_to_optimize);
								// set start point's velocity to be constrained to current velocity
								end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0, 0, 0));
								vertices.push_back(end);
							}
							else
							{
								mav_trajectory_generation::Vertex vertex(dimension);
								vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2]));
								vertices.push_back(vertex);
							}
						}

						// setimate initial segment times
						std::vector<double> segment_times;
						segment_times = estimateSegmentTimes(vertices, 3.0, 1.0);
						// set up optimization problem
						const int N = 10;
						mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
						opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

						// constrain velocity and acceleration
						opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 1.0);
						opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, 1.0);

						// solve trajectory
						opt.optimize();
						opt.getTrajectory(&trajectory);

						// use it for controller.
						// ref : https://github.com/ethz-asl/mav_comm/blob/master/mav_msgs/include/mav_msgs/eigen_mav_msgs.h
						// in EigenTrajectoryPoint struct
						// TODO : having topic for this structure ~ Assign Phu HUYNH.

						// Whole trajectory:
						double sampling_interval = 0.05;
						bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

						double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
						std::string frame_id = "map";

						// From Trajectory class:
						mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &global_trajectory);
						ros_client->way_points_pub.publish(d_way_points);
						ros_client->global_traj_pub.publish(global_trajectory);
						pre_time = ros::Time::now();
						this->planning_type = PLANNING_STEP::FOLLOW_TRAJECTORY;
						break;
					}
					else
					{
						ROS_INFO("FAILED TO FIND PATH WITH OMPL-RRT");
					}
					break;
				}
				case PLANNING_STEP::ASTAR_PLANNING:
				{
					this->grid = new Grid3D(100,100,10,1);
					octomap::point3d current_pos(d_local_position.pose.position.x,
								d_local_position.pose.position.y,
								d_local_position.pose.position.z);	
					grid->Initilize(current_pos);
					grid->readOctomapMsg(this->octomap_msgs);

					astar = new Astar(
					octomap::point3d(endpoint_pos_ENU.pose.position.x,
						endpoint_pos_ENU.pose.position.y,
						endpoint_pos_ENU.pose.position.z),
					this->grid);

					std::vector<int> node_index;
					bool solved = astar->find_path(current_pos, node_index,100000);

					if(solved){
						mav_trajectory_generation::NonlinearOptimizationParameters parameters;
						mav_trajectory_generation ::Vertex::Vector vertices;
						const int dimension = 3;
						const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
						// we have 2 vertices:
						// Start = current position
						// end = desired position and velocity
						mav_trajectory_generation::Vertex start(dimension), end(dimension);

						int marr_index = 0;
						for(std::vector<int>::iterator it = node_index.begin(); it != node_index.end(); ++it){

							octomap::point3d pos = grid->toPosition(*it);

							visualization_msgs::Marker mk;
							mk.id = marr_index;
							mk.type = mk.CUBE;
							marr_index += 1;
							mk.header.frame_id = "map";
							mk.pose.position.x = pos.x();
							mk.pose.position.y = pos.y();
							mk.pose.position.z = pos.z();
							mk.color.r = 1.0;
							mk.color.a = 1.0;
							mk.scale.x = 0.2;
							mk.scale.y = 0.2;
							mk.scale.z = 0.2;
							this->d_way_points.markers.push_back(mk);

							if(it == node_index.begin()){
								start.makeStartOrEnd(Eigen::Vector3d(pos.x(), pos.y(), pos.z()), derivative_to_optimize);
								// set start point's velocity to be constrained to current velocity
								start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
								vertices.push_back(start);
							}
							else if(std::next(it) == node_index.end()){
								end.makeStartOrEnd(Eigen::Vector3d(pos.x(), pos.y(), pos.z()), derivative_to_optimize);
								// set start point's velocity to be constrained to current velocity
								end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0, 0, 0));
								vertices.push_back(end);
							}
							else{
								mav_trajectory_generation::Vertex vertex(dimension);
								vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(pos.x(), pos.y(), pos.z()));
								vertices.push_back(vertex);
							}
						}
						// setimate initial segment times
						std::vector<double> segment_times;
						segment_times = estimateSegmentTimes(vertices, 3.0, 1.0);
						// set up optimization problem
						const int N = 10;
						mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
						opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

						// constrain velocity and acceleration
						opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 1.0);
						opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, 1.0);

						// solve trajectory
						opt.optimize();
						opt.getTrajectory(&trajectory);

						// use it for controller.
						// ref : https://github.com/ethz-asl/mav_comm/blob/master/mav_msgs/include/mav_msgs/eigen_mav_msgs.h
						// in EigenTrajectoryPoint struct
						// TODO : having topic for this structure ~ Assign Phu HUYNH.

						// Whole trajectory:
						double sampling_interval = 0.05;
						bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

						double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
						std::string frame_id = "map";

						// From Trajectory class:
						mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &global_trajectory);
						ros_client->way_points_pub.publish(d_way_points);
						ros_client->global_traj_pub.publish(global_trajectory);
						pre_time = ros::Time::now();
						this->planning_type = PLANNING_STEP::FOLLOW_TRAJECTORY;
						break;				
					}
					else{
						ROS_INFO("FAILED TO FIND PATH WITH ASTAR");
					}
					break;
				}
				case PLANNING_STEP::FOLLOW_TRAJECTORY:
				{
					double dt = ros::Time::now().toSec() - pre_time.toSec();
					int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
					if (dt > trajectory.getMaxTime())
					{
						endpoint_active = false;
						ROS_INFO("Finished.");
						this->planning_type = PLANNING_STEP::IDLE;
						break;
					}

					Eigen::VectorXd position_d = trajectory.evaluate(dt, mav_trajectory_generation::derivative_order::POSITION);
					Eigen::VectorXd vel_d = trajectory.evaluate(dt, mav_trajectory_generation::derivative_order::VELOCITY);
					Eigen::VectorXd acce_d = trajectory.evaluate(dt, mav_trajectory_generation::derivative_order::ACCELERATION);

					//PositionTarget Message Format.

					setpoint_raw.header.stamp = ros::Time::now();
					setpoint_raw.header.frame_id = "map";
					setpoint_raw.type_mask = 0;
					setpoint_raw.position.x = position_d(0);
					setpoint_raw.position.y = position_d(1);
					setpoint_raw.position.z = position_d(2);
					setpoint_raw.velocity.x = vel_d(0);
					setpoint_raw.velocity.y = vel_d(1);
					setpoint_raw.velocity.z = vel_d(2);
					setpoint_raw.acceleration_or_force.x = acce_d(0);
					setpoint_raw.acceleration_or_force.y = acce_d(1);
					setpoint_raw.acceleration_or_force.z = acce_d(2);

					// publishVisualize();
					ros_client->publish_raw_position_target(setpoint_raw);
					break;
				}
				case PLANNING_STEP::IDLE:
				{
					ROS_INFO("Target reach. Waiting for command");
					break;
				}
			}
		}
		ros::spinOnce();
		rate_->sleep();
	}
}

void DPlanning::state_callback(const mavros_msgs::State::ConstPtr &msg)
{
	d_current_state = *msg;
}

void DPlanning::local_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	d_local_position = *msg;
	if (endpoint_active)
	{
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

void DPlanning::get_target_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	if (!endpoint_active)
	{
		endpoint_pos_ENU = *msg;
		ROS_INFO("Requested trajectory : \n start (x,y,z) : %f %f %f \n stop (x,y,z): %f %f %f",
			d_local_position.pose.position.x, d_local_position.pose.position.y, d_local_position.pose.position.z,
			endpoint_pos_ENU.pose.position.x, endpoint_pos_ENU.pose.position.y, endpoint_pos_ENU.pose.position.z);

		start_time = ros::Time::now();
		pre_time = ros::Time::now();
		startpoint_pos_ENU = d_local_position;
		d_previous_position = d_local_position;
		travel_cost = 0.0;
		// astar = new Astar(
		// 	octomap::point3d(endpoint_pos_ENU.pose.position.x,
		// 		endpoint_pos_ENU.pose.position.y,
		// 		endpoint_pos_ENU.pose.position.z),
		// 	this->grid);
		// apf = new APF(this->grid);
		// publishVisualize();

		endpoint_active = true;
		best_duration = 1000000000000;
		worst_duration = 0;
	}
}

void DPlanning::bin_octomap_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	if (!octomap_activate)
	{
		octomap_activate = true;
	}
	this->octomap_cloud = *msg;
}

void DPlanning::full_octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg)
{
	this->octomap_msgs = msg;

	// ROS_INFO("OCTOMAP CALLBACK 1");
	// // convert ColorOcTree to OcTree
	// octomap::OcTree* tree_oct = reinterpret_cast<octomap::OcTree*>(tree_coloct);
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	
	tree_oct->getMetricMin(_min_bounds[0], _min_bounds[1], _min_bounds[2]);
	tree_oct->getMetricMax(_max_bounds[0], _max_bounds[1], _max_bounds[2]);
}



//local map callback
//TODO : Khang VO
void DPlanning::occ_trigger_callback(const std_msgs::Bool::ConstPtr &msg){

}
void DPlanning::apf_force_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){

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
