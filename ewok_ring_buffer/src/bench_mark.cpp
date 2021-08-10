#include <ewok/ed_ring_buffer.h>
#include <ewok/APF.h>
#include <ewok/Astar.h>
#include <ewok/RingStateValidator.h>
#include <ros/ros.h>
#include <math.h>       /* isnan, sqrt */
#include <chrono>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>



#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>


#include <std_msgs/String.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <fstream>
#include <chrono>

#include <visualization_msgs/MarkerArray.h>

using namespace message_filters;

// global declaration
ros::Time _last_time;

bool initialized = false;
const double resolution = 0.4;
static const int POW = 4;
static const int N = (1 << POW);
ewok::EuclideanDistanceRingBuffer<POW> rrb(resolution, 5.0);

octomap::OcTree tree(resolution);

std::ofstream f_time;


ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, norm_marker_pub;
ros::Publisher cloud2_pub, center_pub;

double map_rate, pub_rate;
std::string m_worldFrameId = "/map";

ros::Publisher occ_trigger_pub, apf_grad_pub, local_waypoint_pub, global_trigger_pub; // can has more.

ros::Publisher chatter_pub;

ros::Publisher local_waypoints_pub1;
ros::Publisher local_waypoints_pub2;
ros::Publisher local_waypoints_pub3;
ros::Publisher local_waypoints_pub4;

ros::Publisher local_waypoints_pub11;
ros::Publisher local_waypoints_pub22;
ros::Publisher local_waypoints_pub33;
ros::Publisher local_waypoints_pub44;

visualization_msgs::MarkerArray local_waypoints_markerarray1;
visualization_msgs::MarkerArray local_waypoints_markerarray2;
visualization_msgs::MarkerArray local_waypoints_markerarray3;
visualization_msgs::MarkerArray local_waypoints_markerarray4;

geometry_msgs::PoseArray local_waypoints_markerarray11;
geometry_msgs::PoseArray local_waypoints_markerarray22;
geometry_msgs::PoseArray local_waypoints_markerarray33;
geometry_msgs::PoseArray local_waypoints_markerarray44;

Eigen::Vector3f global_origin;
Eigen::Vector3f local_target;

geometry_msgs::PoseArray trajectory_subset;

bool apf_active = false;
bool local_astar_active = false;
bool local_rrt_active = false;

ewok::APF AP_field;
ewok::Grid3D Grid(100,100,8,1.0);

void odomCloudCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    double elp = ros::Time::now().toSec() - _last_time.toSec();
    // if(elp < (1 / map_rate)) return;
    
    tf2::Quaternion q_orig, q_rot, q_new;
    // Get the original orientation of 'commanded_pose'
    tf2::convert(odom->pose.pose.orientation , q_orig);

    // for simulation iris rotate
    q_rot.setRPY(-1.5, 0, -1.57);

    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();

    Eigen::Quaternionf q;
    q.w() = q_new.getW();
    q.x() = q_new.getX();
    q.y() = q_new.getY();
    q.z() = q_new.getZ();

    // create transform matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block(0, 0, 3, 3) = Eigen::Matrix3f(q);
    transform(0, 3) = odom->pose.pose.position.x;
    transform(1, 3) = odom->pose.pose.position.y;
    transform(2, 3) = odom->pose.pose.position.z;
    
    // convert cloud to pcl form
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *cloud_in);
    // transform to world frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

    // compute ewol pointcloud and origin
    Eigen::Vector3f origin = (transform * Eigen::Vector4f(0, 0, 0, 1)).head<3>();
    global_origin = origin;
    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud_ew;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points = cloud_out->points;

    //for benchmark octomap.
    octomap::Pointcloud octomap_cloud;
    // printf("point cloud size : %d", points.size());
    for(int i = 0; i < points.size(); ++i)
    {
        if (isnan(points.at(i).x) || isnan(points.at(i).y) || isnan(points.at(i).z)){
            continue;
        }
        // printf("(%f, %f, %f)",points.at(i).x, points.at(i).y, points.at(i).z);  
        cloud_ew.push_back(Eigen::Vector4f(points.at(i).x, points.at(i).y, points.at(i).z, 0));                
        octomap_cloud.push_back(points.at(i).x, points.at(i).y, points.at(i).z);

    }

    // initialize the ringbuffer map
    if(!initialized)
    {
        Eigen::Vector3i idx;
        rrb.getIdx(origin, idx);
        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());
        rrb.setOffset(idx);
        initialized = true;
    }
    else
    {
        Eigen::Vector3i origin_idx, offset, diff;
        rrb.getIdx(origin, origin_idx);
        offset = rrb.getVolumeCenter();
        diff = origin_idx - offset;
        if(diff.array().any()) rrb.moveVolume(diff.head<3>());
    }


    octomap::point3d sensor_origin(origin(0), origin(1), origin(2));

    auto t1 = std::chrono::high_resolution_clock::now();

    rrb.insertPointCloud(cloud_ew, origin);

    auto t2 = std::chrono::high_resolution_clock::now();

    rrb.updateDistance();

    auto t3 = std::chrono::high_resolution_clock::now();

    tree.insertPointCloud(octomap_cloud, sensor_origin);

    auto t4 = std::chrono::high_resolution_clock::now();

     std_msgs::String msg;
    std::stringstream ss;
    ss <<  std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()  << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count();
    msg.data = ss.str();
    chatter_pub.publish(msg);




    // visualize ringbuffer
    visualization_msgs::Marker m_occ, m_free, m_dist, m_norm;
    rrb.getMarkerOccupied(m_occ);
    rrb.getMarkerFree(m_free);
    rrb.getMarkerDistance(m_dist, 0.5);
    // rrb.getMarkerNormal(m_norm);

    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);
    dist_marker_pub.publish(m_dist);
    // norm_marker_pub.publish(m_norm);

    _last_time = ros::Time::now();
}

void trajectoryCallback(const geometry_msgs::PoseArrayConstPtr& pose_array){
    trajectory_subset = *pose_array;
}

void endpointCallback(const geometry_msgs::PoseStampedConstPtr& endpoint){
    local_target(0) = endpoint->pose.position.x;
    local_target(1) = endpoint->pose.position.y;
    local_target(2) = endpoint->pose.position.z;
}

void activeAstarCallback(const std_msgs::BoolConstPtr& msg){
    if(msg->data){
        local_astar_active = true;
    }
    else{
        local_astar_active = false;
    }
}

void activeAPFCallback(const std_msgs::BoolConstPtr& msg){
    if(msg->data){
        apf_active = true;
    }
    else{
        apf_active = false;
    }
}

void activeRRTCallback(const std_msgs::BoolConstPtr& msg){
    if(msg->data){
        local_rrt_active = true;
    }
    else{
        local_rrt_active = false;
    }
}

void timerCallback(const ros::TimerEvent& e)
{
    if(!initialized) return;


    if(apf_active){
        Eigen::Vector3f grad;
        float distance = rrb.getDistanceWithGrad(global_origin, grad);
        Eigen::Vector3f vel;

        AP_field.calculate(global_origin, local_target, distance, grad, vel);
        geometry_msgs::PoseStamped vel_;
        vel_.pose.position.x = vel(0);
        vel_.pose.position.y = vel(1);
        vel_.pose.position.z = vel(2);    

        apf_grad_pub.publish(vel_);
        return;
    }

    if(local_astar_active){
        Grid.Initilize(global_origin, &rrb);
        ewok::Astar astar(local_target, &Grid);
        std::vector<Eigen::Vector3f> path;
        std::vector<Eigen::Vector3f> node_index;
        ROS_INFO("LOCAL_ASTAR_ACTIVE");
        bool solved = astar.find_path(global_origin,path, node_index, 500);
        if(solved){
            geometry_msgs::PoseArray local_waypoints;
            for(std::vector<Eigen::Vector3f>::iterator it = node_index.begin(); it != node_index.end(); ++it){
                geometry_msgs::Pose pose;
                pose.position.x = (*it)(0);
                pose.position.y = (*it)(1);
                pose.position.z = (*it)(2);

                local_waypoints.poses.push_back(pose);
            }
            ROS_INFO("LOCAL PATH SIZE: %d", local_waypoints.poses.size());
            local_waypoint_pub.publish(local_waypoints);
        }
        else{
            ROS_INFO("ASTAR FAIL TO FIND PATH IN LOCAL MAP");
            std_msgs::Bool global_trigger;
            global_trigger.data = true;
            global_trigger_pub.publish(global_trigger);
        }
        trajectory_subset.poses.clear();        
        local_astar_active = false;        
        return;
    }

    if(local_rrt_active){
        Grid.Initilize(global_origin, &rrb);
        ewok::Astar astar(local_target, &Grid);
        std::vector<Eigen::Vector3f> path;
        std::vector<Eigen::Vector3f> node_index;
        ROS_INFO("LOCAL_ASTAR_ACTIVE");
        bool solved = astar.find_path(global_origin,path, node_index, 500);

        if(solved){
                        ROS_INFO("BREAK POINT");
                        mav_trajectory_generation::Trajectory global_trajectory1;
						mav_trajectory_generation::NonlinearOptimizationParameters parameters;
						mav_trajectory_generation ::Vertex::Vector vertices;
						const int dimension = 3;
						const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
						// we have 2 vertices:
						// Start = current position
						// end = desired position and velocity
						mav_trajectory_generation::Vertex start(dimension), end(dimension);

						int marr_index = 0;
                        ROS_INFO("BREAK POINT BEFORE LOOP");
						local_waypoints_markerarray1.markers.clear();
						for(std::vector<Eigen::Vector3f>::iterator it = path.begin(); it != path.end(); ++it){
							visualization_msgs::Marker mk;
                            Eigen::Vector3f pos((*it)(0), (*it)(1), (*it)(2));
							mk.id = marr_index;
							mk.type = mk.SPHERE;
							marr_index += 1;
							mk.header.frame_id = "map";
							mk.pose.position.x = (*it)(0);
							mk.pose.position.y = (*it)(1);
							mk.pose.position.z = (*it)(2);
							mk.color.r = 0.5;
							mk.color.g = 0.0;
							mk.color.b = 1.0;
							mk.color.a = 1.0;
							mk.scale.x = 0.4;
							mk.scale.y = 0.4;
							mk.scale.z = 0.4;
							local_waypoints_markerarray1.markers.push_back(mk);

                            geometry_msgs::Pose pose;
                            pose.position.x = (*it)(0);
                            pose.position.y = (*it)(1);
                            pose.position.z = (*it)(2);
                            local_waypoints_markerarray11.poses.push_back(pose);

						}
                        local_waypoints_pub11.publish(local_waypoints_markerarray11);
					
        }
        else{
            ROS_INFO("ASTAR FAIL TO FIND PATH IN LOCAL MAP");
            std_msgs::Bool global_trigger;
            global_trigger.data = true;
            global_trigger_pub.publish(global_trigger);
        } 

        if(solved){
            mav_trajectory_generation::Trajectory global_trajectory2;
						mav_trajectory_generation::NonlinearOptimizationParameters parameters;
						mav_trajectory_generation ::Vertex::Vector vertices;
						const int dimension = 3;
						const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
						// we have 2 vertices:
						// Start = current position
						// end = desired position and velocity
						mav_trajectory_generation::Vertex start(dimension), end(dimension);

						int marr_index = 0;
						local_waypoints_markerarray2.markers.clear();
						for(std::vector<Eigen::Vector3f>::iterator it = node_index.begin(); it != node_index.end(); ++it){

							visualization_msgs::Marker mk;
							mk.id = marr_index;
							mk.type = mk.SPHERE;
							marr_index += 1;
							mk.header.frame_id = "map";
							mk.pose.position.x = (*it)(0);
							mk.pose.position.y = (*it)(1);
							mk.pose.position.z = (*it)(2);
							mk.color.r = 0.5;
							mk.color.g = 0.0;
							mk.color.b = 1.0;
							mk.color.a = 1.0;
							mk.scale.x = 0.4;
							mk.scale.y = 0.4;
							mk.scale.z = 0.4;
							local_waypoints_markerarray2.markers.push_back(mk);

                            geometry_msgs::Pose pose;
                            pose.position.x = (*it)(0);
                            pose.position.y = (*it)(1);
                            pose.position.z = (*it)(2);
                            local_waypoints_markerarray22.poses.push_back(pose);

						}
                        local_waypoints_pub22.publish(local_waypoints_markerarray22);
        }
        else{
            ROS_INFO("ASTAR FAIL TO FIND PATH IN LOCAL MAP");
            std_msgs::Bool global_trigger;
            global_trigger.data = true;
            global_trigger_pub.publish(global_trigger);
        }      

        ompl::base::StateSpacePtr space;
		ompl::base::SpaceInformationPtr si;
	
		space = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());

				    // create a start state
					ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space);
					// create a goal state
					ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(space);
					// set the bounds for the R^3 part of SE(3)
					ompl::base::RealVectorBounds bounds(3);

					//TODO : Assign Khang VO.
					// config pipeline, ref : https://github.com/kosmastsk/path_planning/blob/master/src/path_planning.cpp
					// bounds.setLow(0, _min_bounds[0]);
					// bounds.setHigh(0, _max_bounds[0]);
					// bounds.setLow(1,  _min_bounds[1]);
					// bounds.setHigh(1,  _max_bounds[1]);
					// bounds.setLow(2, 1.5);
					// bounds.setHigh(2, 2.5);

					bounds.setLow(0, -40);
					bounds.setHigh(0, 40);
					bounds.setLow(1,  -5);
					bounds.setHigh(1,  5);
					bounds.setLow(2, 1.0);
					bounds.setHigh(2, 2.0);

					space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

					// construct an instance of  space information from this state space
					si = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(space));
					start->setXYZ(global_origin(0),
						global_origin(1),
						global_origin(2));
					start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity(); // start.random();
					goal->setXYZ(local_target(0),
						local_target(1),
						local_target(2));
					goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
					// goal.random();

					// set state validity checking for this space
					std::shared_ptr<ewok::RingStateValidator> validity_checker(new ewok::RingStateValidator(si, &rrb));
					si->setStateValidityChecker(validity_checker);

					// create a problem instance
					ompl::base::ProblemDefinitionPtr pdef1 = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si));
					// set the start and goal states
					pdef1->setStartAndGoalStates(start, goal);
					// set Optimizattion objective
					ompl::base::OptimizationObjectivePtr obj1(new ompl::base::PathLengthOptimizationObjective(si));
					obj1->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
					pdef1->setOptimizationObjective(obj1);

					// create a problem instance
					ompl::base::ProblemDefinitionPtr pdef2 = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si));
					// set the start and goal states
					pdef2->setStartAndGoalStates(start, goal);
					// set Optimizattion objective
					ompl::base::OptimizationObjectivePtr obj2(new ompl::base::PathLengthOptimizationObjective(si));
					obj2->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
					pdef2->setOptimizationObjective(obj2);

					ompl::base::PlannerPtr plan1(new ompl::geometric::RRTstar(si));
					// set the problem we are trying to solve for the planner
					plan1->setProblemDefinition(pdef1);
					// perform setup steps for the planner
					plan1->setup();
					// attempt to solve the problem within one second of planning time

					auto time_start1 = std::chrono::high_resolution_clock::now(); 
					ompl::base::PlannerStatus solved1 = plan1->solve(0.01);
					auto time_end1 = std::chrono::high_resolution_clock::now();
      				auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(time_end1 - time_start1);
					ROS_INFO("TIME FOR RRT SOLVE: %d", duration1);

					ompl::base::PlannerPtr plan2(new ompl::geometric::InformedRRTstar(si));
					// set the problem we are trying to solve for the planner
					plan2->setProblemDefinition(pdef2);
					// perform setup steps for the planner
					plan2->setup();
					// attempt to solve the problem within one second of planning time
					auto time_start2 = std::chrono::high_resolution_clock::now(); 
					ompl::base::PlannerStatus solved2 = plan2->solve(0.01);
					auto time_end2 = std::chrono::high_resolution_clock::now();
      				auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(time_end2 - time_start2);
					ROS_INFO("TIME FOR INFORMED RRT SOLVE: %d", duration2);

                    if (solved1)
					{
						int marr_index = 0;
						local_waypoints_markerarray3.markers.clear();	
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
						mav_trajectory_generation::Trajectory global_trajectory3;
						mav_trajectory_generation::NonlinearOptimizationParameters parameters;
						mav_trajectory_generation ::Vertex::Vector vertices;
						const int dimension = 3;
						const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
						// we have 2 vertices:
						// Start = current position
						// end = desired position and velocity
						mav_trajectory_generation::Vertex start(dimension), end(dimension);

						ompl::geometric::PathGeometric *path = pdef1->getSolutionPath()->as<ompl::geometric::PathGeometric>();
						ROS_INFO("RRT STAR SOLUTION STATE: %d", path->getStateCount());
						for (std::size_t path_idx = 0; path_idx < path->getStateCount(); path_idx++)
						{
							const ompl::base::SE3StateSpace::StateType *se3state = path->getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();

							// extract the first component of the state and cast it to what we expect
							const ompl::base::RealVectorStateSpace::StateType *pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

							// extract the second component of the state and cast it to what we expect
							const ompl::base::SO3StateSpace::StateType *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

							visualization_msgs::Marker mk;
							mk.id = marr_index;
							mk.type = mk.SPHERE;
							marr_index += 1;
							mk.header.frame_id = "map";
							mk.pose.position.x = pos->values[0];
							mk.pose.position.y = pos->values[1];
							mk.pose.position.z = pos->values[2];
							mk.color.r = 0.5;
							mk.color.g = 0.0;
							mk.color.b = 1.0;
							mk.color.a = 1.0;
							mk.scale.x = 0.4;
							mk.scale.y = 0.4;
							mk.scale.z = 0.4;
							local_waypoints_markerarray3.markers.push_back(mk);

                            geometry_msgs::Pose pose;
                            pose.position.x = pos->values[0];
                            pose.position.y = pos->values[1];
                            pose.position.z = pos->values[2];
                            local_waypoints_markerarray33.poses.push_back(pose);

						}
                        local_waypoints_pub33.publish(local_waypoints_markerarray33);

					}
					else
					{
                        std_msgs::Bool global_trigger;
                        global_trigger.data = true;
                        global_trigger_pub.publish(global_trigger);
						ROS_INFO("FAILED TO FIND LOCAL PATH WITH OMPL-RRT");
					}

                    if (solved2)
					{
						int marr_index = 0;
						local_waypoints_markerarray4.markers.clear();	
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
						mav_trajectory_generation::Trajectory global_trajectory4;
						mav_trajectory_generation::NonlinearOptimizationParameters parameters;
						mav_trajectory_generation ::Vertex::Vector vertices;
						const int dimension = 3;
						const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
						// we have 2 vertices:
						// Start = current position
						// end = desired position and velocity
						mav_trajectory_generation::Vertex start(dimension), end(dimension);

						ompl::geometric::PathGeometric *path = pdef2->getSolutionPath()->as<ompl::geometric::PathGeometric>();
						ROS_INFO("INFORMED RRT STAR SOLUTION STATE: %d", path->getStateCount());
						for (std::size_t path_idx = 0; path_idx < path->getStateCount(); path_idx++)
						{
							const ompl::base::SE3StateSpace::StateType *se3state = path->getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();

							// extract the first component of the state and cast it to what we expect
							const ompl::base::RealVectorStateSpace::StateType *pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

							// extract the second component of the state and cast it to what we expect
							const ompl::base::SO3StateSpace::StateType *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

							visualization_msgs::Marker mk;
							mk.id = marr_index;
							mk.type = mk.SPHERE;
							marr_index += 1;
							mk.header.frame_id = "map";
							mk.pose.position.x = pos->values[0];
							mk.pose.position.y = pos->values[1];
							mk.pose.position.z = pos->values[2];
							mk.color.r = 0.5;
							mk.color.g = 0.0;
							mk.color.b = 1.0;
							mk.color.a = 1.0;
							mk.scale.x = 0.4;
							mk.scale.y = 0.4;
							mk.scale.z = 0.4;
							local_waypoints_markerarray4.markers.push_back(mk);

                            geometry_msgs::Pose pose;
                            pose.position.x = pos->values[0];
                            pose.position.y = pos->values[1];
                            pose.position.z = pos->values[2];
                            local_waypoints_markerarray44.poses.push_back(pose);

						}
                        local_waypoints_pub44.publish(local_waypoints_markerarray44);
					}
					else
					{
                        std_msgs::Bool global_trigger;
                        global_trigger.data = true;
                        global_trigger_pub.publish(global_trigger);
						ROS_INFO("FAILED TO FIND LOCAL PATH WITH OMPL-RRT");
					}
        trajectory_subset.poses.clear();        
        local_rrt_active = false;        
    }

    local_waypoints_pub1.publish(local_waypoints_markerarray1);
    local_waypoints_pub2.publish(local_waypoints_markerarray2);
    local_waypoints_pub3.publish(local_waypoints_markerarray3);
    local_waypoints_pub4.publish(local_waypoints_markerarray4);

    for(std::vector<geometry_msgs::Pose>::iterator it = trajectory_subset.poses.begin(); it !=  trajectory_subset.poses.end(); ++it){
        Eigen::Vector3f grad;
        Eigen::Vector3f check_point(it->position.x, it->position.y, it->position.z);
        float distance = rrb.getDistanceWithGrad(check_point, grad);
        if(distance < 0.3){
            std_msgs::Bool local_collision;
            local_collision.data = true;                
            occ_trigger_pub.publish(local_collision);
            ROS_INFO("FUTURE COLLISION");
            break;
        }
        
    }

    trajectory_subset.poses.clear();

    // if(!apf_active){


    // }
    // else{
    //     float distance = rrb.getDistanceWithGrad(global_origin, grad);
    //     Eigen::Vector3f vel;

    //     AP_field.calculate(global_origin, global_target, distance, grad, vel);
    //     geometry_msgs::PoseStamped vel_;
    //     vel_.pose.position.x = vel(0);
    //     vel_.pose.position.y = vel(1);
    //     vel_.pose.position.z = vel(2);    

    //     apf_grad_pub.publish(vel_);


    // }



    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // Eigen::Vector3d center;
    // // rrb.getBufferAsCloud(cloud, center);

    // // convert to ROS message and publish
    // sensor_msgs::PointCloud2 cloud2;
    // pcl::toROSMsg(cloud, cloud2);

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = center(0);
    // pose.pose.position.y = center(1);
    // pose.pose.position.z = center(2);

    // // message publish should have the same time stamp
    // ros::Time stamp = ros::Time::now();
    // pose.header.stamp = stamp;
    // pose.header.frame_id = "map";
    // cloud2.header.stamp = stamp;
    // cloud2.header.frame_id = "map";

    // cloud2_pub.publish(cloud2);
    // center_pub.publish(pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realtime_example");
    ros::NodeHandle nh;

    chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    //for waypoint visualization
    local_waypoints_pub1 = nh.advertise<visualization_msgs::MarkerArray>("/mapping/local_waypoints1", 10);
	local_waypoints_pub2 = nh.advertise<visualization_msgs::MarkerArray>("/mapping/local_waypoints2", 10);
	local_waypoints_pub3 = nh.advertise<visualization_msgs::MarkerArray>("/mapping/local_waypoints3", 10);
	local_waypoints_pub4 = nh.advertise<visualization_msgs::MarkerArray>("/mapping/local_waypoints4", 10);

    local_waypoints_pub11 = nh.advertise<geometry_msgs::PoseArray>("/mapping/local_waypoints11", 10);
    local_waypoints_pub22 = nh.advertise<geometry_msgs::PoseArray>("/mapping/local_waypoints22", 10);
    local_waypoints_pub33 = nh.advertise<geometry_msgs::PoseArray>("/mapping/local_waypoints33", 10);
    local_waypoints_pub44 = nh.advertise<geometry_msgs::PoseArray>("/mapping/local_waypoints44", 10);

    //Local Mapping Controller 
    occ_trigger_pub = nh.advertise<std_msgs::Bool>("/mapping/has_occupied", 1, true);
    apf_grad_pub = nh.advertise<geometry_msgs::PoseStamped>("/mapping/distance_grad", 1, true);
    local_waypoint_pub = nh.advertise<geometry_msgs::PoseArray>("/mapping/local_waypoints",1,true);
    global_trigger_pub = nh.advertise<std_msgs::Bool>("/mapping/global_trigger",1,true);

    // ringbuffer visualizer
    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("/local_map_visualizer/occupied", 5, true);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5, true);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5, true);
    norm_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/normal", 5, true);
    cloud2_pub = nh.advertise<sensor_msgs::PointCloud2>("ring_buffer/cloud2", 1, true);
    center_pub = nh.advertise<geometry_msgs::PoseStamped>("ring_buffer/center", 1, true);
    
    // Subcriber for trajectory future
    ros::Subscriber future_traj_sub = nh.subscribe<geometry_msgs::PoseArray>("/planning/traj_subset", 5, trajectoryCallback);
    // Subcriber for endpoint
    ros::Subscriber getpoint_target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/planning/local_target", 10, endpointCallback);
    ros::Subscriber local_astar_active_sub = nh.subscribe<std_msgs::Bool>("/planning/local_astar_active", 10, activeAstarCallback);
    ros::Subscriber local_apf_active_sub = nh.subscribe<std_msgs::Bool>("/planning/local_apf_active", 10, activeAPFCallback);
    ros::Subscriber local_rrt_active_sub = nh.subscribe<std_msgs::Bool>("/planning/local_rrt_active", 10, activeRRTCallback);
    

    // synchronized subscriber for pointcloud and odometry
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/mavros/local_position/odom", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/camera/depth/color/points", 1);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pcl_sub);
    sync.registerCallback(boost::bind(&odomCloudCallback, _1, _2));

    // get parameter
    nh.getParam("/ring_buffer/map_rate", map_rate);
    nh.getParam("/ring_buffer/publish_rate", pub_rate);
    std::cout << map_rate << "," << pub_rate << std::endl;

    // timer for publish ringbuffer as pointcloud
    ros::Timer timer = nh.createTimer(ros::Duration(1 / pub_rate), timerCallback);

    ros::Duration(0.12).sleep();
    _last_time = ros::Time::now();
    std::cout << "Start mapping!" << std::endl;

    ros::spin();

    return 0;
}