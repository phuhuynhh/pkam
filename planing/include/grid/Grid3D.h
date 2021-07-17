#ifndef GRID3D_H
#define GRID3D_H

#include <octomap/octomap.h>
#include <bits/stdc++.h>
#include <vector>
#include <set>
#include <unordered_map>
#include <random>

#include "grid/Node.h"
#include "grid/Edge.h"


#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/conversions.h>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"


  class Node;
  class Edge;

  class Grid3D{
  public:
    int infinity = 10e6;

    octomap::point3d origin;
    float resolution;
    octomap::point3d corner;
    int sizeX, sizeY, sizeZ;

    std::vector<int> index_ray; // temporate storage for ray casting
    std::vector<int> neighborIdx; //temporate storage for neighbor finding

    // std::vector<Node> node_list;
    // std::vector<std::unordered_map<int, Edge>> adjency_list;



    std::default_random_engine generator;
    std::uniform_int_distribution<int> random_idx;

    octomap::OcTree* colorTree;
    std::shared_ptr<fcl::CollisionGeometry> tree_obj;
    std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

    Grid3D(
      int x,
      int y,
      int z,
      const float& resolution
    ){
      this->resolution = resolution;

      this->sizeX = x;
      this->sizeY = y;
      this->sizeZ = z;

      random_idx = std::uniform_int_distribution<int>(0, x*y*z-1);
      Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(resolution, resolution, resolution));
      
    }

    void Initilize(const octomap::point3d& origin);

    void readOctomapMsg(const octomap_msgs::Octomap::ConstPtr& octomap);

    //Approprialy adjust edges cost when a node ocupanccy changeconst tf::Point
    // void HandleOccupancyChange(const int &node_index,const bool &change);
    bool isRayHit(const octomap::point3d& start,const octomap::point3d& end);

    void castRay(const octomap::point3d& start,const octomap::point3d& end, std::vector<int>& key_set);

    int toIndex(const float &x, const float &y, const float &z);

    int toIndex(const int &x, const int &y, const int &z);

    octomap::point3d toPosition(const int &index);

    void discritizePosition(const float &x, const float &y, const float &z, int &xkey, int &ykey, int &zkey);

    void getNeighborIndex(const int &index, std::vector<int> &neighbor);

    void getNeighborIndex(const int &index, std::vector<int> &neighbor,const float& radius);

    void getFreeNeighborIndex(const int &index, std::vector<int> &neighbor);

    bool isValidIndex(const int &value);

    bool isValidIndex(const int &x, const int &y, const int &z);

    bool isOccupied(const int& index);

    bool isOccupied(const octomap::point3d& position);

    bool isOccupied(const float& x, const float& y, const float& z);

    int getCost(const int& idx1, const int& idx2);

    //return the index of the current position close to idx1 on ray to idx2
    int steer(const int& idx1, const int& idx2,const float& radius);

    //randomly pick an index on uniform distribution
    int uni_random();
  };


#endif
