#ifndef RRTSTAR_H
#define RRTSTAR_H

#include <vector>;
#include <octomap/octomap.h>

#include "grid/Node.h"
#include "grid/Grid3D.h"
#include "tree/Octree.h"

class Node;
class Octree;

class RRTStar{
private:
  std::vector<Node> node_map;
  std::vector<int> pre_node_map;
  float radius = 3.0;

public:
  Grid3D* grid;
  octomap::point3d target;
  Octree* tree;
  bool reached = false;

  RRTStar(Grid3D* grid,const octomap::point3d& target){
    this->grid = grid;
    this->target = target;
    *tree = Octree(grid);
  };

  void find_path(const octomap::point3d& start_point,
                        std::vector<int>& node_index,
                        const unsigned int& iteration);

  void return_path(const int& start_key, const int& end_key, std::vector<int>& node_index);

  void extend_id(size_t idx);

  void rewire(const int& new_index, std::vector<int>& neighbors);

};

#endif
