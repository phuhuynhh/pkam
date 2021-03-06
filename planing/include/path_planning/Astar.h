#ifndef ASTAR_H
#define ASTAR_H

#include "grid/Grid3D.h"
#include "grid/Node.h"
#include "grid/updatable_priority_queue.h"
#include <vector>
#include <octomap/octomap.h>
#include <map>

class Grid3D;
class Node;

class Astar{
private:
  std::map<int, Node> node_map;
  std::map<int, int> pre_node_map;
public:
  Grid3D* grid;
  octomap::point3d target;

  Astar(const octomap::point3d& end, Grid3D* grid){
      this->grid = grid;
      target = end;
  }

  bool find_path(const octomap::point3d& start_point,
                        std::vector<octomap::point3d>& path,
                        std::vector<octomap::point3d>& path_smooth,
                        const unsigned int& iteration);

  void return_path(const int& start_key, const int& end_key, std::vector<octomap::point3d>& node_index);

  void path_pruning(std::vector<octomap::point3d>& input_path, std::vector<octomap::point3d>& output_path);

  void extend_id(int idx);


};

#endif
