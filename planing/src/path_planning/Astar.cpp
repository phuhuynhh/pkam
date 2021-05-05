#include "test_pcl2/path_planning/Astar.h"
#include "test_pcl2/grid/Grid3D.h"
#include "test_pcl2/grid/updatable_priority_queue.h"
#include <algorithm>

void Astar::extend_id(size_t idx){
  size_t new_size = idx + 1;
  if(node_map.size() < new_size){
    node_map.resize(new_size);
    pre_node_map.resize(new_size, -1);
  }
}

bool Astar::find_path(const octomap::point3d& start_point,
                      std::vector<int>& node_index,
                      const unsigned int& iteration){
  int start_idx = grid->toIndex(start_point.x(), start_point.y(), start_point.z());
  int end_idx = grid->toIndex(target.x(), target.y(), target.z());

  node_map.clear();
  pre_node_map.clear();

  extend_id((size_t) start_idx);
  node_map[start_idx].setH(grid->getCost(start_idx,end_idx));
  node_map[start_idx].setG(0);
  pre_node_map[start_idx] = -1;

  updatable_priority_queue<int, int> queue;
  queue.push(start_idx, node_map[start_idx].getG() + node_map[start_idx].getH(), false);

  std::vector<int> expand;
  unsigned int i = 0;
  while(!queue.empty() && i < iteration){
    ++i;
    int top_key = queue.pop_value(false);
    if(top_key == end_idx){
      return_path(start_idx, end_idx, node_index);
      return true; // TODO, set the path to node_index
    }
    grid->getFreeNeighborIndex(top_key, expand);

    for(std::vector<int>::const_iterator it = expand.begin(); it != expand.end(); ++it){
      extend_id((size_t)(*it));
      node_map[*it].setIndex(*it);
      node_map[*it].setH(grid->getCost(*it, end_idx));
      int newG = node_map[top_key].getG() + static_cast<int>(grid->resolution*1000);

      if(node_map[*it].getG() > newG){
        node_map[*it].setParent(&node_map[top_key])
        node_map[*it].setG(newG);
        queue.update(*it, newG + node_map[*it].getH(), false);
        pre_node_map[*it] = top_key;
      }
    }
  }

  return false;
}

void Astar::return_path(const int& start_key, const int& end_key, std::vector<int>& node_index){
  node_index.clear();
  int current_key = end_key;
  while(current_key != -1 && current_key != start_key){
    node_index.push_back(pre_node_map[current_key]);
    current_key = pre_node_map[current_key];
  }

  std::reverse(node_index.begin(), node_index.end());
}
