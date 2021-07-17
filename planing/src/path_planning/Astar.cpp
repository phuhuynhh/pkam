#include "path_planning/Astar.h"
#include "grid/Grid3D.h"
#include "grid/updatable_priority_queue.h"
#include <algorithm>
#include <chrono>

#include <ros/ros.h>
#include <ros/subscribe_options.h>

void Astar::extend_id(int idx){
  if(node_map.find(idx) == node_map.end()){
    node_map[idx] = Node();
    pre_node_map[idx] = -1;
  }
}

bool Astar::find_path(const octomap::point3d& start_point,
                        std::vector<octomap::point3d>& path,
                        std::vector<octomap::point3d>& path_smooth,
                        const unsigned int& iteration){

  auto start = std::chrono::high_resolution_clock::now();                      
  int start_idx = grid->toIndex(start_point.x(), start_point.y(), start_point.z());
  int end_idx = grid->toIndex(target.x(), target.y(), target.z());

  if(grid->isOccupied(end_idx)){
            ROS_INFO("Astar: The target is unreachable");
            return false;
  }
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
      Node goal_node = node_map[end_idx];
      return_path(start_idx, end_idx, path);
      auto end1 = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start);
      path.insert(path.begin(), start_point);
      path.push_back(target);
      ROS_INFO("Astar find a goal with cost: %f, number of state expand: %d, number of waypoints: %d, iteration: %d, in time: %d", ((float) goal_node.getG())/1000.0, node_map.size(), path.size(),i, duration);


      path_pruning(path, path_smooth);
      float cost = 0;
      for(int i = 0; i < path_smooth.size() -1; i++){
        cost += (path_smooth[i] - path_smooth[i+1]).norm();
      }
      auto end2 = std::chrono::high_resolution_clock::now();
      auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start);
      ROS_INFO("Astar smoothing find a goal with cost: %f, number of state expand: %d,  number of waypoints: %d,iteration: %d, in time: %d", cost, node_map.size(),path_smooth.size(), i, duration2);
      return true; // TODO, set the path to node_index
    }
    grid->getFreeNeighborIndex(top_key, expand);

    for(std::vector<int>::const_iterator it = expand.begin(); it != expand.end(); ++it){
      extend_id((size_t)(*it));
      node_map[*it].setIndex(*it);
      node_map[*it].setH(grid->getCost(*it, end_idx));
      int newG = node_map[top_key].getG() + static_cast<int>(grid->resolution*1000);

      if(node_map[*it].getG() > newG){
        node_map[*it].setParent(&node_map[top_key]);
        node_map[*it].setG(newG);
        queue.update(*it, newG + node_map[*it].getH(), false);
        pre_node_map[*it] = top_key;
      }
    }
  }

  return false;
}

void Astar::return_path(const int& start_key, const int& end_key, std::vector<octomap::point3d>& node_index){
  node_index.clear();
  Node goal_node = node_map[end_key];
  int current_key = end_key;
  while(current_key != -1 && current_key != start_key){
    node_index.insert(node_index.begin(),grid->toPosition(current_key));
    current_key = pre_node_map[current_key];
  }
}

void Astar::path_pruning(std::vector<octomap::point3d>& input_path, std::vector<octomap::point3d>& output_path){
  output_path.clear();
  int current_index = 0;
  output_path.push_back( input_path[current_index]);
  while(current_index < input_path.size()-1){
    int next_index = current_index;
    for(int i = current_index; i < input_path.size(); i++){
      if(i == current_index + 1){
        next_index = i;
      }
      if(!grid->isRayHit(input_path[current_index], input_path[i])){
        next_index = i;
      }
    }
    current_index = next_index;
    output_path.push_back( input_path[current_index]);
  }
}
