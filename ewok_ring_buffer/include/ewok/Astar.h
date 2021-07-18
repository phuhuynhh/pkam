#ifndef ASTAR_H
#define ASTAR_H

#include <ewok/Grid3D.h>
#include <ewok/Node.h>
#include <ewok/updatable_priority_queue.h>
#include <vector>
#include <octomap/octomap.h>
#include <map>
#include <algorithm>
#include <chrono>

#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <Eigen/Dense>
namespace ewok{
    class Astar{
private:
  std::map<int, Node> node_map;
  std::map<int, int> pre_node_map;
public:
    Grid3D* grid;
    Eigen::Vector3f target;

    Astar(const Eigen::Vector3f& end, Grid3D* grid){
        this->grid = grid;
        target = end;
    }

    bool find_path(const Eigen::Vector3f& start_point,
                        std::vector<Eigen::Vector3f>& path,
                        std::vector<Eigen::Vector3f>& path_smooth,
                        const unsigned int& iteration){
        auto start = std::chrono::high_resolution_clock::now();            
        int start_idx = grid->toIndex(start_point(0), start_point(1), start_point(2));
        int end_idx = grid->toIndex(target(0), target(1), target(2));

        if(grid->isOccupied(end_idx)){
            printf("Astar: The target is unreachable");
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
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start);
                path.insert(path.begin(), start_point);
                path.push_back(target);
                ROS_INFO("Astar find a goal with cost: %f, number of state expand: %d, number of waypoints: %d, iteration: %d, in time: %d", ((float) goal_node.getG())/1000.0, node_map.size(), path.size(),i, duration);


                path_pruning(path, path_smooth);
                float cost = 0;
                for(int i = 0; i < path_smooth.size() -1; i++){
                    cost += (path_smooth[i] - path_smooth[i+1]).norm();
                }
                auto end2 = std::chrono::high_resolution_clock::now();
                auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start);
                ROS_INFO("Astar smoothing find a goal with cost: %f, number of state expand: %d,  number of waypoints: %d,iteration: %d, in time: %d", cost, node_map.size(),path_smooth.size(), i, duration2);
                return true; // TODO, set the path to node_index                                                  
            }
            grid->getFreeNeighborIndex(top_key, expand);

            for(std::vector<int>::const_iterator it = expand.begin(); it != expand.end(); ++it){
                extend_id((size_t)(*it));
                node_map[*it].setIndex(*it);
                node_map[*it].setH(grid->getCost(*it, end_idx));
                int newG = node_map[top_key].getG() + grid->getCost(*it,top_key);

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

    void return_path(const int& start_key, const int& end_key, std::vector<Eigen::Vector3f>& node_index){
        node_index.clear();
        Node goal_node = node_map[end_key];
        int current_key = end_key;
        while(current_key != -1 && current_key != start_key){
            node_index.insert(node_index.begin(),grid->toPosition(current_key));
            current_key = pre_node_map[current_key];
        }
    }

    void path_pruning(std::vector<Eigen::Vector3f>& input_path, std::vector<Eigen::Vector3f>& output_path){
        output_path.clear();
        int current_index = 0;
        output_path.push_back(input_path[current_index]);
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

    void extend_id(int idx){
        if(node_map.find(idx) == node_map.end()){
            node_map[idx] = Node();
            pre_node_map[idx] = -1;
        }
    }


};
}

#endif
