#include "path_planning/RRTStar.h"
#include "grid/Grid3D.h"

void RRTStar::extend_id(size_t idx){
  size_t new_size = idx + 1;
  if(node_map.size() < new_size){
    node_map.resize(new_size);
    pre_node_map.resize(new_size, -1);
  }
}

void RRTStar::find_path(const octomap::point3d& start_point,
                        std::vector<int>& node_index,
                        const unsigned int& iteration){

   int start_idx = this->grid->toIndex(start_point.x(), start_point.y(), start_point.z());
   int end_idx = this->grid->toIndex(target.x(), target.y(), target.z());
   node_map.clear();
   pre_node_map.clear();

   extend_id(start_idx);
   node_map[start_idx].setG(0);
   node_map[start_idx].setIndex(start_idx);

   this->tree->add_index(start_idx);
   std::vector<int> ray;
   std::vector<int> neighbors;

   unsigned int i = 0;
   while(i < iteration){
     ++i;
     //pick a new index
     int rand_idx = this->grid->uni_random();
     octomap::point3d pos = this->grid->toPosition(rand_idx);
     int nearest = this->tree->nearest_index(pos);
     int new_index = this->grid->steer(nearest, rand_idx, this->radius);
     if(this->grid->isOccupied(new_index)) continue;
     this->grid->castRay(grid->toPosition(nearest), grid->toPosition(new_index), ray);
     for(std::vector<int>::const_iterator it = ray.begin(); it != ray.end(); ++it){
       if(this->grid->isOccupied(*it)) goto OuterLoop;
     }
     this->extend_id((size_t)(new_index));

     //Handle wiring the tree
     if(node_map[new_index].getIndex() != -1) continue;
     node_map[new_index].setIndex(new_index);
     this->tree->get_neighbor(grid->toPosition(new_index), this->radius + 1.41*this->grid->resolution, neighbors);
     this->rewire(new_index, neighbors);
     this->tree->add_index(new_index);

     //if the the node is inside target radius
     if(!reached && this->target.distance(grid->toPosition(new_index)) < this->radius){
       this->grid->castRay(grid->toPosition(end_idx), grid->toPosition(new_index), ray);
       for(std::vector<int>::const_iterator it = ray.begin(); it != ray.end(); ++it){
         if(this->grid->isOccupied(*it)) goto OuterLoop;
       }

       extend_id(end_idx);
       node_map[end_idx].setIndex(end_idx);
       this->tree->get_neighbor(grid->toPosition(end_idx), this->radius + 1.41*this->grid->resolution, neighbors);
       this->rewire(end_idx, neighbors);
       reached = true;
     }

     OuterLoop:
      continue;
   }
   this->return_path(start_idx, end_idx, node_index);
}

void RRTStar::rewire(const int& new_index, std::vector<int>& neighbors){
  //Choose the best parent in the radius

  for(std::vector<int>::const_iterator it = neighbors.begin(); it != neighbors.end(); ++it){
    if(node_map[new_index].getG() > node_map[*it].getG() + grid->getCost(new_index,*it)){
      node_map[new_index].setG(node_map[*it].getG() + this->grid->getCost(new_index, *it));
      pre_node_map[new_index] = *it;
    }
  }

  //rewire the tree
  for(std::vector<int>::const_iterator it = neighbors.begin(); it != neighbors.end(); ++it){
    if(node_map[*it].getG() > node_map[new_index].getG() + grid->getCost(new_index,*it)){
      node_map[*it].setG(node_map[new_index].getG() + grid->getCost(new_index,*it));
      pre_node_map[*it] = new_index;
    }
  }
}

void RRTStar::return_path(const int& start_key, const int& end_key, std::vector<int>& node_index){
  node_index.clear();
  int current_key = end_key;
  while(current_key != -1 && current_key != start_key){
    node_index.push_back(pre_node_map[current_key]);
    current_key = pre_node_map[current_key];
  }

  std::reverse(node_index.begin(), node_index.end());
}
