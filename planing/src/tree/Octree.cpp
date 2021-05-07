#include "tree/Octree.h"
#include "tree/OctreeNode.h"

void Octree::add_index(const int& index){
  octomap::point3d point = this->grid->toPosition(index);
  OctreeNode* currentNode = this->root;
  bool stop = false;
  while(!stop){
    if(currentNode->children.size() > 1){
      OctreeNode* it = currentNode->children.data();
      currentNode->data.push_back(index);
      for(int i = 0; i <8; i++){
        if(it->isPointInside(point.x(), point.y(), point.z())){
          currentNode = it;
          break;
        }
        ++it;
      }
    }
    else{
      currentNode->data.push_back(index);
      if(currentNode->depth < this->max_depth && currentNode->data.size() >= currentNode->max_element){
        this->expand(currentNode);
      }
      stop = true;
    }
  }
}

void Octree::expand(OctreeNode* node){
  node->expand();
  for(std::vector<int>::const_iterator it = node->data.begin(); it != node->data.end(); ++it){
      octomap::point3d point = this->grid->toPosition(*it);
      for(std::vector<OctreeNode>::iterator child = node->children.begin(); child != node->children.end(); ++child){
        if(child->isPointInside(point.x(), point.y(), point.z())){
          child->data.push_back(*it);
          break;
        }
      }
  }
}

int Octree::nearest_index(const octomap::point3d& point){
  OctreeNode* currentNode = this->root;
  bool stop = false;
  while(!stop){
    if(currentNode->children.size() > 1){
      OctreeNode* it = currentNode->children.data();
      for(int i = 0; i <8; i++){
        if(it->isPointInside(point.x(), point.y(), point.z()) && it->data.size() > 0){
          currentNode = it;
          break;
        }
        else if(it->isPointInside(point.x(), point.y(), point.z())){
          stop = true;
          break;
        }
        ++it;
      }
    }
    else{
      stop = true;
    }
  }

  //find the radius for checking circle in the current Node;
  float dis = 1000000000;
  int index = -1;
  for(std::vector<int>::const_iterator it = currentNode->data.begin(); it != currentNode->data.end(); ++it){
      octomap::point3d p = this->grid->toPosition(*it);
      if(p.distance(point) < dis){
        dis = p.distance(point);
        index = *it;
      }
  }

  std::vector<int> indexToCheck;
  this->root->CircleIntersection(point, dis, indexToCheck);

  for(std::vector<int>::const_iterator it = indexToCheck.begin(); it != indexToCheck.end(); ++it){
    octomap::point3d p = this->grid->toPosition(*it);
    if(p.distance(point) < dis){
      dis = p.distance(point);
      index = *it;
    }
  }

  return index;
}

void Octree::get_neighbor(const octomap::point3d& point, const float & radius, std::vector<int>& neighbors){
  neighbors.clear();
  this->root->CircleIntersection(point, radius, neighbors);
}
