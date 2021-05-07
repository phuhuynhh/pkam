#include "tree/OctreeNode.h"
#include <cmath>        // std::abs
#include <math.h>

void OctreeNode::expand(){
  this->children.clear();
  for(int i = -1; i < 2; i++){
    for(int j = -1; j < 2; j++){
      for(int k = -1; k < 2; k++){
        float x = this->origin.x() + this->size.x()*(float(i) / 4.0);
        float y = this->origin.y() + this->size.y()*(float(j) / 4.0);
        float z = this->origin.z() + this->size.z()*(float(k) / 4.0);

        this->children.push_back(
          OctreeNode(octomap::point3d(x,y,z), this->size*0.5, this->depth + 1)
        );
      }
    }
  }
}

bool OctreeNode::isPointInside(const float& x, const float& y, const float& z){
  octomap::point3d max = this->origin + this->size*0.5;
  octomap::point3d min = this->origin - this->size*0.5;

  if( x < max.x() && x > min.x() &&
      y < max.y() && y > min.y() &&
      z < max.z() && z > min.z()){
        return true;
      }
  return false;
}

bool OctreeNode::isCircleIntersect(const octomap::point3d& point, const float& radius){
  float disX = std::abs(point.x() - this->origin.x());
  float disY = std::abs(point.y() - this->origin.y());
  float disZ = std::abs(point.z() - this->origin.z());

  if(disX > (this->size.x()/2.0 + radius)) return false;
  if(disY > (this->size.y()/2.0 + radius)) return false;
  if(disZ > (this->size.z()/2.0 + radius)) return false;

  float cornerdis = (disX - this->size.x()/2.0)*(disX - this->size.x()/2.0) +
  (disY - this->size.y()/2.0)*(disY - this->size.y()/2.0) +
  (disZ - this->size.z()/2.0)*(disZ - this->size.z()/2.0);
  return (cornerdis <= radius*radius);
}

void OctreeNode::CircleIntersection(const octomap::point3d& point, const float& radius, std::vector<int>& indexlist){
  if(this->children.size() < 1){
    indexlist.insert(indexlist.end(), this->data.begin(), this->data.end());
  }
  else{
    for(std::vector<OctreeNode>::iterator it = this->children.begin(); it != this->children.end(); ++it){
      if(it->isCircleIntersect(point, radius)) it->CircleIntersection(point, radius, indexlist);
    }
  }
}
