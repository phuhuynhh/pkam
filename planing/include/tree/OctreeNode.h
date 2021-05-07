#ifndef OCTREENODE_H
#define OCTREENODE_H

#include <vector>
#include <octomap/octomap.h>

class OctreeNode{
public:

  octomap::point3d origin;
  octomap::point3d size;

  int depth;
  int max_element = 10;

  std::vector<int> data;
  std::vector<OctreeNode> children;

  OctreeNode( const octomap::point3d& origin,
              const octomap::point3d& size,
              const int& depth){
                this->depth = depth;
                this->origin = origin;
                this->size = size;
              }
  OctreeNode( const octomap::point3d& origin,
              const octomap::point3d& size,
              const int& depth,
              const int& max_element){
                this->depth = depth;
                this->origin = origin;
                this->size = size;
                this->max_element = max_element;
              }

  void expand();

  bool isPointInside(const float& x, const float& y, const float& z);

  bool isCircleIntersect(const octomap::point3d& point, const float& radius);

  void CircleIntersection(const octomap::point3d& point, const float& radius, std::vector<int>& indexlist);
};

#endif
