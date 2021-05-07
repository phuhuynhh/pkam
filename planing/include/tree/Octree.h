#ifndef OCTREE_H
#define OCTREE_H

#include "tree/OctreeNode.h"
#include "grid/Grid3D.h"
#include <octomap/octomap.h>

class OctreeNode;
class Grid3D;

class Octree{
private:
  OctreeNode* root;
  Grid3D* grid;
  int max_depth = 8;
public:
  Octree(Grid3D* grid){
    this->grid = grid;
    *root = OctreeNode(grid->origin, octomap::point3d(
                                      grid->sizeX*grid->resolution,
                                      grid->sizeY*grid->resolution,
                                      grid->sizeZ*grid->resolution),
                                    0);
  }

  Octree(Grid3D* grid, const int& max_depth){
    this->grid = grid;
    this->max_depth = max_depth;
    *root = OctreeNode(grid->origin, octomap::point3d(
                                      grid->sizeX*grid->resolution,
                                      grid->sizeY*grid->resolution,
                                      grid->sizeZ*grid->resolution),
                                    0);
  }

  void add_index(const int& index);

  int nearest_index(const octomap::point3d& point);

  void expand(OctreeNode* node);

  void get_neighbor(const octomap::point3d& point, const float & radius, std::vector<int>& neighbors);
};

#endif
