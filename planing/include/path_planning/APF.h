#ifndef APF_H
#define APF_H

#include "grid/Grid3D.h"
#include <octomap/octomap.h>
#include <vector>

class APF{
private:
  float max_vel = 0.2;
  float k_att = 1.0;
  float n_rep = 4.0;
  float radius = 2.0;
  Grid3D* grid;
public:
  APF(Grid3D* grid){
    this->grid = grid;
  }

  // return the velocity for movement update
  octomap::point3d calculate_velocity(octomap::point3d q, octomap::point3d q_end);

  inline void set_att(const float& k_att){
    this->k_att = k_att;
  }

  inline void set_rep(const float& n_rep){
    this->n_rep = n_rep;
  }

  inline void set_radius(const float& radius){
    this->radius = radius;
  }
};

#endif
