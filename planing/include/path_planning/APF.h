#ifndef APF_H
#define APF_H

#include "grid/Grid3D.h"
#include <octomap/octomap.h>
#include <vector>

class APF{
private:
  float max_vel = 1.0;
  float max_accel = 1.0;
  float k_att = 4.0;
  float k_rep = 2.0;
  float n_rep = 0.3;
  float radius = 1.2;
  // Grid3D* grid;
public:
  APF(){
  }

  // return the velocity for movement update
  // octomap::point3d calculate_velocity(octomap::point3d q, octomap::point3d q_end);

  void calculate(const octomap::point3d& q,
                  const octomap::point3d& q_end,
                  const octomap::point3d& distance_grad, 
                  octomap::point3d& accel);

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
