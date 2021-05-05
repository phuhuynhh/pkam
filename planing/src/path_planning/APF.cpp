#include "path_planning/APF.h"
#include <math.h>

octomap::point3d APF::calculate_velocity(octomap::point3d q, octomap::point3d q_end){
  octomap::point3d result;

  //calculate attraction velocity
  octomap::point3d F_att = q_end - q;
  F_att = F_att*this->k_att;
  result = result + F_att;

  //calculate repulsive velocity
  int currentIndex = this->grid->toIndex(q.x(),q.y(),q.z());

  std::vector<int> neighbor;
  this->grid->getNeighborIndex(currentIndex, neighbor, this->radius);
  for(std::vector<int>::const_iterator it = neighbor.begin(); it != neighbor.end(); ++it){
    if(this->grid->isOccupied(*it)){
      float dis = q.distance(this->grid->toPosition(*it));
      if(dis >= this->radius ){
        continue;
      }
      else{
        octomap::point3d repu = q - this->grid->toPosition(*it);
        repu = repu*pow((1/dis - 1/this->radius), this->n_rep)*k_rep;
        result = result + repu;
      }
    }
  }

  // cap to max velocity
  result = result.normalized();
  result = result * this->max_vel;

  return result;
}
