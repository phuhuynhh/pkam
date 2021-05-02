#include "grid/Grid3D.h"
#include "grid/Node.h"

void Node::ChangeOccupancy(bool value){
  if(value == this->occupancy){
    return;
  }
  else{
    this->occupancy = value;
    // parent->HandleOccupancyChange(index, value);
  }
}
