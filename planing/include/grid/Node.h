#ifndef NODE_H
#define NODE_H

#include "grid/Node.h"

  class Grid3D;
  class Node{
  private:
    Node* parent;
    long g;
    long rhs;
    long h;
    bool occupancy;
    int index;
    int infinity = 1000000000;

  public:
    Node(){
      index = -1;
      g = infinity;
      rhs = infinity;
      h = infinity;
      occupancy = false;
    }

    Node(Node* grid, int index){
      this->parent = grid;
      g = infinity;
      h = infinity;
      rhs = infinity;
      occupancy = false;
      this->index = index;
    }

    inline void setG(int value){
      g = value;
    }

    inline void setH(int value){
      h = value;
    }

    inline void setRHS(int value){
      rhs = value;
    }

    inline int getG(){
      return g;
    }

    inline int getRHS(){
      return rhs;
    }

    inline int getH(){
      return h;
    }

    inline void setParent(Node* ptr){
      this->parent = ptr;
    }

    inline void setIndex(int index){
      this->index = index;
    }

    inline int getIndex(){
      return this->index;
    }

    inline bool getOccupancy(){
      return occupancy;
    }

    void ChangeOccupancy(bool value);
  };


#endif
