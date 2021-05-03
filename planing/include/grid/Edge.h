#ifndef EDGE_H
#define EDGE_H

#include "grid/Node.h"

class Node;
 class Edge{
 private:
   Node* node_ptr1;
   Node* node_ptr2;
   int cost;
 public:
   Edge(){
     this->cost = 0;
   }

   Edge(Node* ptr1, Node* ptr2, int cost){
     node_ptr1 = ptr1;
     node_ptr2 = ptr2;
     this->cost = cost;
   }
   inline void set_cost(int cost){
     this->cost = cost;
   }

   inline const int& get_cost(){
     return this->cost;
   }
 };


#endif
