#include "grid/Grid3D.h"
#include "grid/Node.h"
#include "grid/Edge.h"

#include <math.h>       /* round, floor, ceil, trunc, sqrt, pow */
#include <stdlib.h>     /* abs */
#include <set>
#include <iterator>
#include <algorithm>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

void Grid3D::Initilize(const octomap::point3d& origin){
  this->origin = origin;

  corner.x() = origin.x()-static_cast<float>(sizeX-1)*resolution/2.0f;
  corner.y() = origin.y()-static_cast<float>(sizeY-1)*resolution/2.0f;
  corner.z() = origin.z()-static_cast<float>(sizeZ-1)*resolution/2.0f;
}

  // void Grid3D::insertScanPoint(
  //   const pcl::PointCloud<pcl::PointXYZ>& points_cloud,
  //   const tf::Point& sensorOriginTf){

  //   octomap::point3d sensorOrigin(sensorOriginTf.x(), sensorOriginTf.y(), sensorOriginTf.z());

  //   std::set<int> all_nodes;
  //   std::set<int> occupied_nodes;

  //   for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = points_cloud.begin(); it != points_cloud.end(); ++it){
  //     octomap::point3d point(it->x, it->y, it->z);
  //     castRay(sensorOrigin, point, index_ray);
  //     all_nodes.insert(index_ray.begin(), index_ray.end());
  //     occupied_nodes.insert(index_ray.back());
  //   }
  //   std::set<int> free_nodes;
  //   std::set_difference(all_nodes.begin(),
  //     all_nodes.end(),
  //     occupied_nodes.begin(),
  //     occupied_nodes.end(),
  //     std::inserter(free_nodes, free_nodes.end()));

  // }

  // void Grid3D::insertOctomapCloud(const pcl::PointCloud<pcl::PointXYZ>& points_cloud){
  //   all_occupied_nodes.clear();
  //   for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = points_cloud.begin(); it != points_cloud.end(); ++it){
  //     if(isValidIndex(toIndex(it->x, it->y, it->z))){
  //       all_occupied_nodes.insert(toIndex(it->x, it->y, it->z));
  //     }
  //   }

  //   new_free_nodes.clear();
  //   new_occupied_nodes.clear();

  //   std::set_difference(temp_occupied_nodes.begin(),
  //     temp_occupied_nodes.end(),
  //     all_occupied_nodes.begin(),
  //     all_occupied_nodes.end(),
  //     std::inserter(new_free_nodes,new_free_nodes.end())
  //   );

  //   std::set_difference(all_occupied_nodes.begin(),
  //     all_occupied_nodes.end(),
  //     temp_occupied_nodes.begin(),
  //     temp_occupied_nodes.end(),
  //     std::inserter(new_occupied_nodes,new_occupied_nodes.end())
  //   );
  //   //

  //   for(std::set<int>::const_iterator it = new_free_nodes.begin(); it != new_free_nodes.end(); ++it){
  //     this->occupied_nodes[*it] = 0;
  //   }
  //   for(std::set<int>::const_iterator it = new_occupied_nodes.begin(); it != new_occupied_nodes.end(); ++it){
  //     this->occupied_nodes[*it] = 1;
  //   }

  //   temp_occupied_nodes = all_occupied_nodes;
  // }
  bool Grid3D::isRayHit(const octomap::point3d& start,const octomap::point3d& end){
    octomap::point3d direction = end - start;
    float hypotenuse = direction.norm();
    float total_distance = 0;
    direction = direction.normalize();
    octomap::point3d current_pos = start;
    while(total_distance + resolution< hypotenuse){
            total_distance += 0.1;
            current_pos =  current_pos + direction*0.1;
            if(isOccupied(current_pos)){
                return true;
            }
        }
    return false;
  }


  void Grid3D::castRay(const octomap::point3d& start,const octomap::point3d& end, std::vector<int>& key_set){
    key_set.clear();

    int  idx[3];
    discritizePosition(start.x(), start.y(), start.z(), idx[0], idx[1], idx[2]);
    int  endIdx[3];
    discritizePosition(end.x(), end.y(), end.z(), endIdx[0], endIdx[1], endIdx[2]);
    key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
    const int endIndex = toIndex(endIdx[0],endIdx[1],endIdx[2]);

    float dx = abs(end.x() - start.x());
    float dy = abs(end.y() - start.y());
    float dz = abs(end.z() - start.z());

    int stepX = start.x() < end.x() ? 1 : -1;
    int stepY = start.y() < end.y() ? 1 : -1;
    int stepZ = start.z() < end.z() ? 1 : -1;

    double hypotenuse = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    double tMaxX = hypotenuse*0.5 / dx * resolution;
    double tMaxY = hypotenuse*0.5 / dy * resolution;
    double tMaxZ = hypotenuse*0.5 / dz * resolution;
    double tDeltaX = hypotenuse / dx * resolution;
    double tDeltaY = hypotenuse / dy * resolution;
    double tDeltaZ = hypotenuse / dz * resolution;

    bool reach = true;
    while(reach){
      if (tMaxX < tMaxY) {
        if (tMaxX < tMaxZ) {
            idx[0] += stepX;
            tMaxX = tMaxX + tDeltaX;

            if(!isValidIndex(idx[0],idx[1],idx[2])){
              reach = false;
            }
            else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
              key_set.push_back(endIndex);
              reach = false;
            }
            else{
              key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
            }
        }
        else if (tMaxX > tMaxZ){
            idx[2] += stepZ;
            tMaxZ = tMaxZ + tDeltaZ;

            if(!isValidIndex(idx[0],idx[1],idx[2])){
              reach = false;
            }
            else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
              key_set.push_back(endIndex);
              reach = false;
            }
            else{
              key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
            }
        }
        else{
            idx[0] += stepX;
            tMaxX = tMaxX + tDeltaX;
            idx[2] += stepZ;
            tMaxZ = tMaxZ + tDeltaZ;

            if(!isValidIndex(idx[0],idx[1],idx[2])){
              reach = false;
            }
            else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
              key_set.push_back(endIndex);
              reach = false;
            }
            else{
              key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
            }
        }
      }
      else if (tMaxX > tMaxY){
          if (tMaxY < tMaxZ) {
              idx[1] += stepY;
              tMaxY = tMaxY + tDeltaY;

              if(!isValidIndex(idx[0],idx[1],idx[2])){
                reach = false;
              }
              else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
                key_set.push_back(endIndex);
                reach = false;
              }
              else{
                key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
              }
          }
          else if (tMaxY > tMaxZ){
              idx[2] += stepZ;
              tMaxZ = tMaxZ + tDeltaZ;

              if(!isValidIndex(idx[0],idx[1],idx[2])){
                reach = false;
              }
              else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
                key_set.push_back(endIndex);
                reach = false;
              }
              else{
                key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
              }
          }
          else{
              idx[1] += stepY;
              tMaxY = tMaxY + tDeltaY;
              idx[2] += stepZ;
              tMaxZ = tMaxZ + tDeltaZ;

              if(!isValidIndex(idx[0],idx[1],idx[2])){
                reach = false;
              }
              else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
                key_set.push_back(endIndex);
                reach = false;
              }
              else{
                key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
              }
          }
      }
      else{
          if (tMaxY < tMaxZ) {
              idx[1] += stepY;
              tMaxY = tMaxY + tDeltaY;
              idx[0] += stepX;
              tMaxX = tMaxX + tDeltaX;

              if(!isValidIndex(idx[0],idx[1],idx[2])){
                reach = false;
              }
              else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
                key_set.push_back(endIndex);
                reach = false;
              }
              else{
                key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
              }
          }
          else if (tMaxY > tMaxZ){
              idx[2] += stepZ;
              tMaxZ = tMaxZ + tDeltaZ;

              if(!isValidIndex(idx[0],idx[1],idx[2])){
                reach = false;
              }
              else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
                key_set.push_back(endIndex);
                reach = false;
              }
              else{
                key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
              }
          }
          else{
              idx[0] += stepX;
              tMaxX = tMaxX + tDeltaX;
              idx[1] += stepY;
              tMaxY = tMaxY + tDeltaY;
              idx[2] += stepZ;
              tMaxZ = tMaxZ + tDeltaZ;

              if(!isValidIndex(idx[0],idx[1],idx[2])){
                reach = false;
              }
              else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
                key_set.push_back(endIndex);
                reach = false;
              }
              else{
                key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
              }
          }
      }
    }
  }

  int Grid3D::toIndex(const float &x,const float &y,const float &z){
    int indexX = round((x - corner.x())/resolution);
    int indexY = round((y - corner.y())/resolution);
    int indexZ = round((z - corner.z())/resolution);

    if(indexX < 0 || indexX >= sizeX) return -1;
    if(indexY < 0 || indexY >= sizeY) return -1;
    if(indexZ < 0 || indexZ >= sizeZ) return -1;
    return indexX + sizeX*indexY + sizeX*sizeY*indexZ;
  }

  int Grid3D::toIndex(const int &x,const int &y,const int &z){
    return x + sizeX*y + sizeY*sizeX*z;
  }

  octomap::point3d Grid3D::toPosition(const int &index){
    int indexZ = index / (sizeX*sizeY);
    int modZ = index % (sizeX*sizeY);
    int indexY = modZ / sizeX;
    int indexX = modZ % sizeX;

    float posX = corner.x() + static_cast<float>(indexX)*resolution;
    float posY = corner.y() + static_cast<float>(indexY)*resolution;
    float posZ = corner.z() + static_cast<float>(indexZ)*resolution;

    octomap::point3d result(posX, posY, posZ);
    return result;
  }

  void Grid3D::discritizePosition(const float &x, const float &y, const float &z, int &xkey, int &ykey, int &zkey){
    xkey = (int)round((x - corner.x())/resolution);
    ykey = (int)round((y - corner.y())/resolution);
    zkey = (int)round((z - corner.z())/resolution);
  }

  void Grid3D::getNeighborIndex(const int &index, std::vector<int> &neighbor){
    neighbor.clear();
    if(isValidIndex(index+1)){
      neighbor.push_back(index+1);
    }
    if(isValidIndex(index-1)){
      neighbor.push_back(index-1);
    }

    if(isValidIndex(index+sizeX)){
      neighbor.push_back(index+sizeX);
    }

    if(isValidIndex(index-sizeX)){
      neighbor.push_back(index-sizeX);
    }

    if(isValidIndex(index+sizeX*sizeY)){
      neighbor.push_back(index+sizeX*sizeY);
    }

    if(isValidIndex(index-sizeX*sizeY)){
      neighbor.push_back(index-sizeX*sizeY);
    }

    return;
  }

  void Grid3D::getNeighborIndex(const int &index, std::vector<int> &neighbor,const float& radius){
    neighbor.clear();
    int x,y,z;
    octomap::point3d pos = this->toPosition(index);
    this->discritizePosition(pos.x(), pos.y(), pos.z(), x,y,z);
    int rad = round(radius/this->resolution);
    for(int i = -rad; i < rad + 1; i++){
      for(int j = -rad; j < rad + 1; j++){
        for(int k = -rad; k < rad + 1; k++){
          neighbor.push_back(toIndex(x+i,y+j,z+k));
        }
      }
    }
  }

  void Grid3D::getFreeNeighborIndex(const int &index, std::vector<int> &neighbor){
    neighbor.clear();
    std::vector<int> keyset;
    getNeighborIndex(index, keyset);
    for(std::vector<int>::iterator it = keyset.begin(); it != keyset.end(); ++it){
      if(!isRayHit(toPosition(index), toPosition(*it)) && !isOccupied(*it)){
        neighbor.push_back(*it);
      }
    }

    return;
  }

  bool Grid3D::isValidIndex(const int &value){
    if(value > -1 && value < sizeX*sizeY*sizeZ)
      return true;
    else
      return false;
  }

  bool Grid3D::isValidIndex(const int &x, const int &y, const int &z){
    if(x < 0 || x >= sizeX){
      return false;
    }

    if(y < 0 || y >= sizeY){
      return false;
    }

    if(z < 0 || z >= sizeZ){
      return false;
    }

    return true;
  }

  bool Grid3D::isOccupied(const int& index){
    if(!isValidIndex(index)){
      return true;
    }

    octomap::point3d pos = toPosition(index);
    fcl::Vec3f translation(pos.x(),pos.y(),pos.z());
    fcl::Quaternion3f rotation((float)1, (float)0, (float)0, (float)0);

    fcl::CollisionObject aircraftObject(Quadcopter);
    fcl::CollisionObject treeObj((tree_obj));

    aircraftObject.setTransform(rotation, translation);

    fcl::CollisionRequest requestType(1,false,1,false);
		fcl::CollisionResult collisionResult;
		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

		return(collisionResult.isCollision());
  }

  bool Grid3D::isOccupied(const octomap::point3d& position){
    fcl::Vec3f translation(position.x(),position.y(),position.z());
    fcl::Quaternion3f rotation((float)1, (float)0, (float)0, (float)0);

    fcl::CollisionObject aircraftObject(Quadcopter);
    fcl::CollisionObject treeObj((tree_obj));

    aircraftObject.setTransform(rotation, translation);

    fcl::CollisionRequest requestType(1,false,1,false);
		fcl::CollisionResult collisionResult;
		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

		return(collisionResult.isCollision());
  }

  bool Grid3D::isOccupied(const float& x, const float& y, const float& z){
    int index = toIndex(x, y, z);
    return isOccupied(index);
  }

  int Grid3D::getCost(const int& idx1, const int& idx2){
    if(isValidIndex(idx1) && isValidIndex(idx2)){
      return static_cast<int>(toPosition(idx1).distance(toPosition(idx2))*1000);
    }
    else{
      return 10e6;
    }
  }

  int Grid3D::steer(const int& idx1, const int& idx2, const float& radius){
    octomap::point3d pos1 = this->toPosition(idx1);
    octomap::point3d pos2 = this->toPosition(idx2);
    octomap::point3d delta = pos2 - pos1;
    float dis = delta.norm();
    if(dis < radius) return idx2;
    else{
      octomap::point3d del = pos1 + delta*(dis/radius);
      return this->toIndex(del.x(), del.y(), del.z());
    }
  }

  int Grid3D::uni_random(){
    return this->random_idx(this->generator);
  }

  void Grid3D::readOctomapMsg(const octomap_msgs::Octomap::ConstPtr& octomap){
    colorTree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*octomap));
    fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(colorTree));
    tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
  }
