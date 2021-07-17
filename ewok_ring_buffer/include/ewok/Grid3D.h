#ifndef GRID3D_H
#define GRID3D_H

#include <bits/stdc++.h>
#include <vector>
#include <set>
#include <unordered_map>
#include <random>

#include <Eigen/Dense>

#include <ewok/ed_ring_buffer.h>

namespace ewok{
    class Grid3D{
    public:
    int infinity = 10e6;

    Eigen::Vector3f origin;
    float resolution;
    Eigen::Vector3f corner;
    int sizeX, sizeY, sizeZ;

    std::vector<int> index_ray; // temporate storage for ray casting
    std::vector<int> neighborIdx; //temporate storage for neighbor finding

    EuclideanDistanceRingBuffer<4>* rrb;
    // std::vector<Node> node_list;
    // std::vector<std::unordered_map<int, Edge>> adjency_list;

    Grid3D(
      int x,
      int y,
      int z,
      const float& resolution
    ){
      this->resolution = resolution;

      this->sizeX = x;
      this->sizeY = y;
      this->sizeZ = z;
    }

    void Initilize(const Eigen::Vector3f& origin, EuclideanDistanceRingBuffer<4>* ed_field){
        this->origin = origin;
        this->rrb = ed_field;
        corner(0) = origin(0)-static_cast<float>(sizeX-1)*resolution/2.0f;
        corner(1) = origin(1)-static_cast<float>(sizeY-1)*resolution/2.0f;
        corner(2) = origin(2)-static_cast<float>(sizeZ-1)*resolution/2.0f;
    }

    int toIndex(const float &x, const float &y, const float &z){
        int indexX = round((x - corner(0))/resolution);
        int indexY = round((y - corner(1))/resolution);
        int indexZ = round((z - corner(2))/resolution);

        if(indexX < 0 || indexX >= sizeX) return -1;
        if(indexY < 0 || indexY >= sizeY) return -1;
        if(indexZ < 0 || indexZ >= sizeZ) return -1;
        return indexX + sizeX*indexY + sizeX*sizeY*indexZ;
    }

    int toIndex(const int &x, const int &y, const int &z){
        return x + sizeX*y + sizeY*sizeX*z;
    }

    Eigen::Vector3f toPosition(const int &index){
        int indexZ = index / (sizeX*sizeY);
        int modZ = index % (sizeX*sizeY);
        int indexY = modZ / sizeX;
        int indexX = modZ % sizeX;

        float posX = corner.x() + static_cast<float>(indexX)*resolution;
        float posY = corner.y() + static_cast<float>(indexY)*resolution;
        float posZ = corner.z() + static_cast<float>(indexZ)*resolution;

        Eigen::Vector3f result(posX, posY, posZ);
        return result;
    }

    void discritizePosition(const float &x, const float &y, const float &z, int &xkey, int &ykey, int &zkey){
        xkey = (int)round((x - corner.x())/resolution);
        ykey = (int)round((y - corner.y())/resolution);
        zkey = (int)round((z - corner.z())/resolution);
    }

    void getNeighborIndex(const int &index, std::vector<int> &neighbor){
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

    void getFreeNeighborIndex(const int &index, std::vector<int> &neighbor){
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

    bool isValidIndex(const int &value){
        if(value > -1 && value < sizeX*sizeY*sizeZ)
            return true;
        else
            return false;
    }

    bool isValidIndex(const int &x, const int &y, const int &z){
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

    bool isOccupied(const int& index){
        if(!isValidIndex(index)){
            return true;
        }
        Eigen::Vector3f pos = toPosition(index);

        if(rrb->getDistanceWithGrad(pos, Eigen::Vector3f()) < 0.7){
            return true;
        }

        return false;
    }

    bool isOccupied(const Eigen::Vector3f& pos){
        if(rrb->getDistanceWithGrad(pos, Eigen::Vector3f()) < 0.7){
            return true;
        }

        return false;
    }

    bool isOccupied(const float& x, const float& y, const float& z){
        int index = toIndex(x, y, z);
        return isOccupied(index);
    }

    int getCost(const int& idx1, const int& idx2){
        if(isValidIndex(idx1) && isValidIndex(idx2)){
            Eigen::Vector3f pos1 = this->toPosition(idx1);
            Eigen::Vector3f pos2 = this->toPosition(idx2);
            Eigen::Vector3f delta;
            delta = pos2 - pos1; 
            return static_cast<int>(delta.norm()*1000.0);
        } 
        else{
            return 10e9;
        }  
    }

    bool isRayHit(const Eigen::Vector3f& start,const Eigen::Vector3f& end){
        float dx = abs(end(0) - start(0));
        float dy = abs(end(1) - start(1));
        float dz = abs(end(2) - start(2));
        Eigen::Vector3f direction = end -start;
        direction.normalize();
        Eigen::Vector3f current_pos = start;
        double hypotenuse = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        double total_distance = 0;
        while(total_distance + resolution< hypotenuse){
            total_distance += 0.1;
            current_pos =  current_pos + direction*0.1;
            if(rrb->getDistanceWithGrad(current_pos, Eigen::Vector3f()) < 0.7){
                return true;
            }
        }
        return false;
    }

    void castRay(const Eigen::Vector3f& start,const Eigen::Vector3f& end, std::vector<int>& key_set){
        key_set.clear();

        float dx = abs(end(0) - start(0));
        float dy = abs(end(1) - start(1));
        float dz = abs(end(2) - start(2));
        Eigen::Vector3f direction = end -start;
        direction.normalize();
        Eigen::Vector3f current_pos = start;
        double hypotenuse = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        double total_distance = 0;
        while(total_distance + resolution< hypotenuse){
            total_distance += resolution / 2;
            current_pos =  current_pos + direction*resolution/2;
            key_set.push_back(toIndex(current_pos(0), current_pos(1), current_pos(2)));
        }

        // if(toIndex(start(0), start(1), start(2)) == toIndex(end(0), end(1), end(2))){
        //     key_set.push_back(toIndex(start(0), start(1), start(2)));
        //     return;
        // }

        // int  idx[3];
        // discritizePosition(start(0), start(1), start(2), idx[0], idx[1], idx[2]);
        // Eigen::Vector3f start_i = toPosition(toIndex(start(0), start(1), start(2)));
        // int  endIdx[3];
        // discritizePosition(end(0), end(1), end(2), endIdx[0], endIdx[1], endIdx[2]);
        // key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        // const int endIndex = toIndex(endIdx[0],endIdx[1],endIdx[2]);

        // float dx = abs(end(0) - start(0));
        // float dy = abs(end(1) - start(1));
        // float dz = abs(end(2) - start(2));

        // int stepX = start(0) < end(0) ? 1 : -1;
        // int stepY = start(1) < end(1) ? 1 : -1;
        // int stepZ = start(2) < end(2) ? 1 : -1;

        // double hypotenuse = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        // double tMaxX = (resolution / 2.0 - abs(start(0)-start_i(0))) / dx;
        // double tMaxY = (resolution / 2.0 - abs(start(1)-start_i(1))) / dy;
        // double tMaxZ = (resolution / 2.0 - abs(start(2)-start_i(2))) / dz;
        // double tDeltaX = hypotenuse / dx ;
        // double tDeltaY = hypotenuse / dy ;
        // double tDeltaZ = hypotenuse / dz ;

        // bool reach = true;
        // while(reach){
        //     if (tMaxX < tMaxY) {
        //         if (tMaxX < tMaxZ) {
        //             idx[0] += stepX;
        //             tMaxX = tMaxX + tDeltaX;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //             reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //             key_set.push_back(endIndex);
        //             reach = false;
        //             }
        //             else{
        //             key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //         else if (tMaxX > tMaxZ){
        //             idx[2] += stepZ;
        //             tMaxZ = tMaxZ + tDeltaZ;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //             reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //             key_set.push_back(endIndex);
        //             reach = false;
        //             }
        //             else{
        //             key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //         else{
        //             idx[0] += stepX;
        //             tMaxX = tMaxX + tDeltaX;
        //             idx[2] += stepZ;
        //             tMaxZ = tMaxZ + tDeltaZ;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //             reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //             key_set.push_back(endIndex);
        //             reach = false;
        //             }
        //             else{
        //             key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //     }
        //     else if (tMaxX > tMaxY){
        //         if (tMaxY < tMaxZ) {
        //             idx[1] += stepY;
        //             tMaxY = tMaxY + tDeltaY;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //                 reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //                 key_set.push_back(endIndex);
        //                 reach = false;
        //             }
        //             else{
        //                 key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //         else if (tMaxY > tMaxZ){
        //             idx[2] += stepZ;
        //             tMaxZ = tMaxZ + tDeltaZ;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //                 reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //                 key_set.push_back(endIndex);
        //                 reach = false;
        //             }
        //             else{
        //                 key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //         else{
        //             idx[1] += stepY;
        //             tMaxY = tMaxY + tDeltaY;
        //             idx[2] += stepZ;
        //             tMaxZ = tMaxZ + tDeltaZ;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //                 reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //                 key_set.push_back(endIndex);
        //                 reach = false;
        //             }
        //             else{
        //                 key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //     }
        //     else{
        //         if (tMaxY < tMaxZ) {
        //             idx[1] += stepY;
        //             tMaxY = tMaxY + tDeltaY;
        //             idx[0] += stepX;
        //             tMaxX = tMaxX + tDeltaX;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //                 reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //                 key_set.push_back(endIndex);
        //                 reach = false;
        //             }
        //             else{
        //                 key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //         else if (tMaxY > tMaxZ){
        //             idx[2] += stepZ;
        //             tMaxZ = tMaxZ + tDeltaZ;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //                 reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //                 key_set.push_back(endIndex);
        //                 reach = false;
        //             }
        //             else{
        //                 key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //         else{
        //             idx[0] += stepX;
        //             tMaxX = tMaxX + tDeltaX;
        //             idx[1] += stepY;
        //             tMaxY = tMaxY + tDeltaY;
        //             idx[2] += stepZ;
        //             tMaxZ = tMaxZ + tDeltaZ;

        //             if(!isValidIndex(idx[0],idx[1],idx[2])){
        //                 reach = false;
        //             }
        //             else if (toIndex(idx[0],idx[1],idx[2]) == endIndex){
        //                 key_set.push_back(endIndex);
        //                 reach = false;
        //             }
        //             else{
        //                 key_set.push_back(toIndex(idx[0],idx[1],idx[2]));
        //             }
        //         }
        //     }
        // }
    }
  };
}


#endif
