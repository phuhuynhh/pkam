#ifndef APF_H
#define APF_H

#include <Eigen/Dense>
#include <vector>

namespace ewok
{
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

        typedef Eigen::Matrix<float, 3, 1> Vector3;
        typedef Eigen::Matrix<float, 4, 1> Vector4;
        typedef Eigen::Matrix<int, 3, 1> Vector3i;
        
        APF(){
        }

        // return the velocity for movement update
        // octomap::point3d calculate_velocity(octomap::point3d q, octomap::point3d q_end);

        void calculate( const Eigen::Vector3f& q,
                        const Eigen::Vector3f& q_end,
                        const float& distance,
                        const Eigen::Vector3f& distance_grad, 
                        Eigen::Vector3f& accel){
                            
            Eigen::Vector3f attract = q_end - q;
            attract = attract*k_att;

            if(distance > radius){
                accel = attract;
                return;
            }
            else{
                float repu_coff =  k_rep * (1 - distance / radius) * 1 / (distance * distance * distance);
                Eigen::Vector3f repu = repu_coff * (distance * distance_grad.normalized());

                accel = attract + repu;     
                return;
            }
        }

        inline void set_att(const float& k_att){
            this->k_att = k_att;
        }

        inline void set_rep(const float& n_rep){
            this->n_rep = n_rep;
        }

        inline void set_radius(const float& radius){
            this->radius = radius;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };


}

#endif