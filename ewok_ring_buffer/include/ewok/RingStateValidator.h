#ifndef RINGSTATEVALIDATOR_H
#define RINGSTATEVALIDATOR_H

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ewok/ed_ring_buffer.h>

#include <Eigen/Dense>

namespace ewok{
    class RingStateValidator: public ompl::base::StateValidityChecker{
        private: 
        EuclideanDistanceRingBuffer<4>* rrb;
        
        public:
        RingStateValidator( const ompl::base::SpaceInformationPtr& space_info,
                            EuclideanDistanceRingBuffer<4>* field):ompl::base::StateValidityChecker(space_info)
        {
            this->rrb = field;
        }

        virtual bool isValid(const ompl::base::State* state) const {
            // cast the abstract state type to the type we expect
            const ompl::base::SE3StateSpace::StateType *se3state = state->as<ompl::base::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
            const ompl::base::RealVectorStateSpace::StateType *pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
            const ompl::base::SO3StateSpace::StateType *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

            Eigen::Vector3f position(pos->values[0],pos->values[1],pos->values[2]);
            Eigen::Vector3f grad;
            if(this->rrb->getDistanceWithGrad(position, grad) < 0.7){
                return false;
            }
            else{
                return true;
            }
        }
    };
}

#endif