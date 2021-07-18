#ifndef OCTOMAPSTATEVALIDATOR_H
#define OCTOMAPSTATEVALIDATOR_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/conversions.h>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

class OctomapStateValidator: public ompl::base::StateValidityChecker
{
private:
  octomap::OcTree* colorTree;
  std::shared_ptr<fcl::CollisionGeometry> tree_obj;
public:
  OctomapStateValidator(const ompl::base::SpaceInformationPtr& space_info,
                          const octomap_msgs::Octomap::ConstPtr& octomap)
    : ompl::base::StateValidityChecker(space_info){
      colorTree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*octomap));
      fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(colorTree));
      tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);

  }

  virtual bool isValid(const ompl::base::State* state) const {

    std::shared_ptr<fcl::CollisionGeometry> Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.7, 0.7, 0.7));
    fcl::CollisionObject aircraftObject(Quadcopter);
    fcl::CollisionObject treeObj((tree_obj));

    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType *se3state = state->as<ompl::base::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType *pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ompl::base::SO3StateSpace::StateType *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

	  // check validity of state defined by pos & rot
		fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
		fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
		aircraftObject.setTransform(rotation, translation);
		fcl::CollisionRequest requestType(1,false,1,false);
		fcl::CollisionResult collisionResult;
		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

		return(!collisionResult.isCollision());

  }
};

class OctomapMotionValidator: public ompl::base::MotionValidator
{
private:
  octomap::OcTree* colorTree;
public:
  OctomapMotionValidator(const ompl::base::SpaceInformationPtr& space_info,
                          const octomap_msgs::Octomap::ConstPtr& octomap): ompl::base::MotionValidator(space_info)
                          {
    colorTree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*octomap));
  }

  virtual bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const {
    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType *s_ = s1->as<ompl::base::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType *s = s_->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType *e_ = s2->as<ompl::base::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType *e = e_->as<ompl::base::RealVectorStateSpace::StateType>(0);

    octomap::point3d start(s->values[0],s->values[1],s->values[2]);
    octomap::point3d end(e->values[0],e->values[1],e->values[2]); 
    octomap::point3d direction = end - start;  
    // if(direction.norm() > 2) return false;
    octomap::point3d pos;

    return !colorTree->castRay(start, direction, pos, true, direction.norm());
  }

  virtual bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2,
                           std::pair<ompl::base::State*, double>& last_valid) const {
    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType *s_ = s1->as<ompl::base::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType *s = s_->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // cast the abstract state type to the type we expect
    const ompl::base::SE3StateSpace::StateType *e_ = s2->as<ompl::base::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ompl::base::RealVectorStateSpace::StateType *e = e_->as<ompl::base::RealVectorStateSpace::StateType>(0);

    octomap::point3d start(s->values[0],s->values[1],s->values[2]);
    octomap::point3d end(e->values[0],e->values[1],e->values[2]); 
    octomap::point3d direction = end - start; 
    octomap::point3d pos;

    // if(direction.norm() > 2){
    //   octomap::point3d vec = direction.normalized();
    //   pos = start + vec*2;
    //   if (last_valid.first) {
    //     ompl::base::ScopedState<ompl::base::SE3StateSpace> last_valid_state(
    //         si_->getStateSpace());
    //     last_valid_state->setXYZ(pos.x(),
		// 									pos.y(),
		// 									pos.z());
		// 		last_valid_state->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    
    //     si_->copyState(last_valid.first, last_valid_state.get());
    //   }
    
    //   last_valid.second = static_cast<double>(pos.distance(start) / end.distance(start));
    //   return false;
    // } 
    

    if(colorTree->castRay(start, direction, pos, true, direction.norm())){
      if (last_valid.first) {
        ompl::base::ScopedState<ompl::base::SE3StateSpace> last_valid_state(
            si_->getStateSpace());
        last_valid_state->setXYZ(pos.x(),
											pos.y(),
											pos.z());
				last_valid_state->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();
    
        si_->copyState(last_valid.first, last_valid_state.get());
      }
    
      last_valid.second = static_cast<double>(pos.distance(start) / end.distance(start));
      return false;
    }
    return true;
  }

  inline octomap::point3d omplToPoint(const ompl::base::State* state) const {
    const ompl::base::RealVectorStateSpace::StateType* derived
    = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);

    return octomap::point3d(derived->values[0], derived->values[1], derived->values[2]);
  }
};

#endif
