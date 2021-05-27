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

    std::shared_ptr<fcl::CollisionGeometry> Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.3, 0.3, 0.1));
    fcl::CollisionObject aircraftObject(Quadcopter);

    const ompl::base::RealVectorStateSpace::StateType *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();

		fcl::CollisionObject treeObj((tree_obj));
    
	    // check validity of state defined by pos & rot
		fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
		fcl::Quaternion3f rotation(1, 0, 0, 0);
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
    std::pair<ompl::base::State*, double> unused;
    return checkMotion(s1, s2, unused);

    // return true;
  }

  virtual bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2,
                           std::pair<ompl::base::State*, double>& last_valid) const {
    // octomap::point3d start = omplToPoint(s1);
    // octomap::point3d end = omplToPoint(s2);
    // unsigned int depth = colorTree->getTreeDepth();
    //
    // octomap::point3d pos;
    //
    // if(colorTree->castRay(start, end, pos, true)){
    //   if (last_valid.first) {
    //     ompl::base::ScopedState<ompl::base::RealVectorStateSpace> last_valid_state(
    //         si_->getStateSpace());
    //     last_valid_state->values[0] = pos.x();
    //     last_valid_state->values[1] = pos.y();
    //     last_valid_state->values[2] = pos.z();
    //
    //     si_->copyState(last_valid.first, last_valid_state.get());
    //   }
    //
    //   last_valid.second = static_cast<double>(pos.distance(start) / end.distance(start));
    //   return false;
    // }

    return true;
  }

  inline octomap::point3d omplToPoint(const ompl::base::State* state) const {
    const ompl::base::RealVectorStateSpace::StateType* derived
    = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);

    return octomap::point3d(derived->values[0], derived->values[1], derived->values[2]);
  }
};

#endif
