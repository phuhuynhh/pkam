#ifndef PLANNING_SETUP_H
#define PLANNING_SETUP_H

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "validator/OctomapStateValidator.h"

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

class OctomapMotionValidator;
class OctomapStateValidator;

class PlanningSetup: public ompl::geometric::SimpleSetup {
public:
  PlanningSetup() : ompl::geometric::SimpleSetup(ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace())){}

  // Get some defaults.
  void setDefaultObjective() {
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(getSpaceInformation()));
    obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
    getProblemDefinition()->setOptimizationObjective(obj);
  }

  void setDefaultPlanner() { setRrtStar(); }

  void setRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::RRTstar(getSpaceInformation())));
  }

  void setRrtConnect() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::RRTConnect(getSpaceInformation())));
  }

  void setInformedRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::InformedRRTstar(getSpaceInformation())));
  }

  void setPrm() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::PRM(getSpaceInformation())));
  }

  const ompl::base::StateSpacePtr& getGeometricComponentStateSpace() const {
    return getStateSpace();
  }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

  void setOctomapValidator(const octomap_msgs::Octomap::ConstPtr& octomap){
    std::shared_ptr<OctomapStateValidator> validity_checker(new OctomapStateValidator(getSpaceInformation(), octomap));
    setStateValidityChecker(validity_checker);
    // si_->setMotionValidator(ompl::base::MotionValidatorPtr(new OctomapMotionValidator(getSpaceInformation(),octomap)));
  }

};

#endif
