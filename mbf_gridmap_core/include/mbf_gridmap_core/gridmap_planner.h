#ifndef MBF_GRIDMAP_CORE__GRIDMAP_PLANNER_H_
#define MBF_GRIDMAP_CORE__GRIDMAP_PLANNER_H_

#include "gridmap_types.h"
#include <mbf_abstract_core/abstract_controller.h>
#include <mbf_utility/types.h>
#include <mutex>

namespace mbf_gridmap_core
{
/**
 * @class Global Planner/Intermediate Planner
 * @brief Provides an interface for global planners used in navigation.
 * All global planners written to work as MBF plugins must adhere to this interface.
 */
class GridMapPlanner : public mbf_abstract_core::AbstractPlanner
{
public:
  typedef boost::shared_ptr<GridMapPlanner> Ptr;

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
   *        in x and y before failing
   * @param plan The plan... filled by the planner
   * @param cost The cost for the the plan
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on GetPath action result:
   *         SUCCESS         = 0
   *         1..9 are reserved as plugin specific non-error results
   *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
   *         CANCELED        = 51
   *         INVALID_START   = 52
   *         INVALID_GOAL    = 53
   *         NO_PATH_FOUND   = 54
   *         PAT_EXCEEDED    = 55
   *         EMPTY_PATH      = 56
   *         TF_ERROR        = 57
   *         NOT_INITIALIZED = 58
   *         INVALID_PLUGIN  = 59
   *         INTERNAL_ERROR  = 60
   *         71..99 are reserved as plugin specific errors
   */
  virtual uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                            std::string& message) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @remark New on MBF API
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Initialization function for the GridMapPlanner
   * @param name The name of this planner
   * @param tf_ptr A pointer to a transform listener
   * @param local_gridmap_ptr A pointer to the local costmap (gridmap) to use for planning
   * @param global_gridmap_ptr A pointer to the global costmap (gridmap) to use for planning
   * @param gridmap_mtx_ptr A pointer to mutex object to protect simultaneous access to the shared local and global
   * gridmaps
   */
  virtual void initialize(std::string name, const TFPtr& tf_ptr,
                          const ::mbf_gridmap_core::GridmapSPtr& local_gridmap_ptr,
                          const ::mbf_gridmap_core::GridmapSPtr& global_gridmap_ptr,
                          const std::shared_ptr<std::mutex>& gridmap_mtx_ptr) = 0;

  /**
   * @brief  Virtual destructor for the interface
   */
  virtual ~GridMapPlanner() {}

protected:
  GridMapPlanner() {}
};
}  // namespace mbf_gridmap_core

#endif // MBF_GRIDMAP_CORE__GRIDMAP_PLANNER_H_
