#ifndef MBF_GRIDMAP_CORE__GRIDMAP_CONTROLLER_H_
#define MBF_GRIDMAP_CORE__GRIDMAP_CONTROLLER_H_

#include "mbf_gridmap_core/gridmap_types.h"
#include <mbf_abstract_core/abstract_controller.h>
#include <mbf_utility/types.h>
#include <mutex>

namespace mbf_gridmap_core
{
/**
 * @class Local Planner/Controller
 * @brief Provides an interface for local planners used in navigation.
 * All local planners written to work as MBF plugins must adhere to this interface.
 */
class GridMapController : public mbf_abstract_core::AbstractController
{
public:
  typedef boost::shared_ptr<GridMapController> Ptr;

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands
   * to send to the base.
   * @param pose the current pose of the robot.
   * @param velocity the current velocity of the robot.
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on ExePath action result:
   *         SUCCESS         = 0
   *         1..9 are reserved as plugin specific non-error results
   *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
   *         CANCELED        = 101
   *         NO_VALID_CMD    = 102
   *         PAT_EXCEEDED    = 103
   *         COLLISION       = 104
   *         OSCILLATION     = 105
   *         ROBOT_STUCK     = 106
   *         MISSED_GOAL     = 107
   *         MISSED_PATH     = 108
   *         BLOCKED_PATH    = 109
   *         INVALID_PATH    = 110
   *         TF_ERROR        = 111
   *         NOT_INITIALIZED = 112
   *         INVALID_PLUGIN  = 113
   *         INTERNAL_ERROR  = 114
   *         121..149 are reserved as plugin specific errors
   */
  virtual uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                           const geometry_msgs::TwistStamped& velocity,
                                           geometry_msgs::TwistStamped& cmd_vel, std::string& message) = 0;

  /**
   * @brief Check if the goal pose has been achieved by the local planner within tolerance limits
   * @remark New on MBF API
   * @param xy_tolerance Distance tolerance in meters
   * @param yaw_tolerance Heading tolerance in radians
   * @return True if achieved, false otherwise
   */
  virtual bool isGoalReached(double xy_tolerance, double yaw_tolerance) = 0;

  /**
   * @brief  Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time
   * @remark New on MBF API
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param tf_ptr A shared pointer to a transform listener
   * @param local_gridmap_ptr A shared pointer to the local costmap (gridmap) to use for assigning and inferring costs
   * to local plans
   * @param global_gridmap_ptr A shared pointer to the global costmap (gridmap) to use for assigning and inferring costs
   * to local plans
   * @param local_gridmap_mtx_ptr A shared pointer to mutex object to protect access to the shared local gridmap
   * @param global_gridmap_mtx_ptr A shared pointer to mutex object to protect access to the shared global gridmap
   */
  virtual void initialize(const std::string& name, const TFPtr& tf_ptr,
                          const ::mbf_gridmap_core::GridmapSPtr& local_gridmap_ptr,
                          const ::mbf_gridmap_core::GridmapSPtr& global_gridmap_ptr,
                          const std::shared_ptr<std::mutex>& local_gridmap_mtx_ptr,
                          const std::shared_ptr<std::mutex>& global_gridmap_mtx_ptr) = 0;

  /**
   * @brief  Virtual destructor for the interface
   */
  virtual ~GridMapController() {}

protected:
  GridMapController() {}
};
}  // namespace mbf_gridmap_core

#endif  // MBF_GRIDMAP_CORE__GRIDMAP_CONTROLLER_H_
