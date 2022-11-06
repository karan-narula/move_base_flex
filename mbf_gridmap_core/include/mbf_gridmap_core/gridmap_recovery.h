#ifndef MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_
#define MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_

#include "gridmap_types.h"
#include <mbf_abstract_core/abstract_controller.h>
#include <mbf_utility/types.h>
#include <mutex>

namespace mbf_gridmap_core
{
/**
 * @class GridMapRecovery
 * @brief Provides an interface for recovery behaviors used in navigation.
 * All recovery behaviors written to work as MBF plugins must adhere to this interface.
 */
class GridMapRecovery : public mbf_abstract_core::AbstractRecovery
{
public:
  typedef boost::shared_ptr<GridMapRecovery> Ptr;

  /**
   * @brief Initialization function for the GridMapRecovery
   * @param tf_ptr A pointer to a transform listener
   * @param local_gridmap_ptr A pointer to the local costmap (gridmap) used by the navigation stack
   * @param global_gridmap_ptr A pointer to the global costmap (gridmap) used by the navigation stack
   * @param gridmap_mtx_ptr A pointer to mutex object to protect simultaneous access to the shared local and global
   * gridmaps
   */
  virtual void initialize(std::string name, const TFPtr& tf_ptr,
                          const ::mbf_gridmap_core::GridmapSPtr& local_gridmap_ptr,
                          const ::mbf_gridmap_core::GridmapSPtr& global_gridmap_ptr,
                          const std::shared_ptr<std::mutex>& gridmap_mtx_ptr) = 0;

  /**
   * @brief Runs the GridMapRecovery
   * @param message The recovery behavior could set, the message should correspond to the return value
   * @return An outcome which will be hand over to the action result.
   */
  virtual uint32_t runBehavior(std::string& message) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time
   * @remark New on MBF API
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Virtual destructor for the interface
   */
  virtual ~GridMapRecovery() {}

protected:
  GridMapRecovery() {}
};
}  // namespace mbf_gridmap_core

#endif // MBF_GRIDMAP_CORE__GRIDMAP_RECOVERY_H_
