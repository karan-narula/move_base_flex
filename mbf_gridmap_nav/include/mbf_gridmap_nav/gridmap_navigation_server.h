#ifndef MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_
#define MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_

#include <mbf_abstract_nav/abstract_navigation_server.h>
#include <mbf_gridmap_core/gridmap_types.h>
#include <mbf_gridmap_core/gridmap_controller.h>
#include <mbf_gridmap_core/gridmap_planner.h>
#include <mbf_gridmap_core/gridmap_recovery.h>

#include <std_srvs/Empty.h>
#include <mbf_msgs/CheckPath.h>
#include <mbf_msgs/CheckPose.h>
#include <mbf_msgs/CheckPoint.h>

#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>

#include <mutex>
#include <string>

namespace mbf_gridmap_nav
{
/**
 * @defgroup move_base_server Move Base Server
 * @brief Classes belonging to the Move Base Server level.
 */

class GridMapNavigationServer : public mbf_abstract_nav::AbstractNavigationServer
{
public:
  typedef boost::shared_ptr<GridMapNavigationServer> Ptr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  GridMapNavigationServer(const TFPtr& tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~GridMapNavigationServer();

private:
  /**
   * @brief Loads the plugin associated with the given planner_type parameter.
   * @param planner_type The type of the planner plugin to load.
   * @return true, if the local planner plugin was successfully loaded.
   */
  virtual mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string& planner_type);

  /**
   * @brief Initializes the controller plugin with its name, a pointer to the TransformListener
   *        and pointers to the gridmaps (local & global) and a pointer to the mutex object
   * @param name The name of the planner
   * @param planner_ptr pointer to the planner object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializePlannerPlugin(const std::string& name,
                                       const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr);

  /**
   * @brief Loads the plugin associated with the given controller type parameter
   * @param controller_type The type of the controller plugin
   * @return A shared pointer to a new loaded controller, if the controller plugin was loaded successfully,
   *         an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string& controller_type);

  /**
   * @brief Initializes the controller plugin with its name, a pointer to the TransformListener
   *        and pointers to the gridmaps (local & global) and a pointer to the mutex object
   * @param name The name of the controller
   * @param controller_ptr pointer to the controller object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializeControllerPlugin(const std::string& name,
                                          const mbf_abstract_core::AbstractController::Ptr& controller_ptr);

  /**
   * @brief Loads a Recovery plugin associated with given recovery type parameter
   * @param recovery_name The name of the Recovery plugin
   * @return A shared pointer to a Recovery plugin, if the plugin was loaded successfully, an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string& recovery_type);

  /**
   * @brief Initializes a recovery behavior plugin with its name, a pointer to the TransformListener
   *        and pointers to the gridmaps (local & global) and a pointer to the mutex object
   * @param name The name of the recovery behavior
   * @param behavior_ptr pointer to the recovery behavior object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializeRecoveryPlugin(const std::string& name,
                                        const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr);

  /**
   * @brief Callback method for the check_point_cost service
   * @param request Request object, see the mbf_msgs/CheckPoint service definition file.
   * @param response Response object, see the mbf_msgs/CheckPoint service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPointCost(mbf_msgs::CheckPoint::Request& request, mbf_msgs::CheckPoint::Response& response);

  /**
   * @brief Callback method for the check_pose_cost service
   * @param request Request object, see the mbf_msgs/CheckPose service definition file.
   * @param response Response object, see the mbf_msgs/CheckPose service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPoseCost(mbf_msgs::CheckPose::Request& request, mbf_msgs::CheckPose::Response& response);

  /**
   * @brief Callback method for the check_path_cost service
   * @param request Request object, see the mbf_msgs/CheckPath service definition file.
   * @param response Response object, see the mbf_msgs/CheckPath service definition file.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceCheckPathCost(mbf_msgs::CheckPath::Request& request, mbf_msgs::CheckPath::Response& response);

  /**
   * @brief Callback method for the make_plan service
   * @param request Empty request object.
   * @param response Empty response object.
   * @return true, if the service completed successfully, false otherwise
   */
  bool callServiceClearCostmaps(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  pluginlib::ClassLoader<mbf_gridmap_core::GridMapRecovery> recovery_plugin_loader_;
  // pluginlib::ClassLoader<nav_core::RecoveryBehavior> nav_core_recovery_plugin_loader_;
  pluginlib::ClassLoader<mbf_gridmap_core::GridMapController> controller_plugin_loader_;
  // pluginlib::ClassLoader<nav_core::BaseLocalPlanner> nav_core_controller_plugin_loader_;
  pluginlib::ClassLoader<mbf_gridmap_core::GridMapPlanner> planner_plugin_loader_;
  // pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> nav_core_planner_plugin_loader_;

  //! Shared pointer to the common local gridmap
  const mbf_gridmap_core::GridmapSPtr local_gridmap_ptr_;

  //! Shared pointer to the common global gridmap
  const mbf_gridmap_core::GridmapSPtr global_gridmap_ptr_;

  // Shared pointer to a mutex object for locking both local and global gridmaps
  const std::shared_ptr<std::mutex> gridmap_mtx_ptr_;

  // obstacle threshold cost for local and global gridmaps
  double local_obstacle_threshold_, global_obstacle_threshold_;

  //! Service Server for the check_point_cost service
  ros::ServiceServer check_point_cost_srv_;

  //! Service Server for the check_pose_cost service
  ros::ServiceServer check_pose_cost_srv_;

  //! Service Server for the check_path_cost service
  ros::ServiceServer check_path_cost_srv_;

  //! Service Server for the clear_gridmap service
  ros::ServiceServer clear_gridmaps_srv_;
};
}  // namespace mbf_gridmap_nav

#endif /* MBF_GRIDMAP_NAV__GRIDMAP_NAVIGATION_SERVER_H_ */
