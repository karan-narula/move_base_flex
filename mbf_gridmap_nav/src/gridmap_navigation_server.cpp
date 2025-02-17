#include "mbf_gridmap_nav/gridmap_navigation_server.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <xmlrpcpp/XmlRpc.h>
#include <limits>

namespace mbf_gridmap_nav
{
void readGridMapLayers(const std::string& resource, const ros::NodeHandle& nh, std::vector<std::string>& layers,
                       mbf_gridmap_core::Gridmap& gridmap)
{
  using namespace XmlRpc;
  XmlRpcValue raw;

  // Load the data from the param server
  if (!nh.getParam(resource, raw))
    throw std::runtime_error("No parameter " + resource);

  // We expect that resource defines an array
  if (raw.getType() != XmlRpcValue::TypeArray)
    throw std::runtime_error(resource + " must be an XmlRpcValue::TypeArray");

  // allocate vector of layers
  layers.reserve(raw.size());
  for (std::size_t i = 0; i != raw.size(); ++i)
  {
    layers.push_back(static_cast<std::string>(raw[i]));
    gridmap.add(layers.back());
  }
}

void setUpGridMap(const std::string& loc_glob_field, const ros::NodeHandle& nh, mbf_gridmap_core::Gridmap& gridmap,
                  double& obstacle_threshold)
{
  // read and set frame id for the gridmap
  std::string gridmap_frame;
  nh.param<std::string>(loc_glob_field + "/frame", gridmap_frame, "map");
  gridmap.setFrameId(gridmap_frame);

  // set the geometry of the gridmap based on sensor range, the center position is assumed to be at origin to begin with
  double length, width, resolution;
  if (nh.getParam(loc_glob_field + "/length", length) && nh.getParam(loc_glob_field + "/width", width) &&
      nh.getParam(loc_glob_field + "/resolution", resolution))
  {
    gridmap.setGeometry(grid_map::Length(length, width), resolution, grid_map::Position(0, 0));
  }

  // read the obstacle threshold cost
  nh.param<double>(loc_glob_field + "/obstacle_threshold", obstacle_threshold, std::numeric_limits<double>::infinity());

  // add basic and all other layers to the gridmap
  std::string layer_field = loc_glob_field + "/basic_gridmap_layers";
  std::vector<std::string> basic_layers, other_layers;
  readGridMapLayers(layer_field, nh, basic_layers, gridmap);
  gridmap.setBasicLayers(basic_layers);
  layer_field = loc_glob_field + "/other_gridmap_layers";
  readGridMapLayers(layer_field, nh, other_layers, gridmap);
}

ros::Rate setUpGridMapPublisher(const std::string& loc_glob_field, ros::NodeHandle& nh,
                                const mbf_gridmap_core::GridmapSPtr& gridmap_ptr, ros::Publisher& gridmap_pub,
                                std::map<std::string, ros::Publisher>& occupancy_grid_pub)
{
  // read the user-specified rate for publishing the gridmap
  int rate;
  nh.param<int>(loc_glob_field + "/publish_rate", rate, 1);

  // initialise the gridmap publisher
  gridmap_pub = nh.advertise<grid_map_msgs::GridMap>(loc_glob_field + "/grid_map", 10, true);

  // initialise the occupancy grid publisher
  for (const std::string& layer_name : gridmap_ptr->getLayers())
  {
    std::string topic_name(loc_glob_field + "/og_" + layer_name + "_layer");
    occupancy_grid_pub[layer_name] = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, 1, true);
  }

  // initialise timer to publish the gridmap at specified frequency
  return ros::Rate(rate);
}

GridMapNavigationServer::GridMapNavigationServer(const TFPtr& tf_listener_ptr)
  : AbstractNavigationServer(tf_listener_ptr)
  , recovery_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridMapRecovery")
  , controller_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridMapController")
  , planner_plugin_loader_("mbf_gridmap_core", "mbf_gridmap_core::GridMapPlanner")
  , local_gridmap_ptr_(new mbf_gridmap_core::Gridmap())
  , global_gridmap_ptr_(new mbf_gridmap_core::Gridmap())
  , local_gridmap_mtx_ptr_(new std::mutex())
  , global_gridmap_mtx_ptr_(new std::mutex())
{
  // read relevant parmeters from the parameter server and set frame id, and geometry of the local and global gridmap
  // independently NOTE: the geometry of global gridmap can be decided during global routing to encompass the expected
  // area of operation
  setUpGridMap("local_gridmap", private_nh_, *local_gridmap_ptr_, local_obstacle_threshold_);
  setUpGridMap("global_gridmap", private_nh_, *global_gridmap_ptr_, global_obstacle_threshold_);

  // read relevant parameters from the parameter server and set up gridmap publishers
  ros::Rate r1 = setUpGridMapPublisher("local_gridmap", private_nh_, local_gridmap_ptr_, local_gridmap_pub_,
                                       local_occupancy_grid_pub_);
  local_gridmap_pub_timer_ = private_nh_.createTimer(r1, &GridMapNavigationServer::publishLocalGridmap, this);
  ros::Rate r2 = setUpGridMapPublisher("global_gridmap", private_nh_, global_gridmap_ptr_, global_gridmap_pub_,
                                       global_occupancy_grid_pub_);
  global_gridmap_pub_timer_ = private_nh_.createTimer(r2, &GridMapNavigationServer::publishGlobalGridmap, this);

  // advertise services and current goal topic
  check_point_cost_srv_ =
      private_nh_.advertiseService("check_point_cost", &GridMapNavigationServer::callServiceCheckPointCost, this);
  check_pose_cost_srv_ =
      private_nh_.advertiseService("check_pose_cost", &GridMapNavigationServer::callServiceCheckPoseCost, this);
  check_path_cost_srv_ =
      private_nh_.advertiseService("check_path_cost", &GridMapNavigationServer::callServiceCheckPathCost, this);
  clear_gridmaps_srv_ =
      private_nh_.advertiseService("clear_costmaps", &GridMapNavigationServer::callServiceClearGridmaps, this);

  // initialize all plugins
  initializeServerComponents();

  // start all action servers
  startActionServers();
}

GridMapNavigationServer::~GridMapNavigationServer()
{
  // remove every plugin before its classLoader goes out of scope.
  controller_plugin_manager_.clearPlugins();
  planner_plugin_manager_.clearPlugins();
  recovery_plugin_manager_.clearPlugins();

  action_server_recovery_ptr_.reset();
  action_server_exe_path_ptr_.reset();
  action_server_get_path_ptr_.reset();
  action_server_move_base_ptr_.reset();
}

mbf_abstract_core::AbstractPlanner::Ptr GridMapNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  planner_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractPlanner>(
      planner_plugin_loader_.createInstance(planner_type));
  std::string planner_name = planner_plugin_loader_.getName(planner_type);
  ROS_DEBUG_STREAM("mbf_gridmap_core-based planner plugin " << planner_name << " loaded.");

  return planner_ptr;
}

bool GridMapNavigationServer::initializePlannerPlugin(const std::string& name,
                                                      const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr)
{
  mbf_gridmap_core::GridMapPlanner::Ptr gridmap_planner_ptr =
      boost::static_pointer_cast<mbf_gridmap_core::GridMapPlanner>(planner_ptr);
  ROS_DEBUG_STREAM("Initialize planner \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    ROS_FATAL_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!local_gridmap_ptr_)
  {
    ROS_FATAL_STREAM("The local gridmap pointer has not been initialized!");
    return false;
  }

  if (!global_gridmap_ptr_)
  {
    ROS_FATAL_STREAM("The global gridmap pointer has not been initialized!");
    return false;
  }

  if (!local_gridmap_mtx_ptr_)
  {
    ROS_FATAL_STREAM("The local gridmap mutex pointer has not been initialized!");
    return false;
  }

  if (!global_gridmap_mtx_ptr_)
  {
    ROS_FATAL_STREAM("The global gridmap mutex pointer has not been initialized!");
    return false;
  }

  gridmap_planner_ptr->initialize(name, tf_listener_ptr_, local_gridmap_ptr_, global_gridmap_ptr_,
                                  local_gridmap_mtx_ptr_, global_gridmap_mtx_ptr_);
  ROS_DEBUG_STREAM("Planner plugin \"" << name << "\" initialized.");

  return true;
}

mbf_abstract_core::AbstractController::Ptr
GridMapNavigationServer::loadControllerPlugin(const std::string& controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  controller_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractController>(
      controller_plugin_loader_.createInstance(controller_type));
  std::string controller_name = controller_plugin_loader_.getName(controller_type);
  ROS_DEBUG_STREAM("mbf_gridmap_core-based controller plugin " << controller_name << " loaded.");

  return controller_ptr;
}

bool GridMapNavigationServer::initializeControllerPlugin(
    const std::string& name, const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  mbf_gridmap_core::GridMapController::Ptr gridmap_controller_ptr =
      boost::static_pointer_cast<mbf_gridmap_core::GridMapController>(controller_ptr);

  ROS_DEBUG_STREAM("Initialize controller \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    ROS_FATAL_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!local_gridmap_ptr_)
  {
    ROS_FATAL_STREAM("The local gridmap pointer has not been initialized!");
    return false;
  }

  if (!global_gridmap_ptr_)
  {
    ROS_FATAL_STREAM("The global gridmap pointer has not been initialized!");
    return false;
  }

  if (!local_gridmap_mtx_ptr_)
  {
    ROS_FATAL_STREAM("The local gridmap mutex pointer has not been initialized!");
    return false;
  }

  if (!global_gridmap_mtx_ptr_)
  {
    ROS_FATAL_STREAM("The global gridmap mutex pointer has not been initialized!");
    return false;
  }

  gridmap_controller_ptr->initialize(name, tf_listener_ptr_, local_gridmap_ptr_, global_gridmap_ptr_,
                                     local_gridmap_mtx_ptr_, global_gridmap_mtx_ptr_);
  ROS_DEBUG_STREAM("Controller plugin \"" << name << "\" initialized.");

  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr GridMapNavigationServer::loadRecoveryPlugin(const std::string& recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;
  recovery_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractRecovery>(
      recovery_plugin_loader_.createInstance(recovery_type));
  std::string recovery_name = recovery_plugin_loader_.getName(recovery_type);
  ROS_DEBUG_STREAM("mbf_gridmap_core-based recovery behavior plugin " << recovery_name << " loaded.");

  return recovery_ptr;
}

bool GridMapNavigationServer::initializeRecoveryPlugin(const std::string& name,
                                                       const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  mbf_gridmap_core::GridMapRecovery::Ptr recovery_behavior_ptr =
      boost::static_pointer_cast<mbf_gridmap_core::GridMapRecovery>(behavior_ptr);

  ROS_DEBUG_STREAM("Initialize recovery behavior \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    ROS_FATAL_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!local_gridmap_ptr_)
  {
    ROS_FATAL_STREAM("The local gridmap pointer has not been initialized!");
    return false;
  }

  if (!global_gridmap_ptr_)
  {
    ROS_FATAL_STREAM("The global gridmap pointer has not been initialized!");
    return false;
  }

  if (!local_gridmap_mtx_ptr_)
  {
    ROS_FATAL_STREAM("The local gridmap mutex pointer has not been initialized!");
    return false;
  }

  if (!global_gridmap_mtx_ptr_)
  {
    ROS_FATAL_STREAM("The global gridmap mutex pointer has not been initialized!");
    return false;
  }

  recovery_behavior_ptr->initialize(name, tf_listener_ptr_, local_gridmap_ptr_, global_gridmap_ptr_,
                                    local_gridmap_mtx_ptr_, global_gridmap_mtx_ptr_);
  ROS_DEBUG_STREAM("Recovery behavior plugin \"" << name << "\" initialized.");

  return true;
}

void GridMapNavigationServer::publishLocalGridmap(const ros::TimerEvent& ev)
{
  std::lock_guard<std::mutex> guard(*local_gridmap_mtx_ptr_);
  publishGridMap("local_gridmap", local_gridmap_ptr_, local_gridmap_pub_, local_occupancy_grid_pub_,
                 local_obstacle_threshold_);
}

void GridMapNavigationServer::publishGlobalGridmap(const ros::TimerEvent& ev)
{
  std::lock_guard<std::mutex> guard(*global_gridmap_mtx_ptr_);
  publishGridMap("global_gridmap", global_gridmap_ptr_, global_gridmap_pub_, global_occupancy_grid_pub_,
                 global_obstacle_threshold_);
}

void GridMapNavigationServer::publishGridMap(const std::string& loc_glob_field,
                                             const mbf_gridmap_core::GridmapSPtr& gridmap_ptr,
                                             const ros::Publisher& gridmap_pub,
                                             std::map<std::string, ros::Publisher>& occupancy_grid_pub,
                                             double& obstacle_threshold)
{
  // publish the gridmap
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(*gridmap_ptr, message);
  gridmap_pub.publish(message);

  // publish each layer of gridmap as an occupancy grid
  for (const std::string& layer_name : gridmap_ptr->getLayers())
  {
    if (!occupancy_grid_pub.count(layer_name))
    {
      ROS_INFO_STREAM("Detected new layer: " << layer_name << " for " << loc_glob_field
                                             << " and adding occupancy grid for it");
      std::string topic_name(loc_glob_field + "/og_" + layer_name + "_layer");
      occupancy_grid_pub[layer_name] = private_nh_.advertise<nav_msgs::OccupancyGrid>(topic_name, 1, true);
    }
    nav_msgs::OccupancyGrid occupancyGrid;
    grid_map::GridMapRosConverter::toOccupancyGrid(*gridmap_ptr, layer_name, 0, obstacle_threshold, occupancyGrid);
    occupancy_grid_pub[layer_name].publish(occupancyGrid);
  }
}

bool GridMapNavigationServer::callServiceCheckPointCost(mbf_msgs::CheckPoint::Request& request,
                                                        mbf_msgs::CheckPoint::Response& response)
{
  // TODO
}

bool GridMapNavigationServer::callServiceCheckPoseCost(mbf_msgs::CheckPose::Request& request,
                                                       mbf_msgs::CheckPose::Response& response)
{
  // TODO
}

bool GridMapNavigationServer::callServiceCheckPathCost(mbf_msgs::CheckPath::Request& request,
                                                       mbf_msgs::CheckPath::Response& response)
{
  // TODO
}

bool GridMapNavigationServer::callServiceClearGridmaps(std_srvs::Empty::Request& request,
                                                       std_srvs::Empty::Response& response)
{
  // iterate through all the layers of the costmaps and flush the cost to zero
  std::lock_guard<std::mutex> local_guard(*local_gridmap_mtx_ptr_);
  for (const std::string& layer_name : local_gridmap_ptr_->getLayers())
  {
    grid_map::Matrix& data = local_gridmap_ptr_->get(layer_name);
    data.setConstant(0.0);
  }
  std::lock_guard<std::mutex> global_guard(*global_gridmap_mtx_ptr_);
  for (const std::string& layer_name : global_gridmap_ptr_->getLayers())
  {
    grid_map::Matrix& data = global_gridmap_ptr_->get(layer_name);
    data.setConstant(0.0);
  }

  return true;
}
}  // namespace mbf_gridmap_nav
