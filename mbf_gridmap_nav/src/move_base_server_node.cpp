#include "mbf_gridmap_nav/gridmap_navigation_server.h"
#include <mbf_utility/types.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mbf_gridmap_nav_server");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double cache_time;
  private_nh.param("tf_cache_time", cache_time, 10.0);
#ifdef USE_OLD_TF
  TFPtr tf_listener_ptr(new TF(nh, ros::Duration(cache_time), true));
#else
  TFPtr tf_listener_ptr(new TF(ros::Duration(cache_time)));
  tf2_ros::TransformListener tf_listener(*tf_listener_ptr);
#endif
  mbf_gridmap_nav::GridMapNavigationServer::Ptr gridmap_nav_srv_ptr =
      boost::make_shared<mbf_gridmap_nav::GridMapNavigationServer>(tf_listener_ptr);
  ros::spin();
  return EXIT_SUCCESS;
}
