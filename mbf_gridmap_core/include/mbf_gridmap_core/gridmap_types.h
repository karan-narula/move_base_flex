#ifndef MBF_GRIDMAP_CORE__GRIDMAP_TYPES_H_
#define MBF_GRIDMAP_CORE__GRIDMAP_TYPES_H_

#include <grid_map_core/grid_map_core.hpp>
#include <memory>

namespace mbf_gridmap_core
{
typedef grid_map::GridMap Gridmap;
typedef std::unique_ptr<grid_map::GridMap> GridmapUPtr;
typedef grid_map::GridMap* GridmapRPtr;
typedef std::shared_ptr<grid_map::GridMap> GridmapSPtr;
}  // namespace mbf_gridmap_core

#endif // MBF_GRIDMAP_CORE__GRIDMAP_TYPES_H_
