#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>

// save diagnostic state
#pragma GCC diagnostic push 
// turn off the specific warning. Can also use "-Wall"
#pragma GCC diagnostic ignored "-Wpedantic"
#include <pcl/segmentation/extract_clusters.h>
// turn the warnings back on
#pragma GCC diagnostic pop
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/impl/moment_of_inertia_estimation.hpp>

#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>

#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/sample_consensus/sac.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>

#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>

// save diagnostic state
#pragma GCC diagnostic push 
// turn off the specific warning. Can also use "-Wall"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ouster_ros/point.h>
// turn the warnings back on
#pragma GCC diagnostic pop
