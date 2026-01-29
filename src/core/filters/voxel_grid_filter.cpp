#include "voxel_grid_filter.hpp"
#include <pcl/filters/voxel_grid.h>

namespace pointcloud::filters {

VoxelGridFilter::VoxelGridFilter(const VoxelGridConfig& config)
    : config_(config) {}

VoxelGridFilter::VoxelGridFilter(float leaf_size) {
    config_.leaf_size_x = leaf_size;
    config_.leaf_size_y = leaf_size;
    config_.leaf_size_z = leaf_size;
}

void VoxelGridFilter::setConfig(const VoxelGridConfig& config) {
    config_ = config;
}

void VoxelGridFilter::setLeafSize(float leaf_size) {
    config_.leaf_size_x = leaf_size;
    config_.leaf_size_y = leaf_size;
    config_.leaf_size_z = leaf_size;
}

VoxelGridConfig VoxelGridFilter::getConfig() const {
    return config_;
}

PointCloudPtr VoxelGridFilter::apply(PointCloudConstPtr input) const {
    if (!input || input->empty()) {
        return std::make_shared<PointCloud>();
    }
    
    auto output = std::make_shared<PointCloud>();
    
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(config_.leaf_size_x, config_.leaf_size_y, config_.leaf_size_z);
    filter.filter(*output);
    
    return output;
}

} // namespace pointcloud::filters
