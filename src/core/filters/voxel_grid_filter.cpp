#include "voxel_grid_filter.hpp"
#include <pcl/filters/voxel_grid.h>

namespace pointcloud::filters {

VoxelGridFilter::VoxelGridFilter(const VoxelGridConfig& config)
    : m_config_(config) {}

VoxelGridFilter::VoxelGridFilter(float leaf_size) {
    m_config_.leaf_size_x = leaf_size;
    m_config_.leaf_size_y = leaf_size;
    m_config_.leaf_size_z = leaf_size;
}

void VoxelGridFilter::setConfig(const VoxelGridConfig& config) {
    m_config_ = config;
}

void VoxelGridFilter::setLeafSize(float leaf_size) {
    m_config_.leaf_size_x = leaf_size;
    m_config_.leaf_size_y = leaf_size;
    m_config_.leaf_size_z = leaf_size;
}

VoxelGridConfig VoxelGridFilter::getConfig() const {
    return m_config_;
}

PointCloudPtr VoxelGridFilter::apply(PointCloudConstPtr input) const {
    if (!input || input->empty()) {
        return std::make_shared<PointCloud>();
    }
    
    auto output = std::make_shared<PointCloud>();
    
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(m_config_.leaf_size_x, m_config_.leaf_size_y, m_config_.leaf_size_z);
    filter.filter(*output);
    
    return output;
}

} // namespace pointcloud::filters
