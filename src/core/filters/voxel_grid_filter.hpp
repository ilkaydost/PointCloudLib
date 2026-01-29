#pragma once

#include "core/types/point_types.hpp"

namespace pointcloud::filters {

/**
 * @brief VoxelGrid filter configuration
 */
struct VoxelGridConfig {
    float leaf_size_x = 0.01f;
    float leaf_size_y = 0.01f;
    float leaf_size_z = 0.01f;
};

/**
 * @brief VoxelGrid filter - downsamples point cloud using voxel grid
 */
class VoxelGridFilter {
public:
    VoxelGridFilter() = default;
    explicit VoxelGridFilter(const VoxelGridConfig& config);
    explicit VoxelGridFilter(float leaf_size);  // Uniform leaf size
    
    void setConfig(const VoxelGridConfig& config);
    void setLeafSize(float leaf_size);
    VoxelGridConfig getConfig() const;
    
    /**
     * @brief Apply filter to point cloud
     * @param input Input point cloud
     * @return Downsampled point cloud
     */
    PointCloudPtr apply(PointCloudConstPtr input) const;
    
private:
    VoxelGridConfig config_;
};

} // namespace pointcloud::filters
