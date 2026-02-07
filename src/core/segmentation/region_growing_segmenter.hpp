#pragma once

#include "core/types/point_types.hpp"
#include <pcl/PointIndices.h>
#include <vector>

namespace pointcloud::segmentation {

/**
 * @brief Region growing segmentation configuration
 */
struct RegionGrowingConfig {
    // Smoothness constraint (angle threshold in degrees)
    float smoothness_threshold = 5.0f;  // Max angle between normals (degrees)
    
    // Curvature threshold for seed point selection
    float curvature_threshold = 1.0f;   // Max curvature for smooth regions
    
    // Cluster size constraints
    int min_cluster_size = 50;          // Minimum points in a cluster
    int max_cluster_size = 1000000;     // Maximum points in a cluster
    
    // Number of neighbors for region growing
    int number_of_neighbours = 30;      // KNN search parameter
    
    // Normal estimation parameters (used if normals not provided)
    int normal_k_search = 30;           // KNN for normal estimation
    
    // Residual threshold for planar regions
    float residual_threshold = 0.05f;   // Distance to fitted plane (meters)
};

/**
 * @brief Result of region growing segmentation
 */
struct RegionGrowingResult {
    std::vector<PointCloudPtr> clusters;  // Segmented clusters
    PointCloudPtr colored_cloud;          // Cloud with clusters colored differently
    std::vector<pcl::PointIndices> indices; // Indices of points in each cluster
    size_t num_clusters = 0;
    bool success = false;
    std::string error_message;
};

/**
 * @brief Surface-based region growing segmentation
 * 
 * Uses PCL's RegionGrowing algorithm to segment a point cloud into
 * smooth surface regions based on surface normals and curvature.
 * Points are grouped together if they have similar normals (smoothness constraint)
 * and low curvature (residual constraint).
 */
class RegionGrowingSegmenter {
public:
    RegionGrowingSegmenter() = default;
    explicit RegionGrowingSegmenter(const RegionGrowingConfig& config);
    
    void setConfig(const RegionGrowingConfig& config);
    RegionGrowingConfig getConfig() const;
    
    /**
     * @brief Segment the point cloud into smooth regions
     * @param input Input point cloud
     * @return Segmentation result with individual clusters
     */
    RegionGrowingResult segment(PointCloudConstPtr input) const;
    
    /**
     * @brief Apply region growing and return all clusters merged
     * @param input Input point cloud
     * @return Merged point cloud with all clusters (colored differently)
     */
    PointCloudPtr apply(PointCloudConstPtr input) const;

private:
    RegionGrowingConfig m_config_;
};

} // namespace pointcloud::segmentation
