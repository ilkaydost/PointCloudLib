#pragma once

#include "core/types/point_types.hpp"
#include <vector>

namespace pointcloud::segmentation {

/**
 * @brief RANSAC plane segmentation configuration
 */
struct RansacPlaneConfig {
    double distance_threshold = 0.01;  // Max distance from point to plane (meters)
    int max_iterations = 1000;         // Maximum RANSAC iterations
    bool optimize_coefficients = true; // Refine plane coefficients after finding inliers
    bool extract_inliers = true;       // If true, return plane points; if false, return non-plane points
};

/**
 * @brief Result of RANSAC plane segmentation
 */
struct RansacPlaneResult {
    PointCloudPtr plane_cloud;      // Points belonging to the plane
    PointCloudPtr remaining_cloud;  // Points not belonging to the plane
    std::vector<float> coefficients; // Plane coefficients [a, b, c, d] where ax + by + cz + d = 0
    size_t inlier_count = 0;
    bool success = false;
    std::string error_message;
};

/**
 * @brief RANSAC-based plane segmentation
 * 
 * Uses PCL's SACSegmentation to find the dominant plane in a point cloud
 * using the RANSAC (Random Sample Consensus) algorithm.
 */
class RansacPlaneSegmenter {
public:
    RansacPlaneSegmenter() = default;
    explicit RansacPlaneSegmenter(const RansacPlaneConfig& config);
    
    void setConfig(const RansacPlaneConfig& config);
    RansacPlaneConfig getConfig() const;
    
    /**
     * @brief Segment the dominant plane from the point cloud
     * @param input Input point cloud
     * @return Segmentation result with plane and remaining points
     */
    RansacPlaneResult segment(PointCloudConstPtr input) const;
    
    /**
     * @brief Apply segmentation and return either plane or remaining points
     * @param input Input point cloud
     * @return Points based on extract_inliers config (plane if true, remaining if false)
     */
    PointCloudPtr apply(PointCloudConstPtr input) const;
    
private:
    RansacPlaneConfig m_config_;
};

} // namespace pointcloud::segmentation
