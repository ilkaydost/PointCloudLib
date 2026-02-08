#pragma once

#include "core/types/point_types.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

namespace pointcloud::features {

struct NormalEstimatorConfig {
    // Use K nearest neighbors if > 0, otherwise use radius search when radius > 0
    int k_search = 30;
    float radius_search = 0.0f; // if > 0, radius search is used instead of k
};

struct NormalEstimatorResult {
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    bool success = false;
    std::string error_message;
};

class NormalEstimator {
public:
    NormalEstimator() = default;
    explicit NormalEstimator(const NormalEstimatorConfig& config);

    void setConfig(const NormalEstimatorConfig& config);
    NormalEstimatorConfig getConfig() const;

    /**
     * @brief Compute normals for the input cloud
     * @param input Input point cloud
     * @return NormalEstimatorResult containing normals (same ordering as input)
     */
    NormalEstimatorResult compute(PointCloudConstPtr input) const;

private:
    NormalEstimatorConfig m_config_;
};

} // namespace pointcloud::features
