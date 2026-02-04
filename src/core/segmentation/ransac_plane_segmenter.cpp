#include "ransac_plane_segmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <boost/make_shared.hpp>

namespace pointcloud::segmentation {

RansacPlaneSegmenter::RansacPlaneSegmenter(const RansacPlaneConfig& config)
    : m_config_(config) {}

void RansacPlaneSegmenter::setConfig(const RansacPlaneConfig& config) {
    m_config_ = config;
}

RansacPlaneConfig RansacPlaneSegmenter::getConfig() const {
    return m_config_;
}

RansacPlaneResult RansacPlaneSegmenter::segment(PointCloudConstPtr input) const {
    RansacPlaneResult result;
    result.plane_cloud = boost::make_shared<PointCloud>();
    result.remaining_cloud = boost::make_shared<PointCloud>();
    result.success = false;
    
    if (!input || input->empty()) {
        result.error_message = "Input cloud is empty";
        return result;
    }
    
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(m_config_.optimize_coefficients);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(m_config_.max_iterations);
    seg.setDistanceThreshold(m_config_.distance_threshold);
    
    // Segment
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.empty()) {
        result.error_message = "No plane found in the point cloud";
        return result;
    }
    
    // Store plane coefficients
    result.coefficients.reserve(coefficients->values.size());
    for (const auto& val : coefficients->values) {
        result.coefficients.push_back(static_cast<float>(val));
    }
    result.inlier_count = inliers->indices.size();
    
    // Extract plane points (inliers)
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    
    // Extract inliers (plane points)
    extract.setNegative(false);
    extract.filter(*result.plane_cloud);
    
    // Extract outliers (remaining points)
    extract.setNegative(true);
    extract.filter(*result.remaining_cloud);
    
    result.success = true;
    return result;
}

PointCloudPtr RansacPlaneSegmenter::apply(PointCloudConstPtr input) const {
    auto result = segment(input);
    
    if (!result.success) {
        // Return empty cloud on failure
        return boost::make_shared<PointCloud>();
    }
    
    // Return plane points if extract_inliers is true, otherwise return remaining
    return m_config_.extract_inliers ? result.plane_cloud : result.remaining_cloud;
}

} // namespace pointcloud::segmentation
