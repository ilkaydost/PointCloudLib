#include "region_growing_segmenter.hpp"

#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <boost/make_shared.hpp>
#include <random>

namespace pointcloud::segmentation {

RegionGrowingSegmenter::RegionGrowingSegmenter(const RegionGrowingConfig& config)
    : m_config_(config) {}

void RegionGrowingSegmenter::setConfig(const RegionGrowingConfig& config) {
    m_config_ = config;
}

RegionGrowingConfig RegionGrowingSegmenter::getConfig() const {
    return m_config_;
}

RegionGrowingResult RegionGrowingSegmenter::segment(PointCloudConstPtr input) const {
    RegionGrowingResult result;
    result.colored_cloud = boost::make_shared<PointCloud>();
    result.success = false;
    
    if (!input || input->empty()) {
        result.error_message = "Input cloud is empty";
        return result;
    }
    
    // Estimate normals for the input cloud
    pcl::search::Search<PointT>::Ptr tree = boost::make_shared<pcl::search::KdTree<PointT>>();
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
    
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(input);
    normal_estimator.setKSearch(m_config_.normal_k_search);
    normal_estimator.compute(*normals);
    
    if (normals->empty()) {
        result.error_message = "Failed to estimate normals";
        return result;
    }
    
    // Create region growing segmentation object
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize(m_config_.min_cluster_size);
    reg.setMaxClusterSize(m_config_.max_cluster_size);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(m_config_.number_of_neighbours);
    reg.setInputCloud(input);
    reg.setInputNormals(normals);
    
    // Convert smoothness threshold from degrees to radians
    float smoothness_rad = m_config_.smoothness_threshold * M_PI / 180.0f;
    reg.setSmoothnessThreshold(smoothness_rad);
    reg.setCurvatureThreshold(m_config_.curvature_threshold);
    
    // Perform segmentation
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    
    if (clusters.empty()) {
        result.error_message = "No clusters found with current configuration";
        return result;
    }
    
    // Store cluster indices
    result.indices = clusters;
    result.num_clusters = clusters.size();
    
    // Extract individual clusters
    result.clusters.reserve(clusters.size());
    for (const auto& cluster_indices : clusters) {
        PointCloudPtr cluster_cloud = boost::make_shared<PointCloud>();
        cluster_cloud->reserve(cluster_indices.indices.size());
        
        for (int idx : cluster_indices.indices) {
            cluster_cloud->push_back((*input)[idx]);
        }
        
        result.clusters.push_back(cluster_cloud);
    }
    
    // Create colored cloud for visualization
    result.colored_cloud = reg.getColoredCloud();
    
    result.success = true;
    return result;
}

PointCloudPtr RegionGrowingSegmenter::apply(PointCloudConstPtr input) const {
    auto result = segment(input);
    
    if (!result.success) {
        // Return empty cloud on failure
        return boost::make_shared<PointCloud>();
    }
    
    // Return the colored cloud with all segments
    return result.colored_cloud;
}

} // namespace pointcloud::segmentation
