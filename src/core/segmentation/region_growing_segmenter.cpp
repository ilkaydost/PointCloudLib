#include "region_growing_segmenter.hpp"

#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <boost/make_shared.hpp>
#include <random>
#include <iostream>

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
    
    // Extract individual clusters and create colored cloud
    result.clusters.reserve(clusters.size());
    result.colored_cloud->resize(input->size());
    
    // Initialize colored cloud with input positions and GRAY for unclustered points
    for (size_t i = 0; i < input->size(); ++i) {
        result.colored_cloud->points[i] = input->points[i];
        result.colored_cloud->points[i].r = 128;  // Gray for unclustered points
        result.colored_cloud->points[i].g = 128;
        result.colored_cloud->points[i].b = 128;
    }
    
    // Random color generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> color_dist(50, 255);
    
    // Assign unique color to each cluster
    for (size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
        const auto& cluster_indices = clusters[cluster_idx];
        
        // Generate random color for this cluster
        uint8_t r = static_cast<uint8_t>(color_dist(gen));
        uint8_t g = static_cast<uint8_t>(color_dist(gen));
        uint8_t b = static_cast<uint8_t>(color_dist(gen));
                
        // Create cluster cloud
        PointCloudPtr cluster_cloud = boost::make_shared<PointCloud>();
        cluster_cloud->reserve(cluster_indices.indices.size());
        
        // Assign color to all points in this cluster
        for (int idx : cluster_indices.indices) {
            result.colored_cloud->points[idx].r = r;
            result.colored_cloud->points[idx].g = g;
            result.colored_cloud->points[idx].b = b;
            cluster_cloud->push_back((*input)[idx]);
        }
        
        result.clusters.push_back(cluster_cloud);
    }
        
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
