#include "normal_estimator.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <boost/make_shared.hpp>

namespace pointcloud::features {

NormalEstimator::NormalEstimator(const NormalEstimatorConfig& config)
    : m_config_(config) {}

void NormalEstimator::setConfig(const NormalEstimatorConfig& config) {
    m_config_ = config;
}

NormalEstimatorConfig NormalEstimator::getConfig() const {
    return m_config_;
}

NormalEstimatorResult NormalEstimator::compute(PointCloudConstPtr input) const {
    NormalEstimatorResult result;
    result.normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
    result.success = false;

    if (!input || input->empty()) {
        result.error_message = "Input cloud is empty";
        return result;
    }

    // Set up KD-tree
    pcl::search::KdTree<PointT>::Ptr tree = boost::make_shared<pcl::search::KdTree<PointT>>();

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(input);
    ne.setSearchMethod(tree);

    if (m_config_.radius_search > 0.0f) {
        ne.setRadiusSearch(m_config_.radius_search);
    } else if (m_config_.k_search > 0) {
        ne.setKSearch(m_config_.k_search);
    } else {
        // Default fallback
        ne.setKSearch(30);
    }

    try {
        ne.compute(*result.normals);
    } catch (const std::exception& e) {
        result.error_message = std::string("Normal estimation failed: ") + e.what();
        return result;
    }

    if (result.normals->empty()) {
        result.error_message = "Normal estimation produced empty normals";
        return result;
    }

    result.success = true;
    return result;
}

} // namespace pointcloud::features
