#include "pass_through_filter.hpp"
#include <pcl/filters/passthrough.h>

namespace pointcloud::filters {

PassThroughFilter::PassThroughFilter(const PassThroughConfig& config)
    : m_config_(config) {}

void PassThroughFilter::setConfig(const PassThroughConfig& config) {
    m_config_ = config;
}

PassThroughConfig PassThroughFilter::getConfig() const {
    return m_config_;
}

PointCloudPtr PassThroughFilter::apply(PointCloudConstPtr input) const {
    if (!input || input->empty()) {
        return std::make_shared<PointCloud>();
    }
    
    auto output = std::make_shared<PointCloud>();
    
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(input);
    filter.setFilterFieldName(m_config_.field_name);
    filter.setFilterLimits(m_config_.min_limit, m_config_.max_limit);
    filter.setNegative(m_config_.negative);
    filter.filter(*output);
    
    return output;
}

} // namespace pointcloud::filters
