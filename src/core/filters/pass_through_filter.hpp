#pragma once

#include "core/types/point_types.hpp"
#include <string>

namespace pointcloud::filters {

/**
 * @brief PassThrough filter configuration
 */
struct PassThroughConfig {
    std::string field_name = "z";  // "x", "y", or "z"
    float min_limit = 0.0f;
    float max_limit = 1.0f;
    bool negative = false;  // If true, keeps points OUTSIDE the range
};

/**
 * @brief PassThrough filter - removes points outside specified bounds
 */
class PassThroughFilter {
public:
    PassThroughFilter() = default;
    explicit PassThroughFilter(const PassThroughConfig& config);
    
    void setConfig(const PassThroughConfig& config);
    PassThroughConfig getConfig() const;
    
    /**
     * @brief Apply filter to point cloud
     * @param input Input point cloud
     * @return Filtered point cloud
     */
    PointCloudPtr apply(PointCloudConstPtr input) const;
    
private:
    PassThroughConfig m_config_;
};

} // namespace pointcloud::filters
