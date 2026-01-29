#pragma once

#include <nlohmann/json.hpp>
#include "core/types/point_types.hpp"
#include "core/filters/filters.hpp"

namespace pointcloud::server {

using json = nlohmann::json;

// Serialize PointCloudStats to JSON
inline json statsToJson(const PointCloudStats& stats) {
    return {
        {"pointCount", stats.point_count},
        {"bounds", {
            {"minX", stats.min_x},
            {"maxX", stats.max_x},
            {"minY", stats.min_y},
            {"maxY", stats.max_y},
            {"minZ", stats.min_z},
            {"maxZ", stats.max_z}
        }}
    };
}

// Serialize point cloud to JSON array (for small clouds)
inline json cloudToJson(PointCloudConstPtr cloud, bool include_color = true) {
    json points = json::array();
    if (!cloud) return points;
    for (const auto& pt : *cloud) {
        json point = { {"x", pt.x}, {"y", pt.y}, {"z", pt.z} };
        if (include_color) {
            point["r"] = pt.r;
            point["g"] = pt.g;
            point["b"] = pt.b;
        }
        points.push_back(point);
    }
    return points;
}

// Serialize point cloud to flat Float32 array (for WebGL)
inline std::vector<float> cloudToFloatArray(PointCloudConstPtr cloud) {
    std::vector<float> positions;
    if (!cloud) return positions;
    positions.reserve(cloud->size() * 3);
    for (const auto& pt : *cloud) {
        positions.push_back(pt.x);
        positions.push_back(pt.y);
        positions.push_back(pt.z);
    }
    return positions;
}

// Serialize colors to flat Uint8 array
inline std::vector<uint8_t> cloudToColorArray(PointCloudConstPtr cloud) {
    std::vector<uint8_t> colors;
    if (!cloud) return colors;
    colors.reserve(cloud->size() * 3);
    for (const auto& pt : *cloud) {
        colors.push_back(pt.r);
        colors.push_back(pt.g);
        colors.push_back(pt.b);
    }
    return colors;
}

// Parse PassThroughConfig from JSON
inline filters::PassThroughConfig parsePassThroughConfig(const json& j) {
    filters::PassThroughConfig config;
    if (j.contains("fieldName")) config.field_name = j["fieldName"].get<std::string>();
    if (j.contains("minLimit")) config.min_limit = j["minLimit"].get<float>();
    if (j.contains("maxLimit")) config.max_limit = j["maxLimit"].get<float>();
    if (j.contains("negative")) config.negative = j["negative"].get<bool>();
    return config;
}

// Parse VoxelGridConfig from JSON
inline filters::VoxelGridConfig parseVoxelGridConfig(const json& j) {
    filters::VoxelGridConfig config;
    if (j.contains("leafSize")) {
        float size = j["leafSize"].get<float>();
        config.leaf_size_x = size;
        config.leaf_size_y = size;
        config.leaf_size_z = size;
    }
    if (j.contains("leafSizeX")) config.leaf_size_x = j["leafSizeX"].get<float>();
    if (j.contains("leafSizeY")) config.leaf_size_y = j["leafSizeY"].get<float>();
    if (j.contains("leafSizeZ")) config.leaf_size_z = j["leafSizeZ"].get<float>();
    return config;
}

} // namespace pointcloud::server
