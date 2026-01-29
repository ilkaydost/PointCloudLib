#pragma once
// Placeholder - will be implemented in Step 2

#include <string>
#include <optional>
#include "core/types/point_types.hpp"

namespace pointcloud::io {

/**
 * @brief Result of a point cloud load operation
 */
struct LoadResult {
	PointCloudPtr cloud;
	PointCloudStats stats;
	bool success = false;
	std::string error_message;
};

/**
 * @brief Load a point cloud from PCD or PLY file
 * @param file_path Absolute path to the file
 * @return LoadResult containing the cloud and metadata
 */
LoadResult loadPointCloud(const std::string& file_path);

/**
 * @brief Save a point cloud to PCD file
 */
bool savePointCloudPCD(const std::string& file_path,
					   PointCloudConstPtr cloud,
					   bool binary = true);

/**
 * @brief Save a point cloud to PLY file
 */
bool savePointCloudPLY(const std::string& file_path,
					   PointCloudConstPtr cloud,
					   bool binary = true);

/**
 * @brief Calculate statistics for a point cloud
 */
PointCloudStats calculateStats(PointCloudConstPtr cloud);

/**
 * @brief Get file extension in lowercase
 */
std::string getFileExtension(const std::string& file_path);

} // namespace pointcloud::io
