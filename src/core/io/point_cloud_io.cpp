
#include "point_cloud_io.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <algorithm>
#include <filesystem>

namespace pointcloud::io {

std::string getFileExtension(const std::string& file_path) 
{
	std::filesystem::path path(file_path);
	std::string ext = path.extension().string();
	if (!ext.empty() && ext[0] == '.') {
		ext = ext.substr(1);
	}
	std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
	return ext;
}

PointCloudStats calculateStats(PointCloudConstPtr cloud)
{
	PointCloudStats stats{};

	if (!cloud || cloud->empty()) {
		return stats;
	}

	stats.point_count = cloud->size();

	PointT min_pt, max_pt;
	pcl::getMinMax3D(*cloud, min_pt, max_pt);

	stats.min_x = min_pt.x;
	stats.max_x = max_pt.x;
	stats.min_y = min_pt.y;
	stats.max_y = max_pt.y;
	stats.min_z = min_pt.z;
	stats.max_z = max_pt.z;

	return stats;
}

LoadResult loadPointCloud(const std::string& file_path) 
{
	LoadResult result{};
	result.cloud = std::make_shared<PointCloud>();
	result.success = false;

	if (!std::filesystem::exists(file_path)) {
		result.error_message = "File not found: " + file_path;
		return result;
	}

	std::string ext = getFileExtension(file_path);
	int load_status = -1;

	if (ext == "pcd") {
		load_status = pcl::io::loadPCDFile<PointT>(file_path, *result.cloud);
	} else if (ext == "ply") {
		load_status = pcl::io::loadPLYFile<PointT>(file_path, *result.cloud);
	} else {
		result.error_message = "Unsupported file format: " + ext;
		return result;
	}

	if (load_status == -1) {
		result.error_message = "Failed to load file: " + file_path;
		return result;
	}

	result.stats = calculateStats(result.cloud);
	result.success = true;
	return result;
}

bool savePointCloudPCD(const std::string& file_path,
					   PointCloudConstPtr cloud,
					   bool binary)                       
{
	if (!cloud) return false;
	return pcl::io::savePCDFile(file_path, *cloud, binary) == 0;
}

bool savePointCloudPLY(const std::string& file_path,
					   PointCloudConstPtr cloud,
					   bool binary) 
{
	if (!cloud) return false;
	return pcl::io::savePLYFile(file_path, *cloud, binary) == 0;
}

} // namespace pointcloud::io

