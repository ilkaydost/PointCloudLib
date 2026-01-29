#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pointcloud {

// Default point type with color support
using PointT = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

// Point cloud statistics
struct PointCloudStats {
    size_t point_count = 0;
    float min_x = 0.f, max_x = 0.f;
    float min_y = 0.f, max_y = 0.f;
    float min_z = 0.f, max_z = 0.f;
};

} // namespace pointcloud
