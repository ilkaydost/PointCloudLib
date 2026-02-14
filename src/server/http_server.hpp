#pragma once

#include <httplib.h>
#include <memory>
#include <atomic>
#include <mutex>
#include <boost/shared_ptr.hpp>

// Forward declare PCL types to avoid including PCL/Eigen headers
namespace pcl {
template<typename PointT> class PointCloud;
struct PointXYZRGB;
}

namespace pcl_wrapper {
class ICPRegistration;
}

namespace pointcloud::server {

// Use opaque pointer type to avoid Eigen header issues
using PointCloudPtr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>;

class HttpServer {
public:
	HttpServer(int port = 5050);
	~HttpServer();
    
	// Start server (blocking)
	void start();
    
	// Stop server
	void stop();
    
	// Check if running
	bool isRunning() const;
    
	// Get the current loaded point cloud (defined in cpp file)
	PointCloudPtr getCurrentCloud() const;
    
	// Set the current point cloud (defined in cpp file)
	void setCurrentCloud(PointCloudPtr cloud);
    
private:
	void setupRoutes();
	void setupCORS();
    
	httplib::Server m_server_;
	int m_port_;
	std::atomic<bool> m_running_{false};
	PointCloudPtr m_p_current_cloud_;
	mutable std::mutex m_cloud_mutex_;
	
	// ICP registration state
	std::unique_ptr<pcl_wrapper::ICPRegistration> m_icp_;
	PointCloudPtr m_source_cloud_;
	PointCloudPtr m_target_cloud_;
	mutable std::mutex m_icp_mutex_;
};

} // namespace pointcloud::server
