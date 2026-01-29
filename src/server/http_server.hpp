#pragma once

#include <httplib.h>
#include <memory>
#include <atomic>
#include <mutex>
#include "core/types/point_types.hpp"

namespace pointcloud::server {

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
    
	// Get the current loaded point cloud
	PointCloudPtr getCurrentCloud() const;
    
	// Set the current point cloud
	void setCurrentCloud(PointCloudPtr cloud);
    
private:
	void setupRoutes();
	void setupCORS();
    
	httplib::Server m_server_;
	int m_port_;
	std::atomic<bool> m_running_{false};
	PointCloudPtr m_p_current_cloud_;
	mutable std::mutex m_cloud_mutex_;
};

} // namespace pointcloud::server
