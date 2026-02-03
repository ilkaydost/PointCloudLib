#include "core/types/point_types.hpp"  // Include first to get full PCL types
#include "http_server.hpp"
#include "serialization.hpp"
#include "core/io/point_cloud_io.hpp"
#include "core/filters/filters.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace pointcloud::server {

HttpServer::HttpServer(int port) : m_port_(port) {
	setupCORS();
	setupRoutes();
}

HttpServer::~HttpServer() {
	stop();
}

void HttpServer::setupCORS() {
	m_server_.set_default_headers({
		{"Access-Control-Allow-Origin", "*"},
		{"Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS"},
		{"Access-Control-Allow-Headers", "Content-Type, Authorization"}
	});
    
	// Handle preflight requests
	m_server_.Options(".*", [](const httplib::Request& req, httplib::Response& res) {
		res.status = 204;
	});
}

void HttpServer::setupRoutes() {
	// Health check
	m_server_.Get("/api/health", [](const httplib::Request& req, httplib::Response& res) {
		res.set_content(R"({"status":"ok"})", "application/json");
	});

	// Load point cloud from file path
	m_server_.Post("/api/load", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto body = json::parse(req.body);
			std::string file_path = body["path"].get<std::string>();
            
			auto result = io::loadPointCloud(file_path);
            
			if (!result.success) {
				res.status = 400;
				res.set_content(json{{"error", result.error_message}}.dump(), "application/json");
				return;
			}

			setCurrentCloud(result.cloud);

			json response = {
				{"success", true},
				{"stats", statsToJson(result.stats)}
			};
			res.set_content(response.dump(), "application/json");
            
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// Upload point cloud file (multipart/form-data)
	m_server_.Post("/api/upload", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			if (!req.has_file("file")) {
				res.status = 400;
				res.set_content(R"({"error":"No file uploaded"})", "application/json");
				return;
			}
			
			const auto& file = req.get_file_value("file");
			std::string filename = file.filename;
			std::string content = file.content;
			
			// Get file extension
			std::string ext = io::getFileExtension(filename);
			if (ext != "pcd" && ext != "ply") {
				res.status = 400;
				res.set_content(json{{"error", "Unsupported file format: " + ext}}.dump(), "application/json");
				return;
			}
			
			// Create temporary file
			std::string temp_dir = std::filesystem::temp_directory_path().string();
			std::string temp_path = temp_dir + "/pointcloud_upload_" + filename;
			
			// Write content to temp file
			std::ofstream ofs(temp_path, std::ios::binary);
			if (!ofs) {
				res.status = 500;
				res.set_content(R"({"error":"Failed to create temporary file"})", "application/json");
				return;
			}
			ofs.write(content.data(), content.size());
			ofs.close();
			
			// Load point cloud from temp file
			auto result = io::loadPointCloud(temp_path);
			
			// Clean up temp file
			std::filesystem::remove(temp_path);
			
			if (!result.success) {
				res.status = 400;
				res.set_content(json{{"error", result.error_message}}.dump(), "application/json");
				return;
			}
			
			setCurrentCloud(result.cloud);
			
			json response = {
				{"success", true},
				{"stats", statsToJson(result.stats)}
			};
			res.set_content(response.dump(), "application/json");
			
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// Get current point cloud stats
	m_server_.Get("/api/stats", [this](const httplib::Request& req, httplib::Response& res) {
		auto cloud = getCurrentCloud();
        
		if (!cloud || cloud->empty()) {
			res.status = 404;
			res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
			return;
		}
        
		auto stats = io::calculateStats(cloud);
		res.set_content(json{{"stats", statsToJson(stats)}}.dump(), "application/json");
	});

	// Get point cloud data as JSON
	m_server_.Get("/api/points", [this](const httplib::Request& req, httplib::Response& res) {
		auto cloud = getCurrentCloud();
        
		if (!cloud || cloud->empty()) {
			res.status = 404;
			res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
			return;
		}
        
		// Check for binary format request
		std::string format = "json";
		if (req.has_param("format")) {
			format = req.get_param_value("format");
		}
        
		if (format == "binary") {
			auto positions = cloudToFloatArray(cloud);
			auto colors = cloudToColorArray(cloud);
            
			// Create binary response with header
			json header = {
				{"pointCount", cloud->size()},
				{"positionsBytes", positions.size() * sizeof(float)},
				{"colorsBytes", colors.size()}
			};
            
			res.set_content(header.dump(), "application/json");
		} else {
			json response = {
				{"pointCount", cloud->size()},
				{"points", cloudToJson(cloud)}
			};
			res.set_content(response.dump(), "application/json");
		}
	});

	// Get positions as binary Float32Array
	m_server_.Get("/api/points/positions", [this](const httplib::Request& req, httplib::Response& res) {
		auto cloud = getCurrentCloud();
        
		if (!cloud || cloud->empty()) {
			res.status = 404;
			res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
			return;
		}
        
		auto positions = cloudToFloatArray(cloud);
		res.set_content(
			std::string(reinterpret_cast<const char*>(positions.data()), 
					   positions.size() * sizeof(float)),
			"application/octet-stream"
		);
	});

	// Get colors as binary Uint8Array
	m_server_.Get("/api/points/colors", [this](const httplib::Request& req, httplib::Response& res) {
		auto cloud = getCurrentCloud();
        
		if (!cloud || cloud->empty()) {
			res.status = 404;
			res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
			return;
		}
        
		auto colors = cloudToColorArray(cloud);
		res.set_content(
			std::string(reinterpret_cast<const char*>(colors.data()), colors.size()),
			"application/octet-stream"
		);
	});

	// Apply filter
	m_server_.Post("/api/filter", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto cloud = getCurrentCloud();
            
			if (!cloud || cloud->empty()) {
				res.status = 404;
				res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
				return;
			}
            
			auto body = json::parse(req.body);
			std::string filter_type = body["type"].get<std::string>();
            
			PointCloudPtr result;
            
			if (filter_type == "passthrough") {
				auto config = parsePassThroughConfig(body["config"]);
				filters::PassThroughFilter filter(config);
				result = filter.apply(cloud);
			} else if (filter_type == "voxelgrid") {
				auto config = parseVoxelGridConfig(body["config"]);
				filters::VoxelGridFilter filter(config);
				result = filter.apply(cloud);
			} else {
				res.status = 400;
				res.set_content(json{{"error", "Unknown filter type: " + filter_type}}.dump(), 
							   "application/json");
				return;
			}
            
			setCurrentCloud(result);
            
			auto stats = io::calculateStats(result);
			json response = {
				{"success", true},
				{"stats", statsToJson(stats)}
			};
			res.set_content(response.dump(), "application/json");
            
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});
}

void HttpServer::start() {
	m_running_ = true;
	std::cout << "Starting PointCloudLib server on port " << m_port_ << std::endl;
	m_server_.listen("0.0.0.0", m_port_);
}

void HttpServer::stop() {
	if (m_running_) {
		m_running_ = false;
		m_server_.stop();
	}
}

bool HttpServer::isRunning() const {
	return m_running_;
}

PointCloudPtr HttpServer::getCurrentCloud() const {
	std::lock_guard<std::mutex> lock(m_cloud_mutex_);
	return m_p_current_cloud_;
}

void HttpServer::setCurrentCloud(PointCloudPtr cloud) {
	std::lock_guard<std::mutex> lock(m_cloud_mutex_);
	m_p_current_cloud_ = cloud;
}

} // namespace pointcloud::server

