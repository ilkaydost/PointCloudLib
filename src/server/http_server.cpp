#include "core/types/point_types.hpp"  // Include first to get full PCL types
#include "http_server.hpp"
#include "serialization.hpp"
#include "core/io/point_cloud_io.hpp"
#include "core/filters/filters.hpp"
#include "core/segmentation/segmentation.hpp"
#include "core/features/normal_estimator.hpp"
#include "core/registration/icp_registration.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace pointcloud;

namespace pointcloud::server {

HttpServer::HttpServer(int port) : m_port_(port), m_icp_(std::make_unique<pcl_wrapper::ICPRegistration>()) {
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
		std::cout << "GET /api/points/colors: Sending " << colors.size() << " bytes for " 
		          << cloud->size() << " points" << std::endl;
		if (colors.size() >= 3) {
			std::cout << "First color: R=" << (int)colors[0] << " G=" << (int)colors[1] << " B=" << (int)colors[2] << std::endl;
		}
		
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

	// RANSAC plane segmentation
	m_server_.Post("/api/segment/ransac", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto cloud = getCurrentCloud();
            
			if (!cloud || cloud->empty()) {
				res.status = 404;
				res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
				return;
			}
            
			auto body = json::parse(req.body);
            
			// Parse RANSAC config
			segmentation::RansacPlaneConfig config;
			if (body.contains("distanceThreshold")) {
				config.distance_threshold = body["distanceThreshold"].get<double>();
			}
			if (body.contains("maxIterations")) {
				config.max_iterations = body["maxIterations"].get<int>();
			}
			if (body.contains("extractInliers")) {
				config.extract_inliers = body["extractInliers"].get<bool>();
			}
            
			segmentation::RansacPlaneSegmenter segmenter(config);
			auto seg_result = segmenter.segment(cloud);
            
			if (!seg_result.success) {
				res.status = 400;
				res.set_content(json{{"error", seg_result.error_message}}.dump(), "application/json");
				return;
			}
            
			// Set the result cloud based on extractInliers
			PointCloudPtr result_cloud = config.extract_inliers ? 
				seg_result.plane_cloud : seg_result.remaining_cloud;
			setCurrentCloud(result_cloud);
            
			auto stats = io::calculateStats(result_cloud);
			json response = {
				{"success", true},
				{"stats", statsToJson(stats)},
				{"planeCoefficients", seg_result.coefficients},
				{"inlierCount", seg_result.inlier_count},
				{"planePoints", seg_result.plane_cloud->size()},
				{"remainingPoints", seg_result.remaining_cloud->size()}
			};
			res.set_content(response.dump(), "application/json");
            
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// Region growing segmentation
	m_server_.Post("/api/segment/regiongrowing", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto cloud = getCurrentCloud();
            
			if (!cloud || cloud->empty()) {
				res.status = 404;
				res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
				return;
			}
            
			auto body = json::parse(req.body);
            
			// Parse Region Growing config
			segmentation::RegionGrowingConfig config;
			if (body.contains("smoothnessThreshold")) {
				config.smoothness_threshold = body["smoothnessThreshold"].get<float>();
			}
			if (body.contains("curvatureThreshold")) {
				config.curvature_threshold = body["curvatureThreshold"].get<float>();
			}
			if (body.contains("minClusterSize")) {
				config.min_cluster_size = body["minClusterSize"].get<int>();
			}
			if (body.contains("maxClusterSize")) {
				config.max_cluster_size = body["maxClusterSize"].get<int>();
			}
			if (body.contains("numberOfNeighbours")) {
				config.number_of_neighbours = body["numberOfNeighbours"].get<int>();
			}
			if (body.contains("normalKSearch")) {
				config.normal_k_search = body["normalKSearch"].get<int>();
			}
            
			segmentation::RegionGrowingSegmenter segmenter(config);
			auto seg_result = segmenter.segment(cloud);
            
			if (!seg_result.success) {
				res.status = 400;
				res.set_content(json{{"error", seg_result.error_message}}.dump(), "application/json");
				return;
			}
            
			// Set the colored cloud as current
			setCurrentCloud(seg_result.colored_cloud);
                        
			auto stats = io::calculateStats(seg_result.colored_cloud);
			
			// Build cluster info array
			json cluster_info = json::array();
			for (size_t i = 0; i < seg_result.clusters.size(); ++i) {
				cluster_info.push_back({
					{"id", i},
					{"pointCount", seg_result.clusters[i]->size()}
				});
			}
			
			json response = {
				{"success", true},
				{"stats", statsToJson(stats)},
				{"numClusters", seg_result.num_clusters},
				{"clusters", cluster_info}
			};
			res.set_content(response.dump(), "application/json");
            
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// Normal estimation
	m_server_.Post("/api/features/normals", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto cloud = getCurrentCloud();
			if (!cloud || cloud->empty()) {
				res.status = 404;
				res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
				return;
			}

			auto body = json::parse(req.body);

			pointcloud::features::NormalEstimatorConfig config;
			if (body.contains("kSearch")) config.k_search = body["kSearch"].get<int>();
			if (body.contains("radiusSearch")) config.radius_search = body["radiusSearch"].get<float>();

			pointcloud::features::NormalEstimator estimator(config);
			auto ne_result = estimator.compute(cloud);
			if (!ne_result.success) {
				res.status = 400;
				res.set_content(json{{"error", ne_result.error_message}}.dump(), "application/json");
				return;
			}

			// Create colored cloud from normals for visualization
			PointCloudPtr colored = boost::make_shared<PointCloud>();
			colored->resize(cloud->size());
			for (size_t i = 0; i < cloud->size(); ++i) {
				const auto& pt = (*cloud)[i];
				pcl::PointXYZRGB cp;
				cp.x = pt.x; cp.y = pt.y; cp.z = pt.z;
				// Map normal components (-1..1) -> (0..255)
				const auto& n = ne_result.normals->at(i);
				int r = static_cast<int>((n.normal_x + 1.0f) * 0.5f * 255.0f);
				int g = static_cast<int>((n.normal_y + 1.0f) * 0.5f * 255.0f);
				int b = static_cast<int>((n.normal_z + 1.0f) * 0.5f * 255.0f);
				if (r < 0) r = 0; if (r > 255) r = 255;
				if (g < 0) g = 0; if (g > 255) g = 255;
				if (b < 0) b = 0; if (b > 255) b = 255;
				cp.r = static_cast<uint8_t>(r);
				cp.g = static_cast<uint8_t>(g);
				cp.b = static_cast<uint8_t>(b);
				(*colored)[i] = cp;
			}

			setCurrentCloud(colored);
			auto stats = io::calculateStats(colored);
			json response = { {"success", true}, {"stats", statsToJson(stats)} };
			res.set_content(response.dump(), "application/json");

		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// Get normals as binary Float32Array (nx,ny,nz for each point)
	m_server_.Get("/api/features/normals/binary", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto cloud = getCurrentCloud();
			if (!cloud || cloud->empty()) {
				res.status = 404;
				res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
				return;
			}

			// Parse optional query params
			int kSearch = 30;
			float radiusSearch = 0.0f;
		if (req.has_param("kSearch")) {
			kSearch = std::stoi(req.get_param_value("kSearch"));
		}
		if (req.has_param("radiusSearch")) {
			radiusSearch = std::stof(req.get_param_value("radiusSearch"));
		}

		// Compute normals
		pointcloud::features::NormalEstimatorConfig cfg;
		cfg.k_search = kSearch;
		cfg.radius_search = radiusSearch;
		pointcloud::features::NormalEstimator estimator(cfg);
		auto ne_result = estimator.compute(cloud);
		if (!ne_result.success) {
			res.status = 500;
			res.set_content(json{{"error", ne_result.error_message}}.dump(), "application/json");
			return;
		}

		// Build binary buffer
		std::vector<float> normals;
		normals.reserve(ne_result.normals->size() * 3);
		for (const auto& n : *ne_result.normals) {
			normals.push_back(n.normal_x);
			normals.push_back(n.normal_y);
			normals.push_back(n.normal_z);
		}

		res.set_content(
			std::string(reinterpret_cast<const char*>(normals.data()), normals.size() * sizeof(float)),
			"application/octet-stream"
		);

		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// Get binary points and colors after segmentation (region growing or plane)
	m_server_.Get("/api/points/binary", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto cloud = getCurrentCloud();
			if (!cloud || cloud->empty()) {
				res.status = 404;
				res.set_content(R"({"error":"No point cloud loaded"})", "application/json");
				return;
			}

			std::vector<float> positions;
			std::vector<uint8_t> colors;
			positions.reserve(cloud->size() * 3);
			colors.reserve(cloud->size() * 3);

			for (const auto& point : cloud->points) {
				positions.push_back(point.x);
				positions.push_back(point.y);
				positions.push_back(point.z);
				colors.push_back(point.r);
				colors.push_back(point.g);
				colors.push_back(point.b);
			}

			// Pack positions and colors into a single buffer
			// Format: positions_size (4 bytes) | positions_data | colors_data
			std::vector<uint8_t> buffer;
			uint32_t positions_size = static_cast<uint32_t>(positions.size() * sizeof(float));
			buffer.reserve(4 + positions_size + colors.size());

			// Write positions size
			buffer.insert(buffer.end(), 
				reinterpret_cast<const uint8_t*>(&positions_size), 
				reinterpret_cast<const uint8_t*>(&positions_size) + sizeof(uint32_t));

			// Write positions data
			buffer.insert(buffer.end(), 
				reinterpret_cast<const uint8_t*>(positions.data()), 
				reinterpret_cast<const uint8_t*>(positions.data()) + positions_size);

			// Write colors data
			buffer.insert(buffer.end(), colors.begin(), colors.end());

			res.set_content(
				std::string(reinterpret_cast<const char*>(buffer.data()), buffer.size()),
				"application/octet-stream"
			);

		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// ICP Registration - Set source and target clouds
	m_server_.Post("/api/icp/setup", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto body = json::parse(req.body);
			
			// Get which clouds to use: "current" or upload new ones
			std::string mode = body.value("mode", "current_as_target");
			
			std::lock_guard<std::mutex> lock(m_icp_mutex_);
			
			if (mode == "current_as_target") {
				// Use current cloud as target, need to upload or specify source
				auto cloud = getCurrentCloud();
				if (!cloud || cloud->empty()) {
					res.status = 404;
					res.set_content(R"({"error":"No point cloud loaded as target"})", "application/json");
					return;
				}
				m_target_cloud_ = cloud;
				
				// Create transformed version as source (for demo/testing)
				if (body.contains("applyTransform") && body["applyTransform"].get<bool>()) {
					m_source_cloud_ = boost::make_shared<PointCloud>(*cloud);
					
					// Apply a simple rotation and translation
					double theta = body.value("rotationAngle", M_PI / 8);  // Default 22.5 degrees
					double tz = body.value("translationZ", 0.4);
					
					Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
					transform(0, 0) = std::cos(theta);
					transform(0, 1) = -std::sin(theta);
					transform(1, 0) = std::sin(theta);
					transform(1, 1) = std::cos(theta);
					transform(2, 3) = tz;
					
					pcl::transformPointCloud(*cloud, *m_source_cloud_, transform);
				}
			} else if (mode == "separate_clouds") {
				// Future: support loading two separate clouds
				res.status = 400;
				res.set_content(R"({"error":"Separate clouds mode not yet implemented"})", "application/json");
				return;
			}
			
			// Initialize ICP
			if (!m_source_cloud_ || !m_target_cloud_) {
				res.status = 400;
				res.set_content(R"({"error":"Source and target clouds must be set"})", "application/json");
				return;
			}
			
			m_icp_->reset();
			m_icp_->setInputSource(m_source_cloud_);
			m_icp_->setInputTarget(m_target_cloud_);
			
			// Set ICP parameters
			if (body.contains("maxIterations")) {
				m_icp_->setMaximumIterations(body["maxIterations"].get<int>());
			}
			if (body.contains("maxCorrespondenceDistance")) {
				m_icp_->setMaxCorrespondenceDistance(body["maxCorrespondenceDistance"].get<double>());
			}
			if (body.contains("transformationEpsilon")) {
				m_icp_->setTransformationEpsilon(body["transformationEpsilon"].get<double>());
			}
			if (body.contains("euclideanFitnessEpsilon")) {
				m_icp_->setEuclideanFitnessEpsilon(body["euclideanFitnessEpsilon"].get<double>());
			}
			
			json response = {
				{"success", true},
				{"sourcePoints", m_source_cloud_->size()},
				{"targetPoints", m_target_cloud_->size()}
			};
			res.set_content(response.dump(), "application/json");
			
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// ICP Registration - Perform alignment
	m_server_.Post("/api/icp/align", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			auto body = json::parse(req.body);
			int iterations = body.value("iterations", 1);
			
			std::lock_guard<std::mutex> lock(m_icp_mutex_);
			
			if (!m_source_cloud_ || !m_target_cloud_) {
				res.status = 400;
				res.set_content(R"({"error":"ICP not initialized. Call /api/icp/setup first"})", "application/json");
				return;
			}
			
			m_icp_->setMaximumIterations(iterations);
			PointCloudPtr aligned = boost::make_shared<PointCloud>();
			auto result = m_icp_->align(aligned);
			
			if (!result.converged) {
				res.status = 400;
				res.set_content(json{{"error", result.error_message}}.dump(), "application/json");
				return;
			}
			
			// Update source cloud for next iteration
			m_source_cloud_ = aligned;
			setCurrentCloud(aligned);
			
			// Convert transformation matrix to JSON
			json transform_json = json::array();
			for (int i = 0; i < 4; ++i) {
				json row = json::array();
				for (int j = 0; j < 4; ++j) {
					row.push_back(result.transformation_matrix(i, j));
				}
				transform_json.push_back(row);
			}
			
			json cumulative_transform = json::array();
			auto cumulative = m_icp_->getCumulativeTransformation();
			for (int i = 0; i < 4; ++i) {
				json row = json::array();
				for (int j = 0; j < 4; ++j) {
					row.push_back(cumulative(i, j));
				}
				cumulative_transform.push_back(row);
			}
			
			auto stats = io::calculateStats(aligned);
			json response = {
				{"success", true},
				{"converged", result.converged},
				{"fitnessScore", result.fitness_score},
				{"iterationsDone", result.iterations_done},
				{"transformation", transform_json},
				{"cumulativeTransformation", cumulative_transform},
				{"stats", statsToJson(stats)}
			};
			res.set_content(response.dump(), "application/json");
			
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// ICP Registration - Iterate one step (for interactive use)
	m_server_.Post("/api/icp/iterate", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			std::lock_guard<std::mutex> lock(m_icp_mutex_);
			
			if (!m_source_cloud_ || !m_target_cloud_) {
				res.status = 400;
				res.set_content(R"({"error":"ICP not initialized. Call /api/icp/setup first"})", "application/json");
				return;
			}
			
			PointCloudPtr aligned = boost::make_shared<PointCloud>();
			auto result = m_icp_->alignOneIteration(aligned);
			
			if (!result.converged) {
				res.status = 400;
				res.set_content(json{{"error", result.error_message}}.dump(), "application/json");
				return;
			}
			
			// Update source cloud for next iteration
			m_source_cloud_ = aligned;
			setCurrentCloud(aligned);
			
			// Convert matrices to JSON
			json transform_json = json::array();
			for (int i = 0; i < 4; ++i) {
				json row = json::array();
				for (int j = 0; j < 4; ++j) {
					row.push_back(result.transformation_matrix(i, j));
				}
				transform_json.push_back(row);
			}
			
			json cumulative_transform = json::array();
			auto cumulative = m_icp_->getCumulativeTransformation();
			for (int i = 0; i < 4; ++i) {
				json row = json::array();
				for (int j = 0; j < 4; ++j) {
					row.push_back(cumulative(i, j));
				}
				cumulative_transform.push_back(row);
			}
			
			auto stats = io::calculateStats(aligned);
			json response = {
				{"success", true},
				{"converged", result.converged},
				{"fitnessScore", result.fitness_score},
				{"iterationsDone", result.iterations_done},
				{"transformation", transform_json},
				{"cumulativeTransformation", cumulative_transform},
				{"stats", statsToJson(stats)}
			};
			res.set_content(response.dump(), "application/json");
			
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// ICP Registration - Reset
	m_server_.Post("/api/icp/reset", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			std::lock_guard<std::mutex> lock(m_icp_mutex_);
			m_icp_->reset();
			m_source_cloud_.reset();
			m_target_cloud_.reset();
			
			json response = {{"success", true}};
			res.set_content(response.dump(), "application/json");
			
		} catch (const std::exception& e) {
			res.status = 500;
			res.set_content(json{{"error", e.what()}}.dump(), "application/json");
		}
	});

	// ICP Registration - Get source and target clouds for visualization
	m_server_.Get("/api/icp/clouds", [this](const httplib::Request& req, httplib::Response& res) {
		try {
			std::lock_guard<std::mutex> lock(m_icp_mutex_);
			
			if (!m_source_cloud_ || !m_target_cloud_) {
				res.status = 404;
				res.set_content(R"({"error":"ICP clouds not available"})", "application/json");
				return;
			}
			
			json response = {
				{"sourcePoints", m_source_cloud_->size()},
				{"targetPoints", m_target_cloud_->size()}
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

