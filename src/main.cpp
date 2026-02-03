#include <iostream>
#include <csignal>
#include "server/http_server.hpp"

namespace {
    pointcloud::server::HttpServer* g_server = nullptr;
    
    void signalHandler(int signal) {
        if (g_server) {
            std::cout << "\nShutting down server..." << std::endl;
            g_server->stop();
        }
    }
}

int main() {
    std::cout << "PointCloudLib Server v0.1.0" << std::endl;
    std::cout << "Starting HTTP server on port 5050..." << std::endl;
    
    // Set up signal handling for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    try {
        pointcloud::server::HttpServer server(5050);
        g_server = &server;
        
        std::cout << "Server running at http://0.0.0.0:5050" << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;
        
        server.start();  // Blocking call
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Server stopped." << std::endl;
    return 0;
}
