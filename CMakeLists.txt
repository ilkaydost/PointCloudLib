cmake_minimum_required(VERSION 3.16)
project(PointCloudLib VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add compiler flags for compatibility
add_compile_options(-fPIC)

# Find dependencies
find_package(PCL REQUIRED COMPONENTS common io filters features segmentation)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(Threads REQUIRED)
#find_package(GTest CONFIG REQUIRED)

# Find cpp-httplib (header-only, so we just need to find the header path)
find_path(HTTPLIB_INCLUDE_DIR httplib.h HINTS ${CMAKE_PREFIX_PATH})
if(NOT HTTPLIB_INCLUDE_DIR)
    message(FATAL_ERROR "cpp-httplib header not found. Please set CMAKE_PREFIX_PATH to include cpp-httplib directory.")
endif()

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${HTTPLIB_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Core library
add_library(pointcloud_core STATIC
    src/core/io/point_cloud_io.cpp
    src/core/filters/pass_through_filter.cpp
    src/core/filters/voxel_grid_filter.cpp
    src/core/segmentation/ransac_plane_segmenter.cpp
    src/core/segmentation/region_growing_segmenter.cpp
    src/core/features/normal_estimator.cpp
)

target_include_directories(pointcloud_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(pointcloud_core PUBLIC
    ${PCL_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# Server executable
add_executable(pointcloud_server
    src/main.cpp
    src/server/http_server.cpp
    src/server/api_routes.cpp
)

target_link_libraries(pointcloud_server PRIVATE
    pointcloud_core
    nlohmann_json::nlohmann_json
    Threads::Threads
)

# Testing
# enable_testing()

# add_executable(test_io tests/test_io.cpp)
# target_link_libraries(test_io PRIVATE pointcloud_core GTest::gtest GTest::gtest_main)
# add_test(NAME TestIO COMMAND test_io)

# add_executable(test_filters tests/test_filters.cpp)
# target_link_libraries(test_filters PRIVATE pointcloud_core GTest::gtest GTest::gtest_main)
# add_test(NAME TestFilters COMMAND test_filters)

# Copy test data if it exists
# if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/data)
#     file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tests/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tests)
# endif()
