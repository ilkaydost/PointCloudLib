#include <gtest/gtest.h>
#include "core/segmentation/region_growing_segmenter.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

using namespace pointcloud::segmentation;

class RegionGrowingTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple test cloud with two planar surfaces
        cloud_ = boost::make_shared<PointCloud>();
        
        // First plane: z = 0, x,y in [0, 10]
        for (float x = 0; x < 10; x += 0.5f) {
            for (float y = 0; y < 10; y += 0.5f) {
                PointT p;
                p.x = x;
                p.y = y;
                p.z = 0;
                p.r = 255;
                p.g = 0;
                p.b = 0;
                cloud_->push_back(p);
            }
        }
        
        // Second plane: z = 5, x,y in [0, 10]
        for (float x = 0; x < 10; x += 0.5f) {
            for (float y = 0; y < 10; y += 0.5f) {
                PointT p;
                p.x = x;
                p.y = y;
                p.z = 5;
                p.r = 0;
                p.g = 255;
                p.b = 0;
                cloud_->push_back(p);
            }
        }
    }
    
    PointCloudPtr cloud_;
};

TEST_F(RegionGrowingTest, DefaultConfig) {
    RegionGrowingSegmenter segmenter;
    auto config = segmenter.getConfig();
    
    EXPECT_FLOAT_EQ(config.smoothness_threshold, 5.0f);
    EXPECT_FLOAT_EQ(config.curvature_threshold, 1.0f);
    EXPECT_EQ(config.min_cluster_size, 50);
    EXPECT_EQ(config.max_cluster_size, 1000000);
    EXPECT_EQ(config.number_of_neighbours, 30);
    EXPECT_EQ(config.normal_k_search, 30);
}

TEST_F(RegionGrowingTest, SetConfig) {
    RegionGrowingSegmenter segmenter;
    RegionGrowingConfig config;
    config.smoothness_threshold = 10.0f;
    config.curvature_threshold = 2.0f;
    config.min_cluster_size = 100;
    config.max_cluster_size = 5000;
    config.number_of_neighbours = 50;
    config.normal_k_search = 50;
    
    segmenter.setConfig(config);
    auto retrieved = segmenter.getConfig();
    
    EXPECT_FLOAT_EQ(retrieved.smoothness_threshold, 10.0f);
    EXPECT_FLOAT_EQ(retrieved.curvature_threshold, 2.0f);
    EXPECT_EQ(retrieved.min_cluster_size, 100);
    EXPECT_EQ(retrieved.max_cluster_size, 5000);
    EXPECT_EQ(retrieved.number_of_neighbours, 50);
    EXPECT_EQ(retrieved.normal_k_search, 50);
}

TEST_F(RegionGrowingTest, EmptyCloud) {
    RegionGrowingSegmenter segmenter;
    auto empty_cloud = boost::make_shared<PointCloud>();
    
    auto result = segmenter.segment(empty_cloud);
    
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error_message, "Input cloud is empty");
    EXPECT_EQ(result.num_clusters, 0);
}

TEST_F(RegionGrowingTest, NullCloud) {
    RegionGrowingSegmenter segmenter;
    
    auto result = segmenter.segment(nullptr);
    
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error_message, "Input cloud is empty");
}

TEST_F(RegionGrowingTest, SegmentTwoPlanes) {
    RegionGrowingConfig config;
    config.min_cluster_size = 50;
    config.smoothness_threshold = 5.0f;
    config.curvature_threshold = 1.0f;
    
    RegionGrowingSegmenter segmenter(config);
    auto result = segmenter.segment(cloud_);
    
    EXPECT_TRUE(result.success);
    EXPECT_GT(result.num_clusters, 0);
    EXPECT_EQ(result.clusters.size(), result.num_clusters);
    EXPECT_EQ(result.indices.size(), result.num_clusters);
    
    // Verify colored cloud is not empty
    EXPECT_FALSE(result.colored_cloud->empty());
    
    // Verify total points in clusters matches or is close to input
    size_t total_points = 0;
    for (const auto& cluster : result.clusters) {
        EXPECT_FALSE(cluster->empty());
        EXPECT_GE(cluster->size(), static_cast<size_t>(config.min_cluster_size));
        total_points += cluster->size();
    }
    EXPECT_LE(total_points, cloud_->size());
}

TEST_F(RegionGrowingTest, ApplyMethod) {
    RegionGrowingSegmenter segmenter;
    auto result_cloud = segmenter.apply(cloud_);
    
    EXPECT_FALSE(result_cloud->empty());
    // Colored cloud should have same or fewer points (some might be rejected)
    EXPECT_LE(result_cloud->size(), cloud_->size());
}

TEST_F(RegionGrowingTest, ApplyWithEmptyCloud) {
    RegionGrowingSegmenter segmenter;
    auto empty_cloud = boost::make_shared<PointCloud>();
    auto result = segmenter.apply(empty_cloud);
    
    EXPECT_TRUE(result->empty());
}

TEST_F(RegionGrowingTest, StrictSmoothnessThreshold) {
    RegionGrowingConfig config;
    config.smoothness_threshold = 1.0f;  // Very strict
    config.min_cluster_size = 10;
    
    RegionGrowingSegmenter segmenter(config);
    auto result = segmenter.segment(cloud_);
    
    // With strict threshold, we might get more smaller clusters
    if (result.success) {
        EXPECT_GT(result.num_clusters, 0);
    }
}

TEST_F(RegionGrowingTest, RelaxedSmoothnessThreshold) {
    RegionGrowingConfig config;
    config.smoothness_threshold = 30.0f;  // Very relaxed
    config.min_cluster_size = 10;
    
    RegionGrowingSegmenter segmenter(config);
    auto result = segmenter.segment(cloud_);
    
    // With relaxed threshold, we might get fewer larger clusters
    if (result.success) {
        EXPECT_GT(result.num_clusters, 0);
    }
}
