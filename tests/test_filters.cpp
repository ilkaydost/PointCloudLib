#include <gtest/gtest.h>
#include "core/io/point_cloud_io.hpp"
#include "core/filters/filters.hpp"

using namespace pointcloud;
using namespace pointcloud::filters;

class FiltersTest : public ::testing::Test {
protected:
    PointCloudPtr test_cloud;
    
    void SetUp() override {
        // Create a simple test cloud
        test_cloud = std::make_shared<PointCloud>();
        
        // Create a 3x3x3 grid of points
        for (float x = 0.0f; x <= 0.2f; x += 0.1f) {
            for (float y = 0.0f; y <= 0.2f; y += 0.1f) {
                for (float z = 0.0f; z <= 0.2f; z += 0.1f) {
                    PointT pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.r = 255;
                    pt.g = 255;
                    pt.b = 255;
                    test_cloud->push_back(pt);
                }
            }
        }
    }
};

// PassThrough Filter Tests
TEST_F(FiltersTest, PassThroughFilterZ) {
    PassThroughConfig config;
    config.field_name = "z";
    config.min_limit = 0.0f;
    config.max_limit = 0.1f;
    
    PassThroughFilter filter(config);
    auto result = filter.apply(test_cloud);
    
    EXPECT_LT(result->size(), test_cloud->size());
    
    // Verify all remaining points are within Z bounds
    for (const auto& pt : *result) {
        EXPECT_GE(pt.z, 0.0f);
        EXPECT_LE(pt.z, 0.1f);
    }
}

TEST_F(FiltersTest, PassThroughFilterX) {
    PassThroughConfig config;
    config.field_name = "x";
    config.min_limit = 0.05f;
    config.max_limit = 0.15f;
    
    PassThroughFilter filter(config);
    auto result = filter.apply(test_cloud);
    
    for (const auto& pt : *result) {
        EXPECT_GE(pt.x, 0.05f);
        EXPECT_LE(pt.x, 0.15f);
    }
}

TEST_F(FiltersTest, PassThroughFilterNegative) {
    PassThroughConfig config;
    config.field_name = "z";
    config.min_limit = 0.0f;
    config.max_limit = 0.1f;
    config.negative = true;  // Keep points OUTSIDE range
    
    PassThroughFilter filter(config);
    auto result = filter.apply(test_cloud);
    
    // Points outside the range should remain
    for (const auto& pt : *result) {
        EXPECT_TRUE(pt.z < 0.0f || pt.z > 0.1f);
    }
}

TEST_F(FiltersTest, PassThroughEmptyCloud) {
    PassThroughFilter filter;
    auto empty_cloud = std::make_shared<PointCloud>();
    auto result = filter.apply(empty_cloud);
    
    EXPECT_TRUE(result->empty());
}

// VoxelGrid Filter Tests
TEST_F(FiltersTest, VoxelGridDownsample) {
    VoxelGridFilter filter(0.15f);  // Large leaf size
    auto result = filter.apply(test_cloud);
    
    EXPECT_LT(result->size(), test_cloud->size());
    EXPECT_GT(result->size(), 0);
}

TEST_F(FiltersTest, VoxelGridSmallLeafSize) {
    VoxelGridFilter filter(0.01f);  // Very small leaf size
    auto result = filter.apply(test_cloud);
    
    // Should keep most points with small leaf size
    EXPECT_GE(result->size(), test_cloud->size() / 2);
}

TEST_F(FiltersTest, VoxelGridEmptyCloud) {
    VoxelGridFilter filter(0.1f);
    auto empty_cloud = std::make_shared<PointCloud>();
    auto result = filter.apply(empty_cloud);
    
    EXPECT_TRUE(result->empty());
}

TEST_F(FiltersTest, VoxelGridConfig) {
    VoxelGridConfig config;
    config.leaf_size_x = 0.1f;
    config.leaf_size_y = 0.2f;
    config.leaf_size_z = 0.3f;
    
    VoxelGridFilter filter(config);
    auto retrieved = filter.getConfig();
    
    EXPECT_FLOAT_EQ(retrieved.leaf_size_x, 0.1f);
    EXPECT_FLOAT_EQ(retrieved.leaf_size_y, 0.2f);
    EXPECT_FLOAT_EQ(retrieved.leaf_size_z, 0.3f);
}
#include <gtest/gtest.h>

TEST(FiltersTest, Placeholder) {
    EXPECT_TRUE(true);
}
