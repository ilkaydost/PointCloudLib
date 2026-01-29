#include <gtest/gtest.h>

namespace fs = std::filesystem;

class IOTest : public ::testing::Test {
protected:
    std::string test_data_dir;
    
    void SetUp() override {
        // Test data is copied to build/tests/data by CMake
        test_data_dir = "tests/data/";
    }
};

TEST_F(IOTest, LoadPCDFile) {
    std::string file_path = test_data_dir + "sample.pcd";
    
    auto result = pointcloud::io::loadPointCloud(file_path);
    
    EXPECT_TRUE(result.success) << "Error: " << result.error_message;
    EXPECT_NE(result.cloud, nullptr);
    EXPECT_EQ(result.stats.point_count, 10);
}

TEST_F(IOTest, LoadNonExistentFile) {
    std::string file_path = test_data_dir + "nonexistent.pcd";
    
    auto result = pointcloud::io::loadPointCloud(file_path);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

TEST_F(IOTest, LoadUnsupportedFormat) {
    std::string file_path = test_data_dir + "sample.xyz";
    
    auto result = pointcloud::io::loadPointCloud(file_path);
    
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.error_message.find("Unsupported") != std::string::npos);
}

TEST_F(IOTest, CalculateStats) {
    std::string file_path = test_data_dir + "sample.pcd";
    
    auto result = pointcloud::io::loadPointCloud(file_path);
    ASSERT_TRUE(result.success);
    
    EXPECT_FLOAT_EQ(result.stats.min_x, 0.0f);
    EXPECT_FLOAT_EQ(result.stats.max_x, 0.2f);
    EXPECT_FLOAT_EQ(result.stats.min_y, 0.0f);
    EXPECT_FLOAT_EQ(result.stats.max_y, 0.2f);
    EXPECT_FLOAT_EQ(result.stats.min_z, 0.0f);
    EXPECT_FLOAT_EQ(result.stats.max_z, 0.1f);
}

TEST_F(IOTest, GetFileExtension) {
    EXPECT_EQ(pointcloud::io::getFileExtension("test.pcd"), "pcd");
    EXPECT_EQ(pointcloud::io::getFileExtension("test.PCD"), "pcd");
    EXPECT_EQ(pointcloud::io::getFileExtension("path/to/file.ply"), "ply");
    EXPECT_EQ(pointcloud::io::getFileExtension("no_extension"), "");
}
