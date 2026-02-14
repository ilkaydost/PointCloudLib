#ifndef PCL_ICP_REGISTRATION_HPP
#define PCL_ICP_REGISTRATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

#include "../types/point_types.hpp"

using namespace pointcloud;

namespace pcl_wrapper {

/**
 * @brief Result structure for ICP registration
 */
struct ICPResult {
    bool converged;
    double fitness_score;
    Eigen::Matrix4d transformation_matrix;
    int iterations_done;
    std::string error_message;
};

/**
 * @brief Interactive Iterative Closest Point registration wrapper
 * 
 * This class wraps PCL's ICP algorithm for point cloud alignment.
 * Supports both batch alignment and iterative refinement.
 */
class ICPRegistration {
public:
    /**
     * @brief Constructor
     */
    ICPRegistration();

    /**
     * @brief Set the source point cloud (cloud to be aligned)
     * @param cloud Source point cloud
     */
    void setInputSource(const PointCloudPtr& cloud);

    /**
     * @brief Set the target point cloud (reference cloud)
     * @param cloud Target point cloud
     */
    void setInputTarget(const PointCloudPtr& cloud);

    /**
     * @brief Set maximum number of iterations
     * @param max_iterations Maximum iterations per alignment call
     */
    void setMaximumIterations(int max_iterations);

    /**
     * @brief Set the maximum correspondence distance
     * @param distance Maximum distance between corresponding points
     */
    void setMaxCorrespondenceDistance(double distance);

    /**
     * @brief Set transformation epsilon (convergence criterion)
     * @param epsilon Transformation epsilon
     */
    void setTransformationEpsilon(double epsilon);

    /**
     * @brief Set Euclidean fitness epsilon (convergence criterion)
     * @param epsilon Euclidean fitness epsilon
     */
    void setEuclideanFitnessEpsilon(double epsilon);

    /**
     * @brief Perform ICP alignment
     * @param output Output aligned point cloud
     * @return ICPResult containing convergence status, score, and transformation
     */
    ICPResult align(PointCloudPtr& output);

    /**
     * @brief Perform single iteration of ICP (for interactive use)
     * @param output Output aligned point cloud
     * @return ICPResult containing convergence status, score, and transformation
     */
    ICPResult alignOneIteration(PointCloudPtr& output);

    /**
     * @brief Get the final transformation matrix
     * @return 4x4 transformation matrix
     */
    Eigen::Matrix4d getFinalTransformation();

    /**
     * @brief Get the cumulative transformation matrix from all iterations
     * @return 4x4 cumulative transformation matrix
     */
    Eigen::Matrix4d getCumulativeTransformation() const;

    /**
     * @brief Get fitness score
     * @return Fitness score (lower is better)
     */
    double getFitnessScore();

    /**
     * @brief Check if ICP has converged
     * @return True if converged, false otherwise
     */
    bool hasConverged();

    /**
     * @brief Reset the ICP state and cumulative transformation
     */
    void reset();

    /**
     * @brief Get the number of iterations performed
     * @return Number of iterations
     */
    int getIterationsDone() const;

private:
    pcl::IterativeClosestPoint<PointT, PointT> icp_;
    Eigen::Matrix4d cumulative_transformation_;
    int total_iterations_;
    bool is_initialized_;
};

} // namespace pcl_wrapper

#endif // PCL_ICP_REGISTRATION_HPP
