#include "icp_registration.hpp"
#include <iostream>

using namespace pointcloud;

namespace pcl_wrapper {

ICPRegistration::ICPRegistration()
    : cumulative_transformation_(Eigen::Matrix4d::Identity()),
      total_iterations_(0),
      is_initialized_(false) {
    // Set default ICP parameters
    icp_.setMaximumIterations(1);
    icp_.setMaxCorrespondenceDistance(0.05);  // 5cm default
    icp_.setTransformationEpsilon(1e-8);
    icp_.setEuclideanFitnessEpsilon(1e-6);
}

void ICPRegistration::setInputSource(const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Warning: Empty or null source cloud provided to ICP" << std::endl;
        return;
    }
    icp_.setInputSource(cloud);
    is_initialized_ = (icp_.getInputTarget() != nullptr);
}

void ICPRegistration::setInputTarget(const PointCloudPtr& cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Warning: Empty or null target cloud provided to ICP" << std::endl;
        return;
    }
    icp_.setInputTarget(cloud);
    is_initialized_ = (icp_.getInputSource() != nullptr);
}

void ICPRegistration::setMaximumIterations(int max_iterations) {
    if (max_iterations < 1) {
        std::cerr << "Warning: Maximum iterations must be >= 1, using 1" << std::endl;
        max_iterations = 1;
    }
    icp_.setMaximumIterations(max_iterations);
}

void ICPRegistration::setMaxCorrespondenceDistance(double distance) {
    if (distance <= 0) {
        std::cerr << "Warning: Max correspondence distance must be > 0" << std::endl;
        return;
    }
    icp_.setMaxCorrespondenceDistance(distance);
}

void ICPRegistration::setTransformationEpsilon(double epsilon) {
    if (epsilon <= 0) {
        std::cerr << "Warning: Transformation epsilon must be > 0" << std::endl;
        return;
    }
    icp_.setTransformationEpsilon(epsilon);
}

void ICPRegistration::setEuclideanFitnessEpsilon(double epsilon) {
    if (epsilon <= 0) {
        std::cerr << "Warning: Euclidean fitness epsilon must be > 0" << std::endl;
        return;
    }
    icp_.setEuclideanFitnessEpsilon(epsilon);
}

ICPResult ICPRegistration::align(PointCloudPtr& output) {
    ICPResult result;
    result.converged = false;
    result.fitness_score = std::numeric_limits<double>::max();
    result.transformation_matrix = Eigen::Matrix4d::Identity();
    result.iterations_done = 0;

    if (!is_initialized_) {
        result.error_message = "ICP not initialized: source and target clouds must be set";
        return result;
    }

    if (!output) {
        output = boost::make_shared<PointCloud>();
    }

    try {
        icp_.align(*output);
        
        result.converged = icp_.hasConverged();
        if (result.converged) {
            result.fitness_score = icp_.getFitnessScore();
            result.transformation_matrix = icp_.getFinalTransformation().cast<double>();
            
            // Update cumulative transformation
            cumulative_transformation_ = cumulative_transformation_ * result.transformation_matrix;
            total_iterations_ += icp_.getMaximumIterations();
            result.iterations_done = total_iterations_;
        } else {
            result.error_message = "ICP did not converge";
        }
    } catch (const std::exception& e) {
        result.error_message = std::string("ICP alignment failed: ") + e.what();
    }

    return result;
}

ICPResult ICPRegistration::alignOneIteration(PointCloudPtr& output) {
    // Store current max iterations
    int current_max_iter = icp_.getMaximumIterations();
    
    // Set to 1 iteration
    icp_.setMaximumIterations(1);
    
    // Perform alignment
    ICPResult result = align(output);
    
    // Restore max iterations
    icp_.setMaximumIterations(current_max_iter);
    
    return result;
}

Eigen::Matrix4d ICPRegistration::getFinalTransformation() {
    return icp_.getFinalTransformation().cast<double>();
}

Eigen::Matrix4d ICPRegistration::getCumulativeTransformation() const {
    return cumulative_transformation_;
}

double ICPRegistration::getFitnessScore() {
    return icp_.getFitnessScore();
}

bool ICPRegistration::hasConverged() {
    return icp_.hasConverged();
}

void ICPRegistration::reset() {
    cumulative_transformation_ = Eigen::Matrix4d::Identity();
    total_iterations_ = 0;
}

int ICPRegistration::getIterationsDone() const {
    return total_iterations_;
}

} // namespace pcl_wrapper
