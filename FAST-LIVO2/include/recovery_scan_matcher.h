#ifndef FAST_LIVO_RECOVERY_SCAN_MATCHER_H
#define FAST_LIVO_RECOVERY_SCAN_MATCHER_H

#include "utils/types.h"

#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// A local, scan-to-scan constraint used only while normal scan-to-map ESIKF
// constraints are unavailable.  The result is expressed as the transform from
// the current LiDAR frame into the previous accepted LiDAR frame.  Its 6-DoF
// covariance is derived from the point-to-point normal matrix, so weak
// directions remain weak instead of being turned into an isotropic "good ICP"
// flag.
struct RecoveryScanMatchResult
{
  bool converged = false;
  Eigen::Matrix4d current_to_previous = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
  Eigen::Matrix<double, 6, 1> direction_factors =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> direction_basis =
      Eigen::Matrix<double, 6, 6>::Identity();
  double fitness = std::numeric_limits<double>::infinity();
  double rmse = std::numeric_limits<double>::infinity();
  int correspondences = 0;
};

class RecoveryScanMatcher
{
public:
  int maximum_iterations = 20;
  double maximum_correspondence_distance = 1.5;
  double maximum_relative_translation = 3.0;
  double maximum_relative_rotation = 0.70;
  double maximum_rmse = 0.75;
  int minimum_correspondences = 50;
  double minimum_coverage = 0.10;

  RecoveryScanMatchResult align(const PointCloudXYZI::ConstPtr &current,
                                const PointCloudXYZI::ConstPtr &previous,
                                const Eigen::Matrix4d &initial_current_to_previous) const
  {
    RecoveryScanMatchResult result;
    if (!current || !previous || current->size() < 50 || previous->size() < 50 ||
        !initial_current_to_previous.allFinite())
    {
      return result;
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(current);
    icp.setInputTarget(previous);
    icp.setMaximumIterations(std::max(maximum_iterations, 1));
    icp.setMaxCorrespondenceDistance(std::max(maximum_correspondence_distance, 0.05));
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);
    icp.setRANSACOutlierRejectionThreshold(
        0.5 * std::max(maximum_correspondence_distance, 0.05));

    PointCloudXYZI aligned;
    icp.align(aligned, initial_current_to_previous.cast<float>());
    if (!icp.hasConverged()) return result;

    const Eigen::Matrix4d transform = icp.getFinalTransformation().cast<double>();
    const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
    if (!transform.allFinite() ||
        std::abs(rotation.determinant() - 1.0) > 1e-2 ||
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-2 ||
        translation.norm() > std::max(maximum_relative_translation, 0.1) ||
        Eigen::AngleAxisd(rotation).angle() > std::max(maximum_relative_rotation, 0.05))
    {
      return result;
    }

    pcl::KdTreeFLANN<PointType> target_tree;
    target_tree.setInputCloud(previous);
    const double max_distance_sq = maximum_correspondence_distance * maximum_correspondence_distance;
    Eigen::Matrix<double, 6, 6> normal_matrix =
        Eigen::Matrix<double, 6, 6>::Zero();
    double residual_square_sum = 0.0;
    int correspondence_count = 0;
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_distance_sq(1);

    for (size_t point_index = 0; point_index < current->size(); ++point_index)
    {
      const PointType &source_point = current->points[point_index];
      const Eigen::Vector3d source(source_point.x, source_point.y, source_point.z);
      if (!source.allFinite()) continue;
      const Eigen::Vector3d transformed = rotation * source + translation;
      if (!transformed.allFinite()) continue;

      PointType query;
      query.x = static_cast<float>(transformed.x());
      query.y = static_cast<float>(transformed.y());
      query.z = static_cast<float>(transformed.z());
      if (target_tree.nearestKSearch(query, 1, nearest_index, nearest_distance_sq) != 1 ||
          nearest_distance_sq[0] > max_distance_sq)
      {
        continue;
      }

      const PointType &target_point = previous->points[nearest_index[0]];
      const Eigen::Vector3d target(target_point.x, target_point.y, target_point.z);
      if (!target.allFinite()) continue;
      const Eigen::Vector3d residual = transformed - target;

      Eigen::Matrix3d source_skew;
      source_skew << 0.0, -source.z(), source.y(),
                     source.z(), 0.0, -source.x(),
                    -source.y(), source.x(), 0.0;
      Eigen::Matrix<double, 3, 6> jacobian;
      jacobian.block<3, 3>(0, 0) = -rotation * source_skew;
      jacobian.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
      normal_matrix.noalias() += jacobian.transpose() * jacobian;
      residual_square_sum += residual.squaredNorm();
      correspondence_count++;
    }

    const int required_correspondences = std::max(
        std::max(minimum_correspondences, 1),
        static_cast<int>(std::ceil(std::max(minimum_coverage, 0.0) *
                                   static_cast<double>(current->size()))));
    if (correspondence_count < required_correspondences) return result;

    const double rmse = std::sqrt(residual_square_sum /
                                  static_cast<double>(correspondence_count));
    const double fitness = icp.getFitnessScore(maximum_correspondence_distance);
    if (!std::isfinite(rmse) || !std::isfinite(fitness) ||
        rmse > std::max(maximum_rmse, 0.05))
    {
      return result;
    }

    // Use the per-correspondence mean information.  This deliberately avoids
    // treating thousands of spatially correlated LiDAR points as thousands of
    // independent measurements.  Eigenvectors retain the coupled rotation / 
    // translation directions of the local geometry.
    normal_matrix /= static_cast<double>(correspondence_count);
    normal_matrix = 0.5 * (normal_matrix + normal_matrix.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(normal_matrix);
    if (eigensolver.info() != Eigen::Success ||
        !eigensolver.eigenvalues().allFinite() ||
        !eigensolver.eigenvectors().allFinite())
    {
      return result;
    }

    const Eigen::Matrix<double, 6, 1> eigenvalues =
        eigensolver.eigenvalues().cwiseMax(0.0);
    const double maximum_eigenvalue = eigenvalues.maxCoeff();
    if (!std::isfinite(maximum_eigenvalue) || maximum_eigenvalue <= 1e-9) return result;

    Eigen::Matrix<double, 6, 1> covariance_eigenvalues;
    const double point_variance = std::max(rmse * rmse, 0.0025);
    for (int direction = 0; direction < 6; ++direction)
    {
      const double direction_information = eigenvalues(direction);
      const double factor = direction_information /
                            (direction_information + 0.01 * maximum_eigenvalue);
      result.direction_factors(direction) = std::max(0.0, std::min(factor, 1.0));
      if (direction_information <= 1e-4 * maximum_eigenvalue)
      {
        covariance_eigenvalues(direction) = 100.0;
      }
      else
      {
        covariance_eigenvalues(direction) =
            std::max(point_variance / direction_information, 1e-8);
      }
    }

    result.covariance = eigensolver.eigenvectors() *
                        covariance_eigenvalues.asDiagonal() *
                        eigensolver.eigenvectors().transpose();
    result.direction_basis = eigensolver.eigenvectors();
    result.covariance = 0.5 * (result.covariance + result.covariance.transpose());
    result.covariance.diagonal().array() += 1e-9;
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> covariance_llt(result.covariance);
    if (!result.covariance.allFinite() || covariance_llt.info() != Eigen::Success)
    {
      return RecoveryScanMatchResult();
    }

    result.converged = true;
    result.current_to_previous = transform;
    result.fitness = fitness;
    result.rmse = rmse;
    result.correspondences = correspondence_count;
    return result;
  }
};

#endif
