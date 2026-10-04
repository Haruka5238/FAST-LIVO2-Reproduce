/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "voxel_map.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
const std::array<V3D, 13> &normalDirectionBins()
{
  static const std::array<V3D, 13> bins = {
      V3D(1, 0, 0), V3D(0, 1, 0), V3D(0, 0, 1),
      V3D(1, 1, 0).normalized(), V3D(1, -1, 0).normalized(),
      V3D(1, 0, 1).normalized(), V3D(1, 0, -1).normalized(),
      V3D(0, 1, 1).normalized(), V3D(0, 1, -1).normalized(),
      V3D(1, 1, 1).normalized(), V3D(1, 1, -1).normalized(),
      V3D(1, -1, 1).normalized(), V3D(-1, 1, 1).normalized()};
  return bins;
}

template <int N>
bool regularizeAndInvertSymmetric(const Eigen::Matrix<double, N, N> &input, const double relative_floor,
                                  Eigen::Matrix<double, N, N> &regularized,
                                  Eigen::Matrix<double, N, N> &inverse,
                                  double *regularized_min_eigenvalue = nullptr)
{
  if (!input.allFinite()) return false;
  const Eigen::Matrix<double, N, N> symmetric = 0.5 * (input + input.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() || !solver.eigenvectors().allFinite()) return false;

  Eigen::Matrix<double, N, 1> eigenvalues = solver.eigenvalues();
  const double max_eigenvalue = eigenvalues.maxCoeff();
  if (!std::isfinite(max_eigenvalue) || max_eigenvalue <= 0.0) return false;
  const double spectrum_scale = std::max(eigenvalues.cwiseAbs().maxCoeff(), 1e-12);
  const double negative_tolerance = std::max(1e-12, 1e-8 * spectrum_scale);
  if (eigenvalues.minCoeff() < -negative_tolerance) return false;
  const double floor_ratio = std::isfinite(relative_floor) ?
      std::max(1e-15, std::min(relative_floor, 1e-2)) : 1e-12;
  const double eigen_floor = std::max(1e-12, floor_ratio * max_eigenvalue);
  eigenvalues = eigenvalues.cwiseMax(eigen_floor);
  if (regularized_min_eigenvalue != nullptr) *regularized_min_eigenvalue = eigenvalues.minCoeff();
  regularized = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
  inverse = solver.eigenvectors() * eigenvalues.cwiseInverse().asDiagonal() * solver.eigenvectors().transpose();
  regularized = 0.5 * (regularized + regularized.transpose());
  inverse = 0.5 * (inverse + inverse.transpose());
  return regularized.allFinite() && inverse.allFinite();
}

double normalizedSigmoidHealth(const double ratio, const double alpha, const double tau)
{
  auto logistic = [](double value) -> double {
    value = std::max(-40.0, std::min(value, 40.0));
    return 1.0 / (1.0 + std::exp(-value));
  };
  const double ratio_clamped = std::max(0.0, std::min(ratio, 1.0));
  const double lower = logistic(-alpha * tau);
  const double upper = logistic(alpha * (1.0 - tau));
  const double value = logistic(alpha * (ratio_clamped - tau));
  if (upper - lower <= 1e-12) return ratio_clamped;
  return std::max(0.0, std::min((value - lower) / (upper - lower), 1.0));
}

bool lioStateMeanFinite(const StatesGroup &state)
{
  return state.rot_end.allFinite() && state.pos_end.allFinite() && state.vel_end.allFinite() &&
         state.bias_g.allFinite() && state.bias_a.allFinite() && state.gravity.allFinite() &&
         std::isfinite(state.inv_expo_time);
}
}

VoxelMapManager::~VoxelMapManager()
{
  for (auto &entry : voxel_map_) delete entry.second;
  voxel_map_.clear();
}

void VoxelMapManager::resetRuntimeState()
{
  for (auto &entry : voxel_map_) delete entry.second;
  voxel_map_.clear();
  voxel_map_.rehash(0);

  if (feats_undistort_) PointCloudXYZI().swap(*feats_undistort_);
  if (feats_down_body_) PointCloudXYZI().swap(*feats_down_body_);
  if (feats_down_world_) PointCloudXYZI().swap(*feats_down_world_);
  if (state_estimation_world_lidar_scratch_)
    pcl::PointCloud<pcl::PointXYZI>().swap(*state_estimation_world_lidar_scratch_);
  decltype(cross_mat_list_)().swap(cross_mat_list_);
  decltype(body_cov_list_)().swap(body_cov_list_);
  decltype(normal_bin_ids_buffer_)().swap(normal_bin_ids_buffer_);
  decltype(candidate_range_buffer_)().swap(candidate_range_buffer_);
  decltype(matched_range_buffer_)().swap(matched_range_buffer_);
  decltype(residual_ptpl_buffer_)().swap(residual_ptpl_buffer_);
  decltype(residual_useful_buffer_)().swap(residual_useful_buffer_);
  decltype(pv_list_)().swap(pv_list_);
  decltype(ptpl_list_)().swap(ptpl_list_);

  current_frame_id_ = 0;
  build_residual_time = 0.0f;
  ekf_time = 0.0f;
  ave_build_residual_time = 0.0f;
  ave_ekf_time = 0.0f;
  scan_count = 0;
  state_ = StatesGroup();
  position_last_.setZero();
  last_slide_position.setZero();
  feats_down_size_ = 0;
  effct_feat_num_ = 0;
  avg_residual_ = 0.0;
  ekf_converged_ = false;
  state_estimation_valid_ = true;
  state_prior_invalid_ = false;

  degeneracy_eigenvalues_.setOnes();
  degeneracy_direction_factors_.setOnes();
  degeneracy_state_directions_.setIdentity();
  degeneracy_factor_ = 1.0;
  degeneracy_mean_factor_ = 1.0;
  degeneracy_whitened_min_eigenvalue_ = 0.0;
  degeneracy_whitened_max_eigenvalue_ = 0.0;
  degeneracy_whitened_condition_ratio_ = 1.0;
  degeneracy_rot_trans_coupling_ = 0.0;
  degeneracy_coupled_active_ = false;
  degeneracy_numerical_fallback_ = false;
  covariance_relinearization_angle_ = 0.0;
  covariance_reset_angle_ = 0.0;
  covariance_min_eigenvalue_ = -1.0;
  covariance_symmetry_error_ = 0.0;
  normal_anisotropy_ = 1.0;
  normal_balance_mean_weight_ = 1.0;
  dominant_normal_ratio_ = 0.0;
  incidence_weight_mean_ = 1.0;
  robust_kernel_mean_weight_ = 1.0;
  residual_consistency_mean_weight_ = 1.0;
  lio_hard_outlier_reject_num_ = 0;
  scan_distortion_risk_ = 0.0;
  scan_distortion_time_span_ = 0.0;
  scan_distortion_mean_weight_ = 1.0;
  lidar_range_ref_effective_ = lidar_range_ref_;
  lidar_range_quantile_value_ = lidar_range_ref_;
  lidar_range_candidate_quantile_ = -1.0;
  lidar_range_matched_quantile_ = -1.0;
  lidar_range_selection_gap_ = -1.0;
  lidar_range_candidate_count_ = 0;
  lidar_range_matched_coverage_ = 0.0;
  lidar_range_ref_initialized_ = false;
  lidar_return_quality_mean_weight_ = 1.0;
  lidar_return_intensity_score_ = 1.0;
  lidar_return_density_score_ = 1.0;
  map_write_motion_risk_ = 0.0;
  map_write_quality_risk_ = 0.0;
  map_write_starvation_frames_ = 0;
  map_write_recovery_active_ = false;
  map_write_recovery_accept_num_ = 0;
  decltype(map_write_candidate_voxels_)().swap(map_write_candidate_voxels_);
  map_write_candidate_point_num_ = 0;
  map_write_candidate_promoted_voxels_ = 0;
  map_write_candidate_expired_voxels_ = 0;
  map_write_external_freeze_ = false;
  map_write_formal_map_frozen_ = false;
  map_write_accept_num_ = 0;
  map_write_reject_num_ = 0;
  map_write_accept_ratio_ = 1.0;
  map_write_mean_score_ = 1.0;
  dynamic_mean_score_ = 0.0;
  dynamic_mean_weight_ = 1.0;
  dynamic_reject_num_ = 0;
  total_lio_effective_weight_mean_ = 1.0;
  total_lio_low_weight_ratio_ = 0.0;
}

void calcBodyCov(const Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov)
{
  const double range = pb.norm();
  const double range_inc_safe = std::isfinite(range_inc) ? std::fabs(static_cast<double>(range_inc)) : 0.0;
  const double degree_inc_safe = std::isfinite(degree_inc) ? std::fabs(static_cast<double>(degree_inc)) : 0.0;
  const double range_var = range_inc_safe * range_inc_safe;
  if (!pb.allFinite() || !std::isfinite(range) || range <= 1e-9)
  {
    cov = Eigen::Matrix3d::Identity() * std::max(range_var, 1e-12);
    return;
  }

  Eigen::Matrix2d direction_var;
  const double angular_var = std::pow(std::sin(DEG2RAD(degree_inc_safe)), 2);
  direction_var << angular_var, 0, 0, angular_var;
  const Eigen::Vector3d direction = pb / range;
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  const Eigen::Vector3d base_vector1 = direction.unitOrthogonal();
  const Eigen::Vector3d base_vector2 = direction.cross(base_vector1).normalized();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
  cov = 0.5 * (cov + cov.transpose());
  if (!cov.allFinite()) { cov = Eigen::Matrix3d::Identity() * std::max(range_var, 1e-12); }
}

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config)
{
  nh.param<bool>("publish/pub_plane_en", voxel_config.is_pub_plane_map_, false);
  
  nh.param<int>("lio/max_layer", voxel_config.max_layer_, 1);
  nh.param<double>("lio/voxel_size", voxel_config.max_voxel_size_, 0.5);
  nh.param<double>("lio/min_eigen_value", voxel_config.planner_threshold_, 0.01);
  nh.param<double>("lio/sigma_num", voxel_config.sigma_num_, 3);
  nh.param<double>("lio/beam_err", voxel_config.beam_err_, 0.02);
  nh.param<double>("lio/dept_err", voxel_config.dept_err_, 0.05);
  nh.param<vector<int>>("lio/layer_init_num", voxel_config.layer_init_num_, vector<int>{5,5,5,5,5});
  nh.param<int>("lio/max_points_num", voxel_config.max_points_num_, 50);
  nh.param<int>("lio/max_iterations", voxel_config.max_iterations_, 5);

  nh.param<bool>("local_map/map_sliding_en", voxel_config.map_sliding_en, false);
  nh.param<int>("local_map/half_map_size", voxel_config.half_map_size, 100);
  nh.param<double>("local_map/sliding_thresh", voxel_config.sliding_thresh, 8);

  auto rejectConfig = [](const std::string &key, const std::string &reason) {
    const std::string message = "Invalid voxel-map parameter '" + key + "': " + reason;
    ROS_FATAL_STREAM(message);
    throw std::invalid_argument(message);
  };
  auto requireFiniteRange = [&rejectConfig](const std::string &key, const double value,
                                             const double lower, const double upper) {
    if (!std::isfinite(value) || value < lower || value > upper)
    {
      rejectConfig(key, "must be finite and in [" + std::to_string(lower) + ", " +
                            std::to_string(upper) + "]");
    }
  };
  auto requireIntRange = [&rejectConfig](const std::string &key, const int value,
                                         const int lower, const int upper) {
    if (value < lower || value > upper)
    {
      rejectConfig(key, "must be in [" + std::to_string(lower) + ", " +
                            std::to_string(upper) + "]");
    }
  };

  requireIntRange("lio/max_layer", voxel_config.max_layer_, 0, 64);
  requireFiniteRange("lio/voxel_size", voxel_config.max_voxel_size_, 1e-4, 1e4);
  requireFiniteRange("lio/min_eigen_value", voxel_config.planner_threshold_, 1e-12, 1e6);
  requireFiniteRange("lio/sigma_num", voxel_config.sigma_num_, 1e-3, 1e3);
  requireFiniteRange("lio/beam_err", voxel_config.beam_err_, 1e-9, 1e3);
  requireFiniteRange("lio/dept_err", voxel_config.dept_err_, 1e-9, 1e3);
  requireIntRange("lio/max_points_num", voxel_config.max_points_num_, 1, 10000000);
  requireIntRange("lio/max_iterations", voxel_config.max_iterations_, 1, 1000);
  requireIntRange("local_map/half_map_size", voxel_config.half_map_size, 1, 10000000);
  requireFiniteRange("local_map/sliding_thresh", voxel_config.sliding_thresh, 1e-6, 1e9);

  const size_t required_layers = static_cast<size_t>(voxel_config.max_layer_) + 1U;
  if (voxel_config.layer_init_num_.size() < required_layers)
  {
    rejectConfig("lio/layer_init_num", "requires at least max_layer + 1 entries");
  }
  for (size_t layer = 0; layer < voxel_config.layer_init_num_.size(); ++layer)
  {
    if (voxel_config.layer_init_num_[layer] < 1 || voxel_config.layer_init_num_[layer] > 10000000)
    {
      rejectConfig("lio/layer_init_num", "all entries must be in [1, 10000000]");
    }
  }
}

double VoxelMapManager::computeLidarReturnWeight(const double intensity, const double range, const double density_score,
                                                 double *intensity_score_out, double *density_score_out,
                                                 const bool apply_min_weight) const
{
  double intensity_score = 1.0;
  const double density_safe = std::isfinite(density_score) ? density_score : 0.0;
  double density_clamped = std::max(0.0, std::min(density_safe, 1.0));
  double return_weight = 1.0;
  if (lidar_return_quality_en_)
  {
    double intensity_ref = std::max(lidar_intensity_ref_, 1e-3);
    double intensity_safe = std::isfinite(intensity) ? std::max(intensity, 0.0) : 0.0;
    if (lidar_intensity_available_)
    {
      intensity_score = std::min(intensity_safe / intensity_ref, 1.0);
      if (lidar_saturation_ref_ > intensity_ref && intensity_safe > lidar_saturation_ref_)
      {
        intensity_score *= std::max(0.0, std::min(lidar_saturation_ref_ / std::max(intensity_safe, 1e-3), 1.0));
      }
    }

    double range_score = 1.0;
    double range_ref = std::max(lidar_range_adaptive_en_ ? lidar_range_ref_effective_ : lidar_range_ref_, 1e-3);
    if (!std::isfinite(range)) { range_score = 0.0; }
    else
    {
      double range_safe = std::max(range, 0.0);
      if (range_safe > range_ref) { range_score = range_ref / std::max(range_safe, range_ref); }
    }

    return_weight = lidar_intensity_available_ ?
        0.45 * intensity_score + 0.25 * range_score + 0.30 * density_clamped :
        (0.25 * range_score + 0.30 * density_clamped) / 0.55;
    if (apply_min_weight)
    {
      double min_weight = std::max(0.01, std::min(lidar_return_min_weight_, 1.0));
      return_weight = std::max(min_weight, std::min(return_weight, 1.0));
    }
    else
    {
      return_weight = std::max(0.0, std::min(return_weight, 1.0));
    }
  }
  if (intensity_score_out != nullptr) { *intensity_score_out = intensity_score; }
  if (density_score_out != nullptr) { *density_score_out = density_clamped; }
  return return_weight;
}

void VoxelMapManager::updateAdaptiveLidarRangeReference()
{
  const double base_ref = std::isfinite(lidar_range_ref_) ? std::max(lidar_range_ref_, 1e-3) : 35.0;
  const double quantile = std::isfinite(lidar_range_quantile_) ?
      std::max(0.50, std::min(lidar_range_quantile_, 0.95)) : 0.70;
  auto computeQuantile = [quantile](std::vector<double> &ranges) -> double {
    if (ranges.empty()) return -1.0;
    const size_t index = std::min(static_cast<size_t>(quantile * static_cast<double>(ranges.size() - 1)),
                                  ranges.size() - 1);
    std::nth_element(ranges.begin(), ranges.begin() + index, ranges.end());
    return ranges[index];
  };

  std::vector<double> &candidate_ranges = candidate_range_buffer_;
  candidate_ranges.clear();
  candidate_ranges.reserve(pv_list_.size());
  for (const auto &candidate : pv_list_)
  {
    const double range = candidate.range;
    if (std::isfinite(range) && range > 1e-3) candidate_ranges.push_back(range);
  }

  std::vector<double> &matched_ranges = matched_range_buffer_;
  matched_ranges.clear();
  matched_ranges.reserve(ptpl_list_.size());
  for (const auto &constraint : ptpl_list_)
  {
    const double range = static_cast<double>(constraint.range_);
    if (std::isfinite(range) && range > 1e-3) matched_ranges.push_back(range);
  }

  const size_t candidate_count = candidate_ranges.size();
  const size_t matched_count = matched_ranges.size();
  lidar_range_candidate_count_ = static_cast<int>(std::min(candidate_count,
      static_cast<size_t>(std::numeric_limits<int>::max())));
  lidar_range_matched_coverage_ = candidate_count > 0 ?
      std::max(0.0, std::min(static_cast<double>(matched_count) / static_cast<double>(candidate_count), 1.0)) : 0.0;
  lidar_range_candidate_quantile_ = computeQuantile(candidate_ranges);
  lidar_range_matched_quantile_ = computeQuantile(matched_ranges);
  lidar_range_selection_gap_ = lidar_range_candidate_quantile_ >= 0.0 && lidar_range_matched_quantile_ >= 0.0 ?
      lidar_range_candidate_quantile_ - lidar_range_matched_quantile_ : -1.0;

  if (!lidar_range_adaptive_en_)
  {
    lidar_range_ref_effective_ = base_ref;
    lidar_range_quantile_value_ = base_ref;
    lidar_range_ref_initialized_ = false;
    return;
  }

  const std::vector<double> &active_ranges = lidar_range_candidate_source_en_ ? candidate_ranges : matched_ranges;
  const double active_quantile = lidar_range_candidate_source_en_ ?
      lidar_range_candidate_quantile_ : lidar_range_matched_quantile_;
  if (active_ranges.size() < 20 || !std::isfinite(active_quantile) || active_quantile <= 0.0)
  {
    if (!lidar_range_ref_initialized_)
    {
      lidar_range_ref_effective_ = base_ref;
      lidar_range_quantile_value_ = base_ref;
    }
    return;
  }

  lidar_range_quantile_value_ = active_quantile;

  const double max_ref = std::isfinite(lidar_range_ref_max_) ? std::max(lidar_range_ref_max_, base_ref) : base_ref;
  const double target_ref = std::max(base_ref, std::min(lidar_range_quantile_value_, max_ref));
  const double alpha = std::isfinite(lidar_range_ref_ema_alpha_) ?
      std::max(0.01, std::min(lidar_range_ref_ema_alpha_, 1.0)) : 0.15;
  if (!std::isfinite(lidar_range_ref_effective_))
  {
    lidar_range_ref_effective_ = base_ref;
    lidar_range_ref_initialized_ = false;
  }
  if (!lidar_range_ref_initialized_)
  {
    lidar_range_ref_effective_ = base_ref;
    lidar_range_ref_initialized_ = true;
  }
  lidar_range_ref_effective_ += alpha * (target_ref - lidar_range_ref_effective_);
  lidar_range_ref_effective_ = std::max(base_ref, std::min(lidar_range_ref_effective_, max_ref));
}

double VoxelMapManager::computeDynamicConflictScore(const double residual_norm, const double plane_quality,
                                                    const double density_score) const
{
  if (!dynamic_object_filter_en_) { return 0.0; }
  double sigma = std::max(dynamic_residual_sigma_, 1e-3);
  double residual_safe = std::max(residual_norm, 0.0);
  double free_band = 0.5 * sigma;
  if (residual_safe <= free_band) { return 0.0; }
  double x = (residual_safe - free_band) / std::max(sigma - free_band, 1e-3);
  double residual_score = (x * x) / (1.0 + x * x);
  double structure_score = 0.5 * std::max(0.0, std::min(plane_quality, 1.0)) +
                           0.5 * std::max(0.0, std::min(density_score, 1.0));
  return std::max(0.0, std::min(residual_score * structure_score, 1.0));
}

void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
  if (plane == nullptr) return;
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
  plane->covariance_ = Eigen::Matrix3d::Zero();
  plane->center_ = Eigen::Vector3d::Zero();
  plane->normal_ = Eigen::Vector3d::Zero();
  plane->x_normal_ = Eigen::Vector3d::Zero();
  plane->y_normal_ = Eigen::Vector3d::Zero();
  plane->points_size_ = static_cast<int>(points.size());
  plane->radius_ = 0.0f;
  plane->d_ = 0.0f;
  plane->min_eigen_value_ = 0.0f;
  plane->mid_eigen_value_ = 0.0f;
  plane->max_eigen_value_ = 0.0f;
  plane->quality_score_ = 0.15f;
  plane->is_update_ = true;
  plane->is_plane_ = false;
  if (points.size() < 3 || !std::isfinite(planer_threshold_) || planer_threshold_ <= 0.0f) return;

  for (const auto &pv : points)
  {
    if (!pv.point_w.allFinite() || !pv.var.allFinite()) return;
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
    plane->center_ += pv.point_w;
  }
  const double point_count = static_cast<double>(points.size());
  plane->center_ /= point_count;
  plane->covariance_ = plane->covariance_ / point_count - plane->center_ * plane->center_.transpose();
  plane->covariance_ = 0.5 * (plane->covariance_ + plane->covariance_.transpose());
  if (!plane->center_.allFinite() || !plane->covariance_.allFinite()) return;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(plane->covariance_);
  if (solver.info() != Eigen::Success) return;
  const Eigen::Vector3d evals = solver.eigenvalues();
  const Eigen::Matrix3d evecs = solver.eigenvectors();
  if (!evals.allFinite() || !evecs.allFinite()) return;

  const double spectral_scale = std::max(std::abs(evals(2)), 1e-12);
  const double spectral_tol = std::max(1e-12, 1e-8 * spectral_scale);
  if (evals(0) < -spectral_tol || evals(1) - evals(0) <= spectral_tol) return;
  const double lambda_min = std::max(evals(0), 0.0);
  const double lambda_mid = std::max(evals(1), 0.0);
  const double lambda_max = std::max(evals(2), 0.0);
  // MODIFICATIONS: voxel-plane quality grading from eigenvalue ratios.
  // lambda_min / lambda_mid measures how "plate-like" the point distribution is.
  // Ratio < 0.05 → near-perfect plane (quality ≈ 1); ratio > 0.25 → poor (quality ≈ 0).
  // Downstream, low-quality planes have their constraints softly downweighted.
  {
    const float ratio = static_cast<float>(lambda_min / std::max(lambda_mid, spectral_tol));
    plane->quality_score_ = 1.0f - std::min(std::max(ratio, 0.0f) / 0.25f, 1.0f);
    plane->quality_score_ = std::max(std::min(plane->quality_score_, 1.0f), 0.15f);
  }
  const Eigen::Matrix3d J_Q = Eigen::Matrix3d::Identity() / point_count;
  // && evalsReal(evalsMid) > 0.05
  //&& evalsReal(evalsMid) > 0.01
  if (lambda_min < planer_threshold_)
  {
    for (size_t i = 0; i < points.size(); i++)
    {
      Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();
      Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
      for (int m = 0; m < 3; m++)
      {
        if (m != 0)
        {
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].point_w - plane->center_).transpose() / (point_count * (evals(0) - evals(m))) *
              (evecs.col(m) * evecs.col(0).transpose() + evecs.col(0) * evecs.col(m).transpose());
          F.row(m) = F_m;
        }
      }
      J.block<3, 3>(0, 0) = evecs * F;
      J.block<3, 3>(3, 0) = J_Q;
      plane->plane_var_ += J * points[i].var * J.transpose();
    }

    plane->plane_var_ = 0.5 * (plane->plane_var_ + plane->plane_var_.transpose());
    if (!plane->plane_var_.allFinite())
    {
      plane->plane_var_.setZero();
      return;
    }

    plane->normal_ = evecs.col(0);
    plane->y_normal_ = evecs.col(1);
    plane->x_normal_ = evecs.col(2);
    plane->min_eigen_value_ = static_cast<float>(lambda_min);
    plane->mid_eigen_value_ = static_cast<float>(lambda_mid);
    plane->max_eigen_value_ = static_cast<float>(lambda_max);
    plane->radius_ = static_cast<float>(std::sqrt(lambda_max));
    plane->d_ = static_cast<float>(-plane->normal_.dot(plane->center_));
    plane->is_plane_ = true;
    if (!plane->is_init_)
    {
      plane->id_ = voxel_plane_id;
      voxel_plane_id++;
      plane->is_init_ = true;
    }
  }
}

void VoxelOctoTree::init_octo_tree()
{
  if (temp_points_.size() > points_size_threshold_)
  {
    init_plane(temp_points_, plane_ptr_);
    if (plane_ptr_->is_plane_ == true)
    {
      octo_state_ = 0;
      // new added
      if (temp_points_.size() > max_points_num_)
      {
        update_enable_ = false;
        std::vector<pointWithVar>().swap(temp_points_);
        new_points_ = 0;
      }
    }
    else
    {
      octo_state_ = 1;
      cut_octo_tree();
    }
    init_octo_ = true;
    new_points_ = 0;
  }
}

void VoxelOctoTree::cut_octo_tree()
{
  if (layer_ >= max_layer_)
  {
    octo_state_ = 0;
    return;
  }
  for (size_t i = 0; i < temp_points_.size(); i++)
  {
    int xyz[3] = {0, 0, 0};
    if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
    if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
    if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] == nullptr)
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
    }
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    leaves_[leafnum]->new_points_++;
  }
  for (uint i = 0; i < 8; i++)
  {
    if (leaves_[i] != nullptr)
    {
      if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_)
      {
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
        if (leaves_[i]->plane_ptr_->is_plane_)
        {
          leaves_[i]->octo_state_ = 0;
          // new added
          if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_)
          {
            leaves_[i]->update_enable_ = false;
            std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
            new_points_ = 0;
          }
        }
        else
        {
          leaves_[i]->octo_state_ = 1;
          leaves_[i]->cut_octo_tree();
        }
        leaves_[i]->init_octo_ = true;
        leaves_[i]->new_points_ = 0;
      }
    }
  }
}

void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv)
{
  if (!init_octo_)
  {
    new_points_++;
    temp_points_.push_back(pv);
    if (temp_points_.size() > points_size_threshold_) { init_octo_tree(); }
  }
  else
  {
    if (plane_ptr_->is_plane_)
    {
      if (update_enable_)
      {
        new_points_++;
        temp_points_.push_back(pv);
        if (new_points_ > update_size_threshold_)
        {
          init_plane(temp_points_, plane_ptr_);
          new_points_ = 0;
        }
        if (temp_points_.size() >= max_points_num_)
        {
          update_enable_ = false;
          std::vector<pointWithVar>().swap(temp_points_);
          new_points_ = 0;
        }
      }
    }
    else
    {
      if (layer_ < max_layer_)
      {
        int xyz[3] = {0, 0, 0};
        if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
        if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
        if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) { leaves_[leafnum]->UpdateOctoTree(pv); }
        else
        {
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
          leaves_[leafnum]->layer_init_num_ = layer_init_num_;
          leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
          leaves_[leafnum]->quater_length_ = quater_length_ / 2;
          leaves_[leafnum]->UpdateOctoTree(pv);
        }
      }
      else
      {
        if (update_enable_)
        {
          new_points_++;
          temp_points_.push_back(pv);
          if (new_points_ > update_size_threshold_)
          {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
          }
          if (temp_points_.size() > max_points_num_)
          {
            update_enable_ = false;
            std::vector<pointWithVar>().swap(temp_points_);
            new_points_ = 0;
          }
        }
      }
    }
  }
}

VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw)
{
  if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

  int xyz[3] = {0, 0, 0};
  xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
  xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
  xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
  int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

  // printf("leafnum: %d. \n", leafnum);

  return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv)
{
  if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) || (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_)))
  {
    new_points_++;
    temp_points_.push_back(pv);
    return this;
  }

  if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_))
  {
    int xyz[3] = {0, 0, 0};
    xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] != nullptr) { return leaves_[leafnum]->Insert(pv); }
    else
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      return leaves_[leafnum]->Insert(pv);
    }
  }
  return nullptr;
}

void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{
  const StatesGroup state_before_estimation(state_);
  StatesGroup rollback_state(state_before_estimation);
  const double range_ref_before_estimation = lidar_range_ref_effective_;
  const double range_quantile_before_estimation = lidar_range_quantile_value_;
  const bool range_initialized_before_estimation = lidar_range_ref_initialized_;
  state_estimation_valid_ = true;
  state_prior_invalid_ = false;
  degeneracy_numerical_fallback_ = false;
  degeneracy_factor_ = 1.0;
  degeneracy_mean_factor_ = 1.0;
  degeneracy_eigenvalues_ = Eigen::Matrix<double, 6, 1>::Ones();
  degeneracy_direction_factors_ = Eigen::Matrix<double, 6, 1>::Ones();
  degeneracy_state_directions_ = Eigen::Matrix<double, 6, 6>::Identity();
  degeneracy_whitened_min_eigenvalue_ = 0.0;
  degeneracy_whitened_max_eigenvalue_ = 0.0;
  degeneracy_whitened_condition_ratio_ = 1.0;
  degeneracy_rot_trans_coupling_ = 0.0;
  degeneracy_coupled_active_ = false;
  covariance_relinearization_angle_ = 0.0;
  covariance_reset_angle_ = 0.0;
  covariance_min_eigenvalue_ = -1.0;
  covariance_symmetry_error_ = 0.0;
  auto abortStateEstimation = [&](const char *reason) {
    ROS_WARN_THROTTLE(1.0, "%s", reason);
    state_ = rollback_state;
    lidar_range_ref_effective_ = range_ref_before_estimation;
    lidar_range_quantile_value_ = range_quantile_before_estimation;
    lidar_range_ref_initialized_ = range_initialized_before_estimation;
    state_estimation_valid_ = false;
    degeneracy_numerical_fallback_ = true;
    covariance_relinearization_angle_ = 0.0;
    covariance_reset_angle_ = 0.0;
    covariance_min_eigenvalue_ = -1.0;
    covariance_symmetry_error_ = 0.0;
  };

  // Keep a single propagated prior for all IEKF iterations. Each iteration
  // transports this frozen covariance into its own rotation tangent space;
  // posterior information is committed only once at the final iteration.
  MD(DIM_STATE, DIM_STATE) propagated_cov, propagated_precision_unused;
  const bool prior_mean_finite = state_before_estimation.rot_end.allFinite() &&
                                 state_before_estimation.pos_end.allFinite() &&
                                 state_before_estimation.vel_end.allFinite() &&
                                 state_before_estimation.bias_g.allFinite() &&
                                 state_before_estimation.bias_a.allFinite() &&
                                 state_before_estimation.gravity.allFinite() &&
                                 std::isfinite(state_before_estimation.inv_expo_time) &&
                                 state_propagat.rot_end.allFinite() && state_propagat.pos_end.allFinite() &&
                                 state_propagat.vel_end.allFinite() && state_propagat.bias_g.allFinite() &&
                                 state_propagat.bias_a.allFinite() && state_propagat.gravity.allFinite() &&
                                 std::isfinite(state_propagat.inv_expo_time) && state_propagat.cov.allFinite();
  if (!prior_mean_finite)
  {
    state_prior_invalid_ = true;
    abortStateEstimation("[ LIO ] Invalid propagated state; coordinated reset required");
    return;
  }
  if (!regularizeAndInvertSymmetric<DIM_STATE>(state_before_estimation.cov, 1e-12,
                                                 propagated_cov, propagated_precision_unused))
  {
    state_prior_invalid_ = true;
    abortStateEstimation("[ LIO ] Invalid propagated covariance; coordinated reset required");
    return;
  }
  rollback_state.cov = propagated_cov;

  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  body_cov_list_.clear();
  body_cov_list_.reserve(feats_down_size_);

  // build_residual_time = 0.0;
  // ekf_time = 0.0;
  // double t0 = omp_get_wtime();

  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    if (point_this[2] == 0) { point_this[2] = 0.001; }
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    body_cov_list_.push_back(var);
    point_this = extR_ * point_this + extT_;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }

  pv_list_.clear();
  // Use the cloud itself as the indexing authority. A stale external count must not
  // make the per-point metadata vector smaller than the loops below.
  pv_list_.resize(feats_down_body_->size());

  int rematch_num = 0;
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  bool flg_EKF_inited = false, flg_EKF_converged = false, EKF_stop_flg = false;
  for (int iterCount = 0; iterCount < config_setting_.max_iterations_; iterCount++)
  {
    const VD(DIM_STATE) iteration_delta = state_ - state_propagat;
    if (!iteration_delta.allFinite())
    {
      abortStateEstimation("[ LIO ] Non-finite ESIKF relinearization delta; roll back this ESIKF frame");
      break;
    }
    const Eigen::Matrix3d iteration_rotation_transport =
        RightJacobianSO3(iteration_delta.block<3, 1>(0, 0));
    if (!iteration_rotation_transport.allFinite())
    {
      abortStateEstimation("[ LIO ] Invalid ESIKF prior tangent transport; roll back this ESIKF frame");
      break;
    }
    MD(DIM_STATE, DIM_STATE) prior_transport = I_STATE;
    prior_transport.block<3, 3>(0, 0) = iteration_rotation_transport;
    const MD(DIM_STATE, DIM_STATE) prior_cov_candidate =
        prior_transport * propagated_cov * prior_transport.transpose();
    MD(DIM_STATE, DIM_STATE) prior_cov, prior_precision;
    if (!regularizeAndInvertSymmetric<DIM_STATE>(prior_cov_candidate, 1e-12,
                                                  prior_cov, prior_precision))
    {
      abortStateEstimation("[ LIO ] Invalid transported prior covariance; roll back this ESIKF frame");
      break;
    }
    covariance_relinearization_angle_ = iteration_delta.block<3, 1>(0, 0).norm();

    double total_residual = 0.0;
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_,
                   state_estimation_world_lidar_scratch_);
    M3D rot_var = prior_cov.block<3, 3>(0, 0);
    M3D t_var = prior_cov.block<3, 3>(3, 3);
    for (size_t i = 0; i < feats_down_body_->size(); i++)
    {
      pointWithVar &pv = pv_list_[i];
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.point_w << state_estimation_world_lidar_scratch_->points[i].x,
          state_estimation_world_lidar_scratch_->points[i].y,
          state_estimation_world_lidar_scratch_->points[i].z;
      const double point_time = feats_down_body_->points[i].curvature / 1000.0;
      const double point_intensity = static_cast<double>(feats_down_body_->points[i].intensity);
      pv.point_offset_time = std::isfinite(point_time) ? std::max(point_time, 0.0) : 0.0;
      pv.intensity = std::isfinite(point_intensity) ? std::max(point_intensity, 0.0) : 0.0;
      pv.range = pv.point_b.allFinite() ? pv.point_b.norm() : 0.0;
      pv.map_match_success = false;
      pv.map_residual_norm = 0.0;
      pv.map_plane_quality = 1.0;
      pv.map_density_score = 1.0;
      pv.map_return_score = computeLidarReturnWeight(pv.intensity, pv.range, pv.map_density_score, nullptr, nullptr, false);
      pv.map_dynamic_score = 0.0;
      pv.map_write_score = 1.0;

      M3D cov = body_cov_list_[i];
      M3D point_crossmat = cross_mat_list_[i];
      const M3D sensor_rotation = state_.rot_end * extR_;
      const M3D rotation_jacobian = -state_.rot_end * point_crossmat;
      const M3D rot_pos_cov = prior_cov.block<3, 3>(0, 3);
      cov = sensor_rotation * cov * sensor_rotation.transpose() +
            rotation_jacobian * rot_var * rotation_jacobian.transpose() + t_var +
            rotation_jacobian * rot_pos_cov + rot_pos_cov.transpose() * rotation_jacobian.transpose();
      cov = 0.5 * (cov + cov.transpose());
      pv.var = cov;
      pv.body_var = body_cov_list_[i];
    }
    ptpl_list_.clear();

    // double t1 = omp_get_wtime();

    BuildResidualListOMP(pv_list_, ptpl_list_);

    // build_residual_time += omp_get_wtime() - t1;

    for (int i = 0; i < ptpl_list_.size(); i++)
    {
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
    effct_feat_num_ = ptpl_list_.size();
    avg_residual_ = effct_feat_num_ > 0 ? total_residual / effct_feat_num_ : -1.0;
    if (iterCount == 0) updateAdaptiveLidarRangeReference();
    total_lio_effective_weight_mean_ = 1.0;
    total_lio_low_weight_ratio_ = 0.0;

    // MODIFICATIONS: Innovation 4 — anisotropy of the point-to-plane constraint normals.
    // When the UAV only sees one dominant plane (ground / wall / corridor) all normals
    // align, lambda_min/lambda_max of the scatter matrix drops towards 0, and translation
    // along the plane becomes unobservable before it even shows up in H^T H.
    {
      Eigen::Matrix3d normal_scatter = Eigen::Matrix3d::Zero();
      int valid_normal_count = 0;
      for (int i = 0; i < effct_feat_num_; i++)
      {
        const V3D &normal = ptpl_list_[i].normal_;
        if (!normal.allFinite() || normal.squaredNorm() <= 1e-18) continue;
        normal_scatter += normal * normal.transpose();
        valid_normal_count++;
      }
      if (valid_normal_count > 0)
      {
        normal_scatter /= (double)valid_normal_count;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> n_eig(normal_scatter);
        if (n_eig.info() == Eigen::Success && n_eig.eigenvalues().allFinite())
        {
          double lam_n_max = n_eig.eigenvalues().maxCoeff();
          normal_anisotropy_ = (lam_n_max > 1e-10) ?
              std::max(0.0, std::min(n_eig.eigenvalues().minCoeff() / lam_n_max, 1.0)) : 0.0;
        }
        else { normal_anisotropy_ = 0.0; }
      }
      else { normal_anisotropy_ = 0.0; }
    }
    // MODIFICATIONS: Innovation 4 — closed-loop: when constraint normals are highly
    // collinear (single large plane / long corridor), each additional point-to-plane
    // residual provides diminishing independent information. Inflate the measurement
    // noise floor so the filter does not over-weight a batch of near-identical constraints.
    // Guard: at least 10 constraints required for a meaningful anisotropy estimate.
    double anisotropy_correction_ = 1.0;
    const bool coupled_direction_model_active = degeneracy_aware_en_ && coupled_degeneracy_en_;
    if (normal_anisotropy_weight_en_ && !coupled_direction_model_active &&
        effct_feat_num_ >= 10 && normal_anisotropy_ < 0.30)
    {
      anisotropy_correction_ = 1.0 + 5.0 * (1.0 - normal_anisotropy_ / 0.30);
      if (anisotropy_correction_ > 6.0) anisotropy_correction_ = 6.0;
    }

    // MODIFICATIONS: Innovation 9 - normal-direction balance.
    // A UAV often sees many more ground/facade constraints than vertical/edge-like
    // constraints. The global anisotropy correction above reduces over-confidence, but
    // it still lets the dominant direction flood H^T R^-1 H. Here we bin unsigned plane
    // normals and softly cap over-represented bins so rare directions keep leverage.
    normal_bin_ids_buffer_.assign(effct_feat_num_, -1);
    std::vector<int> &normal_bin_ids = normal_bin_ids_buffer_;
    std::array<int, 13> normal_bin_counts{};
    dominant_normal_ratio_ = 0.0;
    normal_balance_mean_weight_ = 1.0;
    if (effct_feat_num_ >= 10)
    {
      const auto &normal_bins = normalDirectionBins();
      for (int i = 0; i < effct_feat_num_; i++)
      {
        V3D n = ptpl_list_[i].normal_;
        double n_norm = n.norm();
        if (!n.allFinite() || !std::isfinite(n_norm) || n_norm <= 1e-9) continue;
        n /= n_norm;

        int best_bin = 0;
        double best_score = -1.0;
        for (int b = 0; b < (int)normal_bins.size(); b++)
        {
          double score = fabs(n.dot(normal_bins[b]));
          if (score > best_score)
          {
            best_score = score;
            best_bin = b;
          }
        }
        normal_bin_ids[i] = best_bin;
        normal_bin_counts[best_bin]++;
      }

      int dominant_count = 0;
      for (int count : normal_bin_counts) dominant_count = std::max(dominant_count, count);
      dominant_normal_ratio_ = (effct_feat_num_ > 0) ? (double)dominant_count / (double)effct_feat_num_ : 0.0;
    }

    if (verbose_runtime_log_)
    {
      cout << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_
           << " effective feature num: " << effct_feat_num_ << " average residual: " << avg_residual_ << endl;
    }

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
    MatrixXd Hsub(effct_feat_num_, 6);
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_);
    VectorXd R_inv(effct_feat_num_);
    VectorXd meas_vec(effct_feat_num_);
    meas_vec.setZero();
    double incidence_weight_sum = 0.0;
    int incidence_weight_count = 0;
    double normal_balance_weight_sum = 0.0;
    int normal_balance_weight_count = 0;
    double robust_kernel_weight_sum = 0.0;
    int robust_kernel_weight_count = 0;
    double scan_distortion_weight_sum = 0.0;
    int scan_distortion_weight_count = 0;
    double lidar_return_weight_sum = 0.0;
    double lidar_intensity_score_sum = 0.0;
    double lidar_density_score_sum = 0.0;
    int lidar_return_weight_count = 0;
    double dynamic_score_sum = 0.0;
    double dynamic_weight_sum = 0.0;
    int dynamic_weight_count = 0;
    double residual_consistency_weight_sum = 0.0;
    int residual_consistency_weight_count = 0;
    lio_hard_outlier_reject_num_ = 0;
    double total_lio_effective_weight_sum = 0.0;
    int total_lio_low_weight_count = 0;
    int total_lio_weight_count = 0;
    int valid_lio_constraint_count = 0;
    for (int i = 0; i < effct_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];
      auto clearInvalidConstraint = [&]() {
        Hsub.row(i).setZero();
        Hsub_T_R_inv.col(i).setZero();
        R_inv(i) = 0.0;
        meas_vec(i) = 0.0;
      };
      if (!ptpl.point_b_.allFinite() || !ptpl.normal_.allFinite() || !ptpl.body_cov_.allFinite() ||
          !ptpl.plane_var_.allFinite() || !std::isfinite(ptpl.dis_to_plane_))
      {
        clearInvalidConstraint();
        continue;
      }
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/

      V3D point_world = state_.rot_end * point_this + state_.pos_end;
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;

      M3D var;
      // V3D normal_b = state_.rot_end.inverse() * ptpl_list_[i].normal_;
      // V3D point_b = ptpl_list_[i].point_b_;
      // double cos_theta = fabs(normal_b.dot(point_b) / point_b.norm());
      // ptpl_list_[i].body_cov_ = ptpl_list_[i].body_cov_ * (1.0 / cos_theta) * (1.0 / cos_theta);

      // point_w cov
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) + (-point_crossmat) * state_propagat.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose();

      // point_w cov (another_version)
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) - point_crossmat * state_propagat.cov.block<3, 3>(0, 0) * point_crossmat;

      // point_body cov
      var = state_.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_.rot_end * extR_).transpose();

      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();

      double sigma_body = ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_;
      if (!std::isfinite(sigma_l) || !std::isfinite(sigma_body))
      {
        clearInvalidConstraint();
        continue;
      }
      double residual_var = std::max(0.001 + sigma_l + sigma_body, 1e-12);
      if (!std::isfinite(residual_var))
      {
        clearInvalidConstraint();
        continue;
      }
      R_inv(i) = 1.0 / (anisotropy_correction_ * residual_var);
      const double base_precision = std::max(R_inv(i), 1e-12);
      // R_inv(i) = 1.0 / (sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      double normal_balance_weight = 1.0;
      if (normal_balance_en_ && effct_feat_num_ >= 10 &&
          i < (int)normal_bin_ids.size() && normal_bin_ids[i] >= 0 &&
          normal_bin_ids[i] < (int)normal_bin_counts.size())
      {
        int bin_count = normal_bin_counts[normal_bin_ids[i]];
        int target_count = std::max(1, (int)std::ceil(normal_balance_target_fraction_ * (double)effct_feat_num_));
        if (bin_count > target_count)
        {
          double raw_weight = (double)target_count / (double)bin_count;
          double min_weight = std::max(0.01, std::min(normal_balance_min_weight_, 1.0));
          normal_balance_weight = std::max(min_weight, std::min(raw_weight, 1.0));
        }
        R_inv(i) *= normal_balance_weight;
      }
      normal_balance_weight_sum += normal_balance_weight;
      normal_balance_weight_count++;

      // MODIFICATIONS: Innovation 7 - LiDAR incidence-angle weighting.
      // Grazing-angle returns on ground/facade planes are noisier and more likely to
      // create biased point-to-plane constraints. The ray and plane normal are compared
      // in the world frame, then the constraint precision is reduced before H^T R^-1 H.
      double incidence_weight = 1.0;
      if (incidence_weight_en_)
      {
        V3D ray_body = extR_ * ptpl.point_b_;
        V3D ray_w = state_.rot_end * ray_body;
        double ray_norm = ray_w.norm();
        double normal_norm = ptpl.normal_.norm();
        if (ray_norm > 1e-9 && normal_norm > 1e-9)
        {
          double cos_incidence = fabs(ray_w.dot(ptpl.normal_) / (ray_norm * normal_norm));
          cos_incidence = std::max(0.0, std::min(cos_incidence, 1.0));
          double power = std::max(incidence_weight_power_, 0.1);
          incidence_weight = std::pow(cos_incidence, power);
          double min_weight = std::max(0.0, std::min(incidence_min_weight_, 1.0));
          incidence_weight = std::max(incidence_weight, min_weight);
        }
        R_inv(i) *= incidence_weight;
      }
      incidence_weight_sum += incidence_weight;
      incidence_weight_count++;

      // MODIFICATIONS: Innovation 10 - residual robust kernel.
      // Dynamic objects, rolling-shutter/scan distortion and badly fitted local planes
      // all first appear as oversized point-to-plane residuals. A continuous robust
      // kernel avoids the abrupt 5-sigma hard switch and feeds a smoother precision
      // weight into H^T R^-1 H.
      double robust_kernel_weight = 1.0;
      double normalized_residual = 0.0;
      {
        double sigma_total = sqrt(residual_var);
        double delta = std::max(robust_kernel_delta_, 1e-3);
        normalized_residual = (sigma_total > 1e-6) ?
            fabs(ptpl_list_[i].dis_to_plane_) / sigma_total : 0.0;
        if (robust_kernel_en_ && sigma_total > 1e-6)
        {
          if (robust_kernel_type_ == "huber")
          {
            robust_kernel_weight = (normalized_residual <= delta) ?
                1.0 : delta / std::max(normalized_residual, 1e-6);
          }
          else
          {
            double x = normalized_residual / delta;
            robust_kernel_weight = 1.0 / (1.0 + x * x);
          }
          double min_weight = std::max(0.0, std::min(robust_kernel_min_weight_, 1.0));
          robust_kernel_weight = std::max(min_weight, std::min(robust_kernel_weight, 1.0));
        }
      }
      robust_kernel_weight_sum += robust_kernel_weight;
      robust_kernel_weight_count++;

      // MODIFICATIONS: Innovation 16 - dynamic/transient object weighting.
      // A point that conflicts with a stable voxel plane is likely a moving object,
      // vegetation/dust, or another transient surface. Reduce its precision before
      // it contributes to H^T R^-1 H, while keeping a minimum weight for stability.
      double dynamic_score = std::max(0.0, std::min((double)ptpl.dynamic_score_, 1.0));
      double dynamic_weight = 1.0;
      if (dynamic_object_filter_en_)
      {
        const double motion_risk = std::max(0.0, std::min(map_write_motion_risk_, 1.0));
        const double classification_confidence =
            1.0 / (1.0 + std::max(dynamic_motion_gain_, 0.0) * motion_risk);
        dynamic_score = std::max(0.0, std::min(dynamic_score * classification_confidence, 1.0));
        dynamic_weight = 1.0 - dynamic_score;
        dynamic_weight = std::max(0.0, std::min(dynamic_weight, 1.0));
      }
      dynamic_score_sum += dynamic_score;
      dynamic_weight_sum += dynamic_weight;
      dynamic_weight_count++;

      // Dynamic classification is retained for hard conflict rejection and map
      // quarantine, but it no longer continuously downweights the same residual
      // a second time.  The single robust kernel is the residual precision model.
      const double residual_consistency_weight = robust_kernel_weight;
      residual_consistency_weight_sum += residual_consistency_weight;
      residual_consistency_weight_count++;
      const double hard_residual_thresh = std::max(lio_hard_outlier_residual_sigma_, 1.0);
      const double hard_dynamic_thresh = std::max(0.0, std::min(lio_hard_outlier_dynamic_score_, 1.0));
      if (lio_hard_outlier_reject_en_ && dynamic_object_filter_en_ &&
          normalized_residual >= hard_residual_thresh && dynamic_score >= hard_dynamic_thresh)
      {
        lio_hard_outlier_reject_num_++;
        clearInvalidConstraint();
        continue;
      }
      R_inv(i) *= residual_consistency_weight;

      // MODIFICATIONS: Innovation 11 - scan-distortion-aware LIO weighting.
      // Points acquired early in a high-motion scan require the largest backward
      // compensation and are most exposed to residual deskew errors. Shrink their
      // precision smoothly before they enter H^T R^-1 H.
      double scan_distortion_weight = 1.0;
      if (scan_distortion_weight_en_)
      {
        double risk = std::max(0.0, std::min(scan_distortion_risk_, 1.0));
        double local_risk = risk;
        if (scan_distortion_time_span_ > 1e-4)
        {
          double time_to_end = (scan_distortion_time_span_ - std::max((double)ptpl.point_offset_time_, 0.0)) / scan_distortion_time_span_;
          time_to_end = std::max(0.0, std::min(time_to_end, 1.0));
          local_risk = risk * (0.35 + 0.65 * time_to_end);
        }
        double gain = std::max(scan_distortion_noise_gain_, 0.0);
        scan_distortion_weight = 1.0 / (1.0 + gain * local_risk * local_risk);
        double min_weight = std::max(0.01, std::min(scan_distortion_min_weight_, 1.0));
        scan_distortion_weight = std::max(min_weight, std::min(scan_distortion_weight, 1.0));
        R_inv(i) *= scan_distortion_weight;
      }
      scan_distortion_weight_sum += scan_distortion_weight;
      scan_distortion_weight_count++;

      // MODIFICATIONS: Innovation 12 - LiDAR return-quality weighting.
      // Weak returns, saturated/specular returns, very far points and sparsely
      // supported local planes are less reliable point-to-plane constraints. Convert
      // those signals into a continuous precision weight before building H^T R^-1 H.
      double lidar_return_weight = 1.0;
      double intensity_score = 1.0;
      double density_score = 1.0;
      lidar_return_weight = computeLidarReturnWeight(ptpl.intensity_, ptpl.range_, ptpl.density_score_,
                                                     &intensity_score, &density_score);
      R_inv(i) *= lidar_return_weight;
      lidar_return_weight_sum += lidar_return_weight;
      lidar_intensity_score_sum += intensity_score;
      lidar_density_score_sum += density_score;
      lidar_return_weight_count++;

      // MODIFICATIONS: downweight constraints from low-quality voxel planes.
      // Planes fitted to noisy / sparse / non-planar point clusters produce
      // unreliable normals and distances; their constraints should carry less
      // weight than well-conditioned planes (quality near 1.0).
      if (plane_quality_weight_en_)
      {
        const double plane_quality = std::isfinite(ptpl_list_[i].plane_quality_) ?
                                         std::max(0.0, std::min((double)ptpl_list_[i].plane_quality_, 1.0)) :
                                         0.0;
        R_inv(i) *= plane_quality;
      }
      double effective_weight = R_inv(i) / base_precision;
      double min_total_weight = std::max(0.0, std::min(total_lio_min_weight_, 1.0));
      if (min_total_weight > 0.0)
      {
        effective_weight = std::max(effective_weight, min_total_weight);
        R_inv(i) = base_precision * effective_weight;
      }
      total_lio_effective_weight_sum += effective_weight;
      total_lio_weight_count++;
      if (effective_weight <= std::max(0.05, min_total_weight * 1.5)) { total_lio_low_weight_count++; }

      /*** calculate the Measuremnt Jacobian matrix H ***/
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
      if (!std::isfinite(R_inv(i)) || R_inv(i) <= 0.0 || !Hsub.row(i).allFinite() ||
          !Hsub_T_R_inv.col(i).allFinite() || !std::isfinite(meas_vec(i)))
      {
        clearInvalidConstraint();
        continue;
      }
      valid_lio_constraint_count++;
    }
    incidence_weight_mean_ = (incidence_weight_count > 0) ? incidence_weight_sum / incidence_weight_count : 1.0;
    normal_balance_mean_weight_ = (normal_balance_weight_count > 0) ? normal_balance_weight_sum / normal_balance_weight_count : 1.0;
    robust_kernel_mean_weight_ = (robust_kernel_weight_count > 0) ? robust_kernel_weight_sum / robust_kernel_weight_count : 1.0;
    scan_distortion_mean_weight_ = (scan_distortion_weight_count > 0) ? scan_distortion_weight_sum / scan_distortion_weight_count : 1.0;
    lidar_return_quality_mean_weight_ = (lidar_return_weight_count > 0) ? lidar_return_weight_sum / lidar_return_weight_count : 1.0;
    lidar_return_intensity_score_ = (lidar_return_weight_count > 0) ? lidar_intensity_score_sum / lidar_return_weight_count : 1.0;
    lidar_return_density_score_ = (lidar_return_weight_count > 0) ? lidar_density_score_sum / lidar_return_weight_count : 1.0;
    dynamic_mean_score_ = (dynamic_weight_count > 0) ? dynamic_score_sum / dynamic_weight_count : 0.0;
    dynamic_mean_weight_ = (dynamic_weight_count > 0) ? dynamic_weight_sum / dynamic_weight_count : 1.0;
    residual_consistency_mean_weight_ = (residual_consistency_weight_count > 0) ?
        residual_consistency_weight_sum / residual_consistency_weight_count : 1.0;
    total_lio_effective_weight_mean_ = (total_lio_weight_count > 0) ? total_lio_effective_weight_sum / total_lio_weight_count : 1.0;
    total_lio_low_weight_ratio_ = (total_lio_weight_count > 0) ? (double)total_lio_low_weight_count / total_lio_weight_count : 0.0;
    if (valid_lio_constraint_count == 0)
    {
      abortStateEstimation("[ LIO ] No usable point-to-plane constraint; roll back and block map writing");
      break;
    }
    EKF_stop_flg = false;
    flg_EKF_converged = false;
    /*** Iterative Kalman Filter Update ***/
    Eigen::Matrix<double, 6, 1> HTz = Hsub_T_R_inv * meas_vec;
    Eigen::Matrix<double, 6, 6> pose_information = Hsub_T_R_inv * Hsub;
    pose_information = 0.5 * (pose_information + pose_information.transpose());
    if (!pose_information.allFinite() || !HTz.allFinite())
    {
      abortStateEstimation("[ LIO ] Non-finite measurement information; roll back this ESIKF frame");
      break;
    }

    H_T_H.setZero();
    H_T_H.block<6, 6>(0, 0) = pose_information;
    Eigen::Matrix<double, 6, 6> D_degenerate = Eigen::Matrix<double, 6, 6>::Identity();
    MD(DIM_STATE, DIM_STATE) state_projection = I_STATE;
    bool legacy_projection_active = false;
    degeneracy_factor_ = 1.0;
    degeneracy_mean_factor_ = 1.0;
    degeneracy_eigenvalues_ = Eigen::Matrix<double, 6, 1>::Ones();
    degeneracy_whitened_min_eigenvalue_ = 0.0;
    degeneracy_whitened_max_eigenvalue_ = 0.0;
    degeneracy_whitened_condition_ratio_ = 1.0;
    degeneracy_coupled_active_ = false;

    const double rot_information_norm = pose_information.block<3, 3>(0, 0).norm();
    const double trans_information_norm = pose_information.block<3, 3>(3, 3).norm();
    const double coupling_denom = std::sqrt(std::max(rot_information_norm, 0.0)) *
                                  std::sqrt(std::max(trans_information_norm, 0.0));
    degeneracy_rot_trans_coupling_ = (coupling_denom > 1e-12) ?
        std::max(0.0, std::min(pose_information.block<3, 3>(0, 3).norm() / coupling_denom, 1.0)) : 0.0;

    if (degeneracy_aware_en_ && effct_feat_num_ >= 15)
    {
      bool coupled_valid = false;
      if (coupled_degeneracy_en_)
      {
        Eigen::Matrix<double, 6, 6> pose_prior, pose_prior_inverse;
        if (regularizeAndInvertSymmetric<6>(prior_cov.block<6, 6>(0, 0),
                                            degeneracy_prior_eigen_ratio_floor_,
                                            pose_prior, pose_prior_inverse))
        {
          Eigen::LLT<Eigen::Matrix<double, 6, 6>> prior_llt(pose_prior);
          if (prior_llt.info() == Eigen::Success)
          {
            const Eigen::Matrix<double, 6, 6> L = prior_llt.matrixL();
            const Eigen::Matrix<double, 6, 6> L_inverse =
                L.triangularView<Eigen::Lower>().solve(Eigen::Matrix<double, 6, 6>::Identity());
            Eigen::Matrix<double, 6, 6> whitened_information = L.transpose() * pose_information * L;
            whitened_information = 0.5 * (whitened_information + whitened_information.transpose());
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig_solver(whitened_information);
            if (L_inverse.allFinite() && eig_solver.info() == Eigen::Success &&
                eig_solver.eigenvalues().allFinite() && eig_solver.eigenvectors().allFinite())
            {
              Eigen::Matrix<double, 6, 1> eig_vals = eig_solver.eigenvalues();
              const double spectrum_scale = std::max(std::fabs(eig_vals.maxCoeff()), 1.0);
              if (eig_vals.minCoeff() >= -1e-9 * spectrum_scale)
              {
                eig_vals = eig_vals.cwiseMax(0.0);
                const double lambda_max = eig_vals.maxCoeff();
                Eigen::Matrix<double, 6, 1> d_factors;
                const double min_factor = std::max(0.0, std::min(degeneracy_min_factor_, 1.0));
                for (int k = 0; k < 6; k++)
                {
                  const double relative_ratio = (lambda_max > 1e-12) ? eig_vals(k) / lambda_max : 0.0;
                  const double relative_health = normalizedSigmoidHealth(relative_ratio, degeneracy_alpha_, degeneracy_tau_);
                  const double absolute_health = eig_vals(k) / (1.0 + eig_vals(k));
                  const double health = std::max(relative_health, absolute_health);
                  d_factors(k) = min_factor + (1.0 - min_factor) * std::max(0.0, std::min(health, 1.0));
                }

                const Eigen::Matrix<double, 6, 6> eigenvectors = eig_solver.eigenvectors();
                const Eigen::Matrix<double, 6, 1> weighted_eigenvalues = d_factors.array() * eig_vals.array();
                const Eigen::Matrix<double, 6, 6> weighted_information =
                    eigenvectors * weighted_eigenvalues.asDiagonal() * eigenvectors.transpose();
                const Eigen::Matrix<double, 6, 1> whitened_gradient = L.transpose() * HTz;
                const Eigen::Matrix<double, 6, 1> weighted_gradient =
                    eigenvectors * d_factors.asDiagonal() * eigenvectors.transpose() * whitened_gradient;
                Eigen::Matrix<double, 6, 6> effective_information =
                    L_inverse.transpose() * weighted_information * L_inverse;
                const Eigen::Matrix<double, 6, 1> effective_gradient = L_inverse.transpose() * weighted_gradient;
                effective_information = 0.5 * (effective_information + effective_information.transpose());
                if (effective_information.allFinite() && effective_gradient.allFinite())
                {
                  H_T_H.block<6, 6>(0, 0) = effective_information;
                  HTz = effective_gradient;
                  degeneracy_eigenvalues_ = eig_vals;
                  degeneracy_direction_factors_ = d_factors;
                  degeneracy_state_directions_ = L * eigenvectors;
                  for (int direction = 0; direction < 6; ++direction)
                  {
                    const double direction_norm = degeneracy_state_directions_.col(direction).norm();
                    if (direction_norm > 1e-12)
                    {
                      degeneracy_state_directions_.col(direction) /= direction_norm;
                    }
                  }
                  degeneracy_factor_ = d_factors.minCoeff();
                  degeneracy_mean_factor_ = d_factors.mean();
                  degeneracy_whitened_min_eigenvalue_ = eig_vals.minCoeff();
                  degeneracy_whitened_max_eigenvalue_ = lambda_max;
                  degeneracy_whitened_condition_ratio_ =
                      (lambda_max > 1e-12) ? eig_vals.minCoeff() / lambda_max : 0.0;
                  degeneracy_coupled_active_ = true;
                  coupled_valid = true;
                }
              }
            }
          }
        }
      }

      // The old split-block path is retained for direct ablation, but its
      // post-projected gain is paired with a Joseph covariance update below.
      if (!coupled_degeneracy_en_ || !coupled_valid)
      {
        if (coupled_degeneracy_en_) degeneracy_numerical_fallback_ = true;
        Eigen::Matrix<double, 6, 1> d_factors = Eigen::Matrix<double, 6, 1>::Ones();
        bool legacy_valid = true;
        for (int b = 0; b < 2; b++)
        {
          const Eigen::Matrix3d block_information = 0.5 *
              (pose_information.block<3, 3>(3 * b, 3 * b) + pose_information.block<3, 3>(3 * b, 3 * b).transpose());
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(block_information);
          if (eig_solver.info() != Eigen::Success || !eig_solver.eigenvalues().allFinite() ||
              !eig_solver.eigenvectors().allFinite())
          {
            legacy_valid = false;
            break;
          }
          const Eigen::Vector3d eig_vals = eig_solver.eigenvalues().cwiseMax(0.0);
          const double lambda_max = eig_vals.maxCoeff();
          Eigen::Vector3d d_blk;
          for (int k = 0; k < 3; k++)
          {
            const double ratio = (lambda_max > 1e-10) ? eig_vals(k) / lambda_max : 0.0;
            double exponent = -degeneracy_alpha_ * (ratio - degeneracy_tau_);
            exponent = std::max(-40.0, std::min(exponent, 40.0));
            d_blk(k) = 1.0 / (1.0 + std::exp(exponent));
            d_blk(k) = std::max(std::max(0.0, std::min(degeneracy_min_factor_, 1.0)), d_blk(k));
          }
          D_degenerate.block<3, 3>(3 * b, 3 * b) =
              eig_solver.eigenvectors() * d_blk.asDiagonal() * eig_solver.eigenvectors().transpose();
          degeneracy_state_directions_.block<3, 3>(3 * b, 3 * b) = eig_solver.eigenvectors();
          degeneracy_eigenvalues_.segment<3>(3 * b) = eig_vals;
          d_factors.segment<3>(3 * b) = d_blk;
        }
        if (legacy_valid)
        {
          state_projection.block<6, 6>(0, 0) = D_degenerate;
          legacy_projection_active = true;
          degeneracy_factor_ = d_factors.minCoeff();
          degeneracy_mean_factor_ = d_factors.mean();
          degeneracy_direction_factors_ = d_factors;
        }
        else
        {
          degeneracy_numerical_fallback_ = true;
        }
      }
    }

    MD(DIM_STATE, DIM_STATE) posterior_information = prior_precision + H_T_H;
    MD(DIM_STATE, DIM_STATE) posterior_information_regularized, K_1;
    if (!regularizeAndInvertSymmetric<DIM_STATE>(posterior_information, 1e-12,
                                                  posterior_information_regularized, K_1))
    {
      abortStateEstimation("[ LIO ] Invalid posterior information; roll back this ESIKF frame");
      break;
    }
    G.setZero();
    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
    const VD(DIM_STATE) vec = -iteration_delta;
    VD(DIM_STATE)
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
    if (legacy_projection_active)
    {
      solution = vec.block<DIM_STATE, 1>(0, 0) +
                 state_projection * (solution - vec.block<DIM_STATE, 1>(0, 0));
    }
    if (!solution.allFinite())
    {
      abortStateEstimation("[ LIO ] Non-finite ESIKF increment; roll back this ESIKF frame");
      break;
    }
    state_ += solution;
    if (!lioStateMeanFinite(state_))
    {
      abortStateEstimation("[ LIO ] Non-finite state mean after ESIKF injection; roll back this ESIKF frame");
      break;
    }
    auto rot_add = solution.block<3, 1>(0, 0);
    auto t_add = solution.block<3, 1>(3, 0);
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }
    V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0);

    /*** Rematch Judgement ***/

    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2)))) { rematch_num++; }

    /*** Convergence Judgements and Covariance Update ***/
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1)))
    {
      /*** Covariance Update ***/
      MD(DIM_STATE, DIM_STATE) covariance_candidate = K_1;
      if (legacy_projection_active)
      {
        const MD(DIM_STATE, DIM_STATE) projected_gain = state_projection * G;
        const MD(DIM_STATE, DIM_STATE) transition = I_STATE - projected_gain;
        const MD(DIM_STATE, DIM_STATE) gain_noise =
            state_projection * (K_1 * H_T_H * K_1) * state_projection.transpose();
        covariance_candidate = transition * prior_cov * transition.transpose() + gain_noise;
      }

      // StatesGroup uses a right-local rotation error: R_true = R_hat * Exp(error).
      // After injecting rot_add into R_hat, J_r(rot_add) transports the complete
      // posterior covariance, including all rotation cross blocks, into the new
      // tangent coordinates before it is committed to the state.
      const Eigen::Matrix3d rotation_reset_jacobian = RightJacobianSO3(rot_add);
      if (!rotation_reset_jacobian.allFinite())
      {
        abortStateEstimation("[ LIO ] Invalid SO(3) covariance reset Jacobian; roll back this ESIKF frame");
        break;
      }
      MD(DIM_STATE, DIM_STATE) covariance_reset = I_STATE;
      covariance_reset.block<3, 3>(0, 0) = rotation_reset_jacobian;
      covariance_candidate = covariance_reset * covariance_candidate * covariance_reset.transpose();
      if (!covariance_candidate.allFinite())
      {
        abortStateEstimation("[ LIO ] Non-finite covariance after SO(3) tangent reset; roll back this ESIKF frame");
        break;
      }
      covariance_reset_angle_ = rot_add.norm();
      covariance_symmetry_error_ =
          (covariance_candidate - covariance_candidate.transpose()).norm() /
          std::max(covariance_candidate.norm(), 1e-15);
      MD(DIM_STATE, DIM_STATE) covariance_regularized, covariance_inverse_unused;
      if (!regularizeAndInvertSymmetric<DIM_STATE>(covariance_candidate, 1e-12,
                                                    covariance_regularized, covariance_inverse_unused,
                                                    &covariance_min_eigenvalue_))
      {
        abortStateEstimation("[ LIO ] Invalid posterior covariance; roll back this ESIKF frame");
        break;
      }
      state_.cov = covariance_regularized;
      // total_distance += (_state.pos_end - position_last).norm();
      position_last_ = state_.pos_end;
      geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
      EKF_stop_flg = true;
    }
    if (EKF_stop_flg) break;
  }
  ekf_converged_ = state_estimation_valid_ && flg_EKF_converged;

  // double t2 = omp_get_wtime();
  // scan_count++;
  // ekf_time = t2 - t0 - build_residual_time;

  // ave_build_residual_time = ave_build_residual_time * (scan_count - 1) / scan_count + build_residual_time / scan_count;
  // ave_ekf_time = ave_ekf_time * (scan_count - 1) / scan_count + ekf_time / scan_count;

  // cout << "[ Mapping ] ekf_time: " << ekf_time << "s, build_residual_time: " << build_residual_time << "s" << endl;
  // cout << "[ Mapping ] ave_ekf_time: " << ave_ekf_time << "s, ave_build_residual_time: " << ave_build_residual_time << "s" << endl;
}

void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
  trans_cloud->clear();
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void VoxelMapManager::BuildVoxelMap()
{
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;

  std::vector<pointWithVar> input_points;
  const size_t input_size = std::min(feats_down_world_->size(), feats_down_body_->size());
  input_points.reserve(input_size);

  for (size_t i = 0; i < input_size; i++)
  {
    pointWithVar pv;
    pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    if (!pv.point_w.allFinite() || !point_this.allFinite()) continue;
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    point_this = extR_ * point_this + extT_;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    const M3D sensor_rotation = state_.rot_end * extR_;
    const M3D rotation_jacobian = -state_.rot_end * point_crossmat;
    const M3D rot_pos_cov = state_.cov.block<3, 3>(0, 3);
    var = sensor_rotation * var * sensor_rotation.transpose() +
          rotation_jacobian * state_.cov.block<3, 3>(0, 0) * rotation_jacobian.transpose() +
          state_.cov.block<3, 3>(3, 3) + rotation_jacobian * rot_pos_cov +
          rot_pos_cov.transpose() * rotation_jacobian.transpose();
    pv.var = 0.5 * (var + var.transpose());
    input_points.push_back(pv);
  }

  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar p_v = input_points[i];
    int64_t loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = static_cast<int64_t>(std::floor(p_v.point_w[j] / voxel_size));
    }
    VOXEL_LOCATION position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
    }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
    }
  }
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter)
  {
    iter->second->init_octo_tree();
  }
}

V3F VoxelMapManager::RGBFromVoxel(const V3D &input_point)
{
  int64_t loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = floor(input_point[j] / config_setting_.max_voxel_size_);
  }

  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  int64_t ind = loc_xyz[0] + loc_xyz[1] + loc_xyz[2];
  uint k((ind + 100000) % 3);
  V3F RGB((k == 0) * 255.0, (k == 1) * 255.0, (k == 2) * 255.0);
  // cout<<"RGB: "<<RGB.transpose()<<endl;
  return RGB;
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;
  uint plsize = input_points.size();
  current_frame_id_++;
  map_write_accept_num_ = 0;
  map_write_reject_num_ = 0;
  dynamic_reject_num_ = 0;
  map_write_recovery_accept_num_ = 0;
  map_write_candidate_promoted_voxels_ = 0;
  map_write_candidate_expired_voxels_ = 0;
  double map_write_score_sum = 0.0;
  const double self_motion_risk = std::max(0.0, std::min(map_write_motion_risk_, 1.0));
  const double pose_quality_risk = std::max(0.0, std::min(map_write_quality_risk_, 1.0));
  const double map_write_risk = std::max(self_motion_risk, pose_quality_risk);
  // Recovery quarantine exists precisely because LIO pose quality is missing.
  // Its admission still requires low measured motion and temporal geometric
  // consistency; pose-quality risk continues to block every direct map write.
  const double candidate_pose_risk = map_write_external_freeze_ ? self_motion_risk : map_write_risk;
  const int recovery_trigger = std::max(1, map_write_recovery_trigger_frames_);
  map_write_recovery_active_ = map_write_gate_en_ && map_write_recovery_en_ &&
                               map_write_starvation_frames_ >= recovery_trigger;
  map_write_formal_map_frozen_ = map_write_external_freeze_ || map_write_recovery_active_;

  // Candidate voxels are deliberately short lived.  They are not part of the
  // scan-matching map, so expiring them cannot change the current estimator.
  const int candidate_ttl_frames = std::max(30, 3 * recovery_trigger);
  for (auto candidate_it = map_write_candidate_voxels_.begin();
       candidate_it != map_write_candidate_voxels_.end();)
  {
    if (current_frame_id_ - candidate_it->second.last_observed_frame > candidate_ttl_frames)
    {
      candidate_it = map_write_candidate_voxels_.erase(candidate_it);
      map_write_candidate_expired_voxels_++;
    }
    else
    {
      ++candidate_it;
    }
  }

  struct FrameCandidateBucket
  {
    std::vector<pointWithVar> points;
    V3D position_sum = V3D::Zero();
    double score_sum = 0.0;
    double return_sum = 0.0;
    int count = 0;
  };
  std::unordered_map<VOXEL_LOCATION, FrameCandidateBucket> frame_candidates;
  const int candidate_points_per_voxel = std::max(5, std::min(max_points_num, 50));

  auto insert_point = [&](const pointWithVar &point) {
    int64_t insert_loc[3];
    for (int axis = 0; axis < 3; axis++)
    {
      insert_loc[axis] = static_cast<int64_t>(std::floor(point.point_w[axis] / voxel_size));
    }
    VOXEL_LOCATION insert_position(insert_loc[0], insert_loc[1], insert_loc[2]);
    auto insert_iter = voxel_map_.find(insert_position);
    if (insert_iter != voxel_map_.end())
    {
      insert_iter->second->UpdateOctoTree(point);
      return;
    }
    VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
    voxel_map_[insert_position] = octo_tree;
    octo_tree->layer_init_num_ = layer_init_num;
    octo_tree->quater_length_ = voxel_size / 4;
    octo_tree->voxel_center_[0] = (0.5 + insert_position.x) * voxel_size;
    octo_tree->voxel_center_[1] = (0.5 + insert_position.y) * voxel_size;
    octo_tree->voxel_center_[2] = (0.5 + insert_position.z) * voxel_size;
    octo_tree->UpdateOctoTree(point);
  };

  auto stage_candidate = [&](const VOXEL_LOCATION &position, const pointWithVar &point,
                             const double map_score, const double return_score) {
    FrameCandidateBucket &bucket = frame_candidates[position];
    bucket.position_sum += point.point_w;
    bucket.score_sum += map_score;
    bucket.return_sum += return_score;
    bucket.count++;
    if (static_cast<int>(bucket.points.size()) < candidate_points_per_voxel)
    {
      bucket.points.push_back(point);
    }
  };

  for (uint i = 0; i < plsize; i++)
  {
    pointWithVar p_v = input_points[i];
    if (!p_v.point_w.allFinite() || !p_v.var.allFinite())
    {
      map_write_reject_num_++;
      continue;
    }
    int64_t loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = static_cast<int64_t>(std::floor(p_v.point_w[j] / voxel_size));
    }
    VOXEL_LOCATION position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    bool stable_existing_plane = false;
    if (iter != voxel_map_.end())
    {
      VoxelOctoTree *leaf = iter->second->find_correspond(p_v.point_w);
      stable_existing_plane = (leaf != nullptr && leaf->init_octo_ && leaf->plane_ptr_ != nullptr && leaf->plane_ptr_->is_plane_);
    }

    double return_score = computeLidarReturnWeight(p_v.intensity, p_v.range, p_v.map_density_score, nullptr, nullptr, false);
    double dynamic_score = std::max(0.0, std::min(p_v.map_dynamic_score, 1.0));
    if (dynamic_object_filter_en_)
    {
      const double classification_confidence =
          1.0 / (1.0 + std::max(dynamic_motion_gain_, 0.0) * self_motion_risk);
      dynamic_score = std::max(0.0, std::min(dynamic_score * classification_confidence, 1.0));
    }
    double map_score = return_score;
    bool reject_write = false;
    bool dynamic_reject = false;
    // MODIFICATIONS: Innovation 13 - degradation-aware map-write gate.
    // The pose update can downweight bad constraints, but writing those same points
    // into the voxel map would still pollute future frames. Gate map insertion with
    // residual consistency, structural support, return quality and motion risk.
    if (map_write_gate_en_)
    {
      double min_score = std::max(0.0, std::min(map_write_min_score_, 1.0));
      double min_return = std::max(0.0, std::min(map_write_min_return_, 1.0));
      if (p_v.map_match_success)
      {
        double sigma = std::max(map_write_residual_sigma_, 1e-3);
        double residual_ratio = std::max(p_v.map_residual_norm, 0.0) / sigma;
        double residual_score = 1.0 / (1.0 + residual_ratio * residual_ratio);
        double plane_score = std::max(0.0, std::min(p_v.map_plane_quality, 1.0));
        double density_score = std::max(0.0, std::min(p_v.map_density_score, 1.0));
        double structure_score = 0.5 * plane_score + 0.5 * density_score;
        double dynamic_consistency_score = 1.0 - dynamic_score;
        map_score = 0.40 * residual_score + 0.20 * return_score +
                    0.20 * structure_score + 0.20 * dynamic_consistency_score;
        map_score *= 1.0 - 0.25 * pose_quality_risk;
        if (p_v.map_residual_norm > 1.5 * sigma && map_score < std::min(min_score + 0.10, 1.0)) { reject_write = true; }
        if (map_score < min_score) { reject_write = true; }
      }
      else
      {
        double unmatched_risk_thresh = std::max(0.0, std::min(map_write_unmatched_risk_thresh_, 1.0));
        map_score = return_score * (1.0 - 0.50 * map_write_risk) * (1.0 - 0.60 * dynamic_score);
        if (stable_existing_plane)
        {
          double conflict_score_thresh = std::min(min_score + 0.25 + 0.25 * map_write_risk, 1.0);
          if (map_score < conflict_score_thresh || map_write_risk > 0.75) { reject_write = true; }
        }
        else if (map_write_risk > unmatched_risk_thresh && map_score < std::min(min_score + 0.15, 1.0))
        {
          reject_write = true;
        }
      }
      if (return_score < min_return) { reject_write = true; }
      if (dynamic_object_filter_en_ && dynamic_score > std::max(0.0, std::min(dynamic_map_reject_score_, 1.0)))
      {
        reject_write = true;
        dynamic_reject = true;
      }
    }
    p_v.map_return_score = return_score;
    p_v.map_dynamic_score = dynamic_score;
    p_v.map_write_score = map_score;
    map_write_score_sum += map_score;

    const double recovery_min_return = std::max(map_write_min_return_,
        std::max(0.0, std::min(map_write_recovery_min_return_, 1.0)));
    const double recovery_max_dynamic = std::max(0.0, std::min(map_write_recovery_max_dynamic_, 1.0));
    const double recovery_max_risk = std::max(0.0, std::min(map_write_recovery_max_risk_, 1.0));
    const double recovery_score_floor = 0.75 * std::max(0.0, std::min(map_write_min_score_, 1.0));
    const bool candidate_eligible = !stable_existing_plane && !dynamic_reject &&
                                     return_score >= recovery_min_return &&
                                     dynamic_score <= recovery_max_dynamic &&
                                     candidate_pose_risk <= recovery_max_risk &&
                                     map_score >= recovery_score_floor;

    // During recovery the trusted map is immutable.  Potential frontier data
    // goes to the quarantine regardless of whether the old one-frame gate would
    // have accepted it.  Stable existing planes remain usable for registration,
    // but are not mutated until recovery is validated.
    if (map_write_formal_map_frozen_)
    {
      map_write_reject_num_++;
      if (dynamic_reject) dynamic_reject_num_++;
      if (candidate_eligible) stage_candidate(position, p_v, map_score, return_score);
      continue;
    }

    if (reject_write)
    {
      map_write_reject_num_++;
      if (dynamic_reject) { dynamic_reject_num_++; }
      continue;
    }
    map_write_accept_num_++;
    insert_point(p_v);
  }

  // Merge this frame's quarantine observations.  Repeated points from one scan
  // count as one support frame; promotion therefore requires temporal evidence.
  if (map_write_formal_map_frozen_)
  {
    const double consistency_distance = std::max(0.10, 0.75 * static_cast<double>(voxel_size));
    for (auto frame_it = frame_candidates.begin(); frame_it != frame_candidates.end(); ++frame_it)
    {
      FrameCandidateBucket &bucket = frame_it->second;
      if (bucket.count <= 0 || bucket.points.empty()) continue;
      const V3D frame_centroid = bucket.position_sum / static_cast<double>(bucket.count);
      const double frame_score = bucket.score_sum / static_cast<double>(bucket.count);
      const double frame_return = bucket.return_sum / static_cast<double>(bucket.count);
      auto candidate_it = map_write_candidate_voxels_.find(frame_it->first);
      if (candidate_it == map_write_candidate_voxels_.end())
      {
        MapWriteCandidateVoxel candidate;
        candidate.points = bucket.points;
        candidate.centroid = frame_centroid;
        candidate.score_sum = frame_score;
        candidate.return_sum = frame_return;
        candidate.support_frames = 1;
        candidate.first_observed_frame = current_frame_id_;
        candidate.last_observed_frame = current_frame_id_;
        map_write_candidate_voxels_[frame_it->first] = candidate;
        continue;
      }

      MapWriteCandidateVoxel &candidate = candidate_it->second;
      if ((candidate.centroid - frame_centroid).norm() > consistency_distance)
      {
        candidate.points = bucket.points;
        candidate.centroid = frame_centroid;
        candidate.score_sum = frame_score;
        candidate.return_sum = frame_return;
        candidate.support_frames = 1;
        candidate.first_observed_frame = current_frame_id_;
        candidate.last_observed_frame = current_frame_id_;
        map_write_candidate_expired_voxels_++;
        continue;
      }

      if (candidate.last_observed_frame != current_frame_id_)
      {
        const double old_support = static_cast<double>(std::max(candidate.support_frames, 1));
        candidate.centroid = (old_support * candidate.centroid + frame_centroid) / (old_support + 1.0);
        candidate.score_sum += frame_score;
        candidate.return_sum += frame_return;
        candidate.support_frames++;
        candidate.last_observed_frame = current_frame_id_;
      }
      for (size_t point_index = 0;
           point_index < bucket.points.size() &&
           static_cast<int>(candidate.points.size()) < candidate_points_per_voxel;
           ++point_index)
      {
        candidate.points.push_back(bucket.points[point_index]);
      }
    }

    // Bound quarantine memory independently of the formal map.  Oldest
    // candidates are discarded first and can never trigger a map insertion.
    const size_t max_candidate_voxels = 4096;
    if (map_write_candidate_voxels_.size() > max_candidate_voxels)
    {
      std::vector<std::pair<int, VOXEL_LOCATION>> candidate_age_order;
      candidate_age_order.reserve(map_write_candidate_voxels_.size());
      for (auto candidate_it = map_write_candidate_voxels_.begin();
           candidate_it != map_write_candidate_voxels_.end(); ++candidate_it)
      {
        candidate_age_order.push_back(std::make_pair(candidate_it->second.last_observed_frame,
                                                       candidate_it->first));
      }
      std::sort(candidate_age_order.begin(), candidate_age_order.end(),
                [](const std::pair<int, VOXEL_LOCATION> &lhs,
                   const std::pair<int, VOXEL_LOCATION> &rhs) {
                  return lhs.first < rhs.first;
                });
      const size_t remove_count = candidate_age_order.size() - max_candidate_voxels;
      for (size_t remove_index = 0; remove_index < remove_count; ++remove_index)
      {
        map_write_candidate_voxels_.erase(candidate_age_order[remove_index].second);
        map_write_candidate_expired_voxels_++;
      }
    }

    struct PromotionCandidate
    {
      VOXEL_LOCATION location;
      double priority;
    };
    std::vector<PromotionCandidate> promotion_candidates;
    const int min_support_frames = 3;
    // A candidate reaching this point has already passed the recovery-specific
    // risk gate and three-frame temporal validation.  Reapplying the ordinary
    // one-frame unmatched-write threshold here would make validated recovery
    // impossible during exactly the high-motion intervals it is designed for.
    const double commit_risk =
        std::max(0.0, std::min(map_write_recovery_max_risk_, 1.0));
    if (candidate_pose_risk <= commit_risk)
    {
      for (auto candidate_it = map_write_candidate_voxels_.begin();
           candidate_it != map_write_candidate_voxels_.end(); ++candidate_it)
      {
        const MapWriteCandidateVoxel &candidate = candidate_it->second;
        if (candidate.last_observed_frame != current_frame_id_ ||
            candidate.support_frames < min_support_frames || candidate.points.empty())
        {
          continue;
        }
        const double mean_score = candidate.score_sum / static_cast<double>(candidate.support_frames);
        const double mean_return = candidate.return_sum / static_cast<double>(candidate.support_frames);
        const double score_floor = std::max(0.0, std::min(map_write_min_score_, 1.0));
        const double return_floor = std::max(map_write_min_return_, map_write_recovery_min_return_);
        if (mean_score < score_floor || mean_return < return_floor) continue;
        promotion_candidates.push_back({candidate_it->first, 0.60 * mean_return + 0.40 * mean_score});
      }
    }

    std::sort(promotion_candidates.begin(), promotion_candidates.end(),
              [](const PromotionCandidate &lhs, const PromotionCandidate &rhs) {
                return lhs.priority > rhs.priority;
              });
    const double budget_ratio = std::max(0.001, std::min(map_write_recovery_budget_ratio_, 0.25));
    const int minimum_plane_points = layer_init_num.empty() ? 5 : std::max(1, layer_init_num[0]);
    const int recovery_budget = std::max(minimum_plane_points,
        static_cast<int>(std::ceil(budget_ratio * static_cast<double>(plsize))));
    int remaining_budget = recovery_budget;
    for (size_t promotion_index = 0;
         promotion_index < promotion_candidates.size() && remaining_budget > 0;
         ++promotion_index)
    {
      auto candidate_it = map_write_candidate_voxels_.find(promotion_candidates[promotion_index].location);
      if (candidate_it == map_write_candidate_voxels_.end()) continue;
      const int promotion_point_count = std::min(
          remaining_budget, static_cast<int>(candidate_it->second.points.size()));
      for (int point_index = 0; point_index < promotion_point_count; ++point_index)
      {
        insert_point(candidate_it->second.points[point_index]);
      }
      if (promotion_point_count > 0)
      {
        map_write_recovery_accept_num_ += promotion_point_count;
        map_write_candidate_promoted_voxels_++;
        remaining_budget -= promotion_point_count;
      }
      map_write_candidate_voxels_.erase(candidate_it);
    }

    if (map_write_recovery_accept_num_ > 0)
    {
      map_write_accept_num_ = map_write_recovery_accept_num_;
      map_write_reject_num_ = std::max(0, map_write_reject_num_ - map_write_recovery_accept_num_);
      map_write_starvation_frames_ = 0;
      map_write_recovery_active_ = false;
      map_write_formal_map_frozen_ = map_write_external_freeze_;
      // A successful promotion establishes a new trusted frontier.  Do not let
      // unrelated stale candidates leak into the next healthy interval.
      map_write_candidate_voxels_.clear();
    }
  }

  map_write_candidate_point_num_ = 0;
  for (auto candidate_it = map_write_candidate_voxels_.begin();
       candidate_it != map_write_candidate_voxels_.end(); ++candidate_it)
  {
    map_write_candidate_point_num_ += static_cast<int>(candidate_it->second.points.size());
  }
  map_write_accept_ratio_ = (plsize > 0) ? (double)map_write_accept_num_ / (double)plsize : 1.0;
  map_write_mean_score_ = (plsize > 0) ? map_write_score_sum / (double)plsize : 1.0;
  if (!map_write_gate_en_ || !map_write_recovery_en_ || plsize < 10)
  {
    map_write_starvation_frames_ = 0;
    map_write_recovery_active_ = false;
    map_write_formal_map_frozen_ = map_write_external_freeze_;
    if (!map_write_external_freeze_)
    {
      map_write_candidate_voxels_.clear();
      map_write_candidate_point_num_ = 0;
    }
  }
  else if (map_write_recovery_accept_num_ > 0)
  {
    map_write_starvation_frames_ = 0;
  }
  else if (map_write_formal_map_frozen_ || map_write_accept_ratio_ < 0.02)
  {
    map_write_starvation_frames_ = std::min(map_write_starvation_frames_ + 1, 1000000);
  }
  else if (map_write_accept_ratio_ > 0.10)
  {
    map_write_starvation_frames_ = 0;
  }
}

void VoxelMapManager::RefreshMapWriteMetadata(std::vector<pointWithVar> &input_points)
{
  for (pointWithVar &point : input_points)
  {
    point.normal.setZero();
    point.map_match_success = false;
    point.map_residual_norm = 0.0;
    point.map_plane_quality = 1.0;
    point.map_density_score = 1.0;
    point.map_return_score = computeLidarReturnWeight(point.intensity, point.range,
                                                       point.map_density_score, nullptr, nullptr, false);
    point.map_dynamic_score = 0.0;
    point.map_write_score = 1.0;
  }

  // This second correspondence query belongs only to map-write/VIO metadata.
  // Keep the accepted ESIKF mean, covariance, information, and diagnostics intact.
  std::vector<PointToPlane> final_pose_matches;
  BuildResidualListOMP(input_points, final_pose_matches);
}

void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
  double voxel_size = config_setting_.max_voxel_size_;
  ptpl_list.clear();
  ptpl_list.reserve(pv_list.size());
  residual_ptpl_buffer_.resize(pv_list.size());
  residual_useful_buffer_.assign(pv_list.size(), 0U);
  #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
  #endif
  for (int i = 0; i < static_cast<int>(pv_list.size()); i++)
  {
    pointWithVar &pv = pv_list[i];
    if (!pv.point_w.allFinite()) continue;
    int64_t loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = static_cast<int64_t>(std::floor(pv.point_w[j] / voxel_size));
    }
    VOXEL_LOCATION position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      VoxelOctoTree *current_octo = iter->second;
      PointToPlane single_ptpl;
      bool is_sucess = false;
      double prob = 0;
      build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl);
      if (!is_sucess)
      {
        VOXEL_LOCATION near_position = position;
        // voxel_center_ and quater_length_ are in world metres; loc_xyz is a
        // dimensionless voxel index and cannot be compared against them.
        if (pv.point_w[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) { near_position.x = near_position.x + 1; }
        else if (pv.point_w[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) { near_position.x = near_position.x - 1; }
        if (pv.point_w[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) { near_position.y = near_position.y + 1; }
        else if (pv.point_w[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) { near_position.y = near_position.y - 1; }
        if (pv.point_w[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) { near_position.z = near_position.z + 1; }
        else if (pv.point_w[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) { near_position.z = near_position.z - 1; }
        auto iter_near = voxel_map_.find(near_position);
        if (iter_near != voxel_map_.end()) { build_single_residual(pv, iter_near->second, 0, is_sucess, prob, single_ptpl); }
      }
      if (is_sucess)
      {
        residual_useful_buffer_[i] = 1U;
        residual_ptpl_buffer_[i] = single_ptpl;
      }
    }
  }
  for (size_t i = 0; i < residual_useful_buffer_.size(); i++)
  {
    if (residual_useful_buffer_[i] != 0U) { ptpl_list.push_back(residual_ptpl_buffer_[i]); }
  }
}

void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess,
                                            double &prob, PointToPlane &single_ptpl)
{
  if (current_octo == nullptr || current_octo->plane_ptr_ == nullptr) { return; }
  int max_layer = config_setting_.max_layer_;
  double sigma_num = config_setting_.sigma_num_;

  double radius_k = 3;
  Eigen::Vector3d p_w = pv.point_w;
  if (current_octo->plane_ptr_->is_plane_)
  {
    VoxelPlane &plane = *current_octo->plane_ptr_;
    Eigen::Vector3d p_world_to_center = p_w - plane.center_;
    float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);
    float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                          (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
    float range_dis = sqrt(std::max(dis_to_center - dis_to_plane * dis_to_plane, 0.0f));

    if (range_dis <= radius_k * plane.radius_)
    {
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
      J_nq.block<1, 3>(0, 3) = -plane.normal_;
      double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
      sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
      if (!std::isfinite(sigma_l) || sigma_l <= 0.0) { return; }
      sigma_l = std::max(sigma_l, 1e-12);
      double sigma_sqrt = sqrt(sigma_l);
      double residual_norm = dis_to_plane / std::max(sigma_sqrt, 1e-6);
      double density_ref = std::max(lidar_density_ref_, 1.0);
      double density_score = std::max(0.0, std::min((double)plane.points_size_ / density_ref, 1.0));
      double dynamic_score = computeDynamicConflictScore(residual_norm, plane.quality_score_, density_score);
      if (dis_to_plane < sigma_num * sigma_sqrt)
      {
        is_sucess = true;
        double this_prob = 1.0 / sigma_sqrt * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
        if (this_prob > prob)
        {
          prob = this_prob;
          pv.normal = plane.normal_;
          pv.map_match_success = true;
          pv.map_residual_norm = residual_norm;
          pv.map_plane_quality = plane.quality_score_;
          pv.map_density_score = density_score;
          pv.map_dynamic_score = dynamic_score;
          single_ptpl.body_cov_ = pv.body_var;
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          single_ptpl.point_offset_time_ = static_cast<float>(pv.point_offset_time);
          single_ptpl.intensity_ = static_cast<float>(pv.intensity);
          single_ptpl.range_ = static_cast<float>(pv.range);
          pv.map_return_score = computeLidarReturnWeight(pv.intensity, pv.range, density_score, nullptr, nullptr, false);
          single_ptpl.density_score_ = static_cast<float>(density_score);
          single_ptpl.dynamic_score_ = static_cast<float>(dynamic_score);
          single_ptpl.plane_var_ = plane.plane_var_;
          single_ptpl.normal_ = plane.normal_;
          single_ptpl.center_ = plane.center_;
          single_ptpl.d_ = plane.d_;
          single_ptpl.layer_ = current_layer;
          single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
          single_ptpl.plane_quality_ = plane.quality_score_;
        }
        return;
      }
      else
      {
        // is_sucess = false;
        if (!pv.map_match_success)
        {
          pv.map_plane_quality = plane.quality_score_;
          pv.map_density_score = density_score;
          pv.map_dynamic_score = std::max(pv.map_dynamic_score, dynamic_score);
          pv.map_residual_norm = std::max(pv.map_residual_norm, residual_norm);
          pv.map_return_score = computeLidarReturnWeight(pv.intensity, pv.range, density_score, nullptr, nullptr, false);
        }
        return;
      }
    }
    else
    {
      // is_sucess = false;
      return;
    }
  }
  else
  {
    if (current_layer < max_layer)
    {
      for (size_t leafnum = 0; leafnum < 8; leafnum++)
      {
        if (current_octo->leaves_[leafnum] != nullptr)
        {

          VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
          build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
        }
      }
      return;
    }
    else { return; }
  }
}

void VoxelMapManager::pubVoxelMap()
{
  double max_trace = 0.25;
  double pow_num = 0.2;
  ros::Rate loop(500);
  float use_alpha = 0.8;
  visualization_msgs::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);
  std::vector<VoxelPlane> pub_plane_list;
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
  {
    GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
  }
  for (size_t i = 0; i < pub_plane_list.size(); i++)
  {
    V3D plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) { trace = max_trace; }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    double alpha;
    if (pub_plane_list[i].is_plane_) { alpha = use_alpha; }
    else { alpha = 0; }
    pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
  }
  voxel_map_pub_.publish(voxel_plane);
  loop.sleep();
}

void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list)
{
  if (current_octo->layer_ > pub_max_voxel_layer) { return; }
  if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
  if (current_octo->layer_ < current_octo->max_layer_)
  {
    if (!current_octo->plane_ptr_->is_plane_)
    {
      for (size_t i = 0; i < 8; i++)
      {
        if (current_octo->leaves_[i] != nullptr) { GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list); }
      }
    }
  }
  return;
}

void VoxelMapManager::pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string &plane_ns, const VoxelPlane &single_plane,
                                    const float alpha, const Eigen::Vector3d rgb)
{
  visualization_msgs::Marker plane;
  plane.header.frame_id = "camera_init";
  plane.header.stamp = ros::Time();
  plane.ns = plane_ns;
  plane.id = single_plane.id_;
  plane.type = visualization_msgs::Marker::CYLINDER;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = single_plane.center_[0];
  plane.pose.position.y = single_plane.center_[1];
  plane.pose.position.z = single_plane.center_[2];
  geometry_msgs::Quaternion q;
  CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
  plane.pose.orientation = q;
  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_);
  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = ros::Duration();
  plane_pub.markers.push_back(plane);
}

void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                                      geometry_msgs::Quaternion &q)
{
  Eigen::Matrix3d rot;
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
  Eigen::Matrix3d rotation = rot.transpose();
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) { v = vmin; }

  if (v > vmax) { v = vmax; }

  double dr, dg, db;

  if (v < 0.1242)
  {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  }
  else if (v < 0.3747)
  {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  }
  else if (v < 0.6253)
  {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  }
  else if (v < 0.8758)
  {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  }
  else
  {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void VoxelMapManager::mapSliding()
{
  if((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh)
  {
    if (verbose_runtime_log_)
      std::cout<<RED<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
    return;
  }

  //get global id now
  last_slide_position = position_last_;
  double t_sliding_start = omp_get_wtime();
  int64_t loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = static_cast<int64_t>(std::floor(position_last_[j] / config_setting_.max_voxel_size_));
  }
  clearMemOutOfMap(loc_xyz[0] + config_setting_.half_map_size, loc_xyz[0] - config_setting_.half_map_size,
                    loc_xyz[1] + config_setting_.half_map_size, loc_xyz[1] - config_setting_.half_map_size,
                    loc_xyz[2] + config_setting_.half_map_size, loc_xyz[2] - config_setting_.half_map_size);
  double t_sliding_end = omp_get_wtime();
  if (verbose_runtime_log_)
    std::cout<<RED<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
  return;
}

void VoxelMapManager::clearMemOutOfMap(int64_t x_max, int64_t x_min, int64_t y_max, int64_t y_min, int64_t z_max, int64_t z_min)
{
  int delete_voxel_cout = 0;
  // double delete_time = 0;
  // double last_delete_time = 0;
  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); )
  {
    const VOXEL_LOCATION& loc = it->first;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      // last_delete_time = omp_get_wtime();
      delete it->second;
      it = voxel_map_.erase(it);
      // delete_time += omp_get_wtime() - last_delete_time;
      delete_voxel_cout++;
    } else {
      ++it;
    }
  }
  if (verbose_runtime_log_)
    std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<RESET<<"\n";
  // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}
