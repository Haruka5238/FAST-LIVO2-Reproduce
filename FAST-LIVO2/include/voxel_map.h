/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;

typedef struct VoxelMapConfig
{
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  std::vector<int> layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double sigma_num_;
  bool is_pub_plane_map_;

  // config of local map sliding
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;
} VoxelMapConfig;

typedef struct PointToPlane
{
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
  float plane_quality_ = 1.0; // 0=poor, 1=reliable (from voxel eigenvalue ratios)
  float point_offset_time_ = 0.0f; // seconds from scan start, used for scan-distortion weighting
  float intensity_ = 0.0f; // LiDAR return intensity/reflectivity
  float range_ = 0.0f; // LiDAR body-frame range
  float density_score_ = 1.0f; // local plane support density score, 0=weak support, 1=healthy
  float dynamic_score_ = 0.0f; // transient/dynamic conflict likelihood against stable map planes
} PointToPlane;

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  float quality_score_ = 1.0; // planar quality: lambda_min / lambda_mid ratio
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

// Points rejected from the formal map during a degraded interval are kept in a
// bounded, frame-confirmed quarantine.  A candidate voxel is never queried by
// scan matching; it can only be promoted after consistent support from several
// different frames.  This separates recoverable frontier observations from the
// estimator's trusted map.
struct MapWriteCandidateVoxel
{
  std::vector<pointWithVar> points;
  V3D centroid = V3D::Zero();
  double score_sum = 0.0;
  double return_sum = 0.0;
  int support_frames = 0;
  int first_observed_frame = -1;
  int last_observed_frame = -1;
};

struct DS_POINT
{
  float xyz[3];
  float intensity;
  int count = 0;
};

void calcBodyCov(const Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{

public:
  VoxelOctoTree() = default;
  std::vector<pointWithVar> temp_points_;
  VoxelPlane *plane_ptr_ = nullptr;
  int layer_ = 0;
  int octo_state_ = 0; // 0 is end of tree, 1 is not
  VoxelOctoTree *leaves_[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
  double voxel_center_[3] = {0.0, 0.0, 0.0}; // x, y, z
  std::vector<int> layer_init_num_;
  float quater_length_ = 0.0f;
  float planer_threshold_ = 0.0f;
  int points_size_threshold_ = 0;
  int update_size_threshold_ = 5;
  int max_points_num_ = 0;
  int max_layer_ = 0;
  int new_points_ = 0;
  bool init_octo_ = false;
  bool update_enable_ = true;

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  VoxelOctoTree(const VoxelOctoTree &) = delete;
  VoxelOctoTree &operator=(const VoxelOctoTree &) = delete;

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
  void init_octo_tree();
  void cut_octo_tree();
  void UpdateOctoTree(const pointWithVar &pv);

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
  VoxelOctoTree *Insert(const pointWithVar &pv);
};

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

class VoxelMapManager
{
public:
  VoxelMapManager() = delete;
  VoxelMapConfig config_setting_;
  int current_frame_id_ = 0;
  bool verbose_runtime_log_ = false;
  ros::Publisher voxel_map_pub_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;

  M3D extR_;
  V3D extT_;
  float build_residual_time = 0.0f, ekf_time = 0.0f;
  float ave_build_residual_time = 0.0;
  float ave_ekf_time = 0.0;
  int scan_count = 0;
  StatesGroup state_;
  V3D position_last_;

  V3D last_slide_position = {0,0,0};

  geometry_msgs::Quaternion geoQuat_;

  int feats_down_size_ = 0;
  int effct_feat_num_ = 0;
  double avg_residual_ = 0.0; // diagnostics: mean point-to-plane residual of the last ESIKF iteration
  bool ekf_converged_ = false; // diagnostics: convergence flag of the last scan's ESIKF update
  bool state_estimation_valid_ = true; // false when a fatal numerical guard rolls the frame back to its propagated prior
  bool state_prior_invalid_ = false; // invalid incoming mean/covariance requires coordinated estimator reset

  // MODIFICATIONS: degeneracy-aware ESIKF (Innovations 1, 4 & 9)
  bool degeneracy_aware_en_ = true;   // enable directional degeneracy suppression in the Kalman update
  bool coupled_degeneracy_en_ = true; // use prior-whitened full 6DoF information instead of legacy 3x3 blocks
  double degeneracy_alpha_ = 10.0;    // sigmoid steepness for eigenvalue-ratio -> scaling factor
  double degeneracy_tau_ = 0.05;      // eigenvalue ratio below which a direction is considered degenerate
  double degeneracy_min_factor_ = 0.08; // floor for directional scaling to avoid fully starving a pose direction
  double degeneracy_prior_eigen_ratio_floor_ = 1e-9; // relative SPD floor used only for prior whitening
  Eigen::Matrix<double, 6, 1> degeneracy_eigenvalues_ = Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> degeneracy_direction_factors_ = Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 6> degeneracy_state_directions_ = Eigen::Matrix<double, 6, 6>::Identity();
  double degeneracy_factor_ = 1.0;    // min directional scaling of the last update, 0 = fully degenerate, 1 = healthy
  double degeneracy_mean_factor_ = 1.0; // mean full-6DoF direction reliability
  double degeneracy_whitened_min_eigenvalue_ = 0.0; // dimensionless prior-whitened information spectrum
  double degeneracy_whitened_max_eigenvalue_ = 0.0;
  double degeneracy_whitened_condition_ratio_ = 1.0;
  double degeneracy_rot_trans_coupling_ = 0.0; // normalized Frobenius energy of the off-diagonal information block
  bool degeneracy_coupled_active_ = false;
  bool degeneracy_numerical_fallback_ = false;
  double covariance_relinearization_angle_ = 0.0; // current-vs-propagated rotation used to transport the frozen prior (rad)
  double covariance_reset_angle_ = 0.0; // final SO(3) injection magnitude used by the covariance reset (rad)
  double covariance_min_eigenvalue_ = -1.0; // minimum eigenvalue after tangent reset and SPD regularization
  double covariance_symmetry_error_ = 0.0; // relative asymmetry before final covariance regularization
  double normal_anisotropy_ = 1.0;    // lambda_min/lambda_max of the constraint-normal scatter, 0 = single-direction constraints
  bool normal_anisotropy_weight_en_ = true; // apply anisotropy-derived measurement-noise inflation
  bool normal_balance_en_ = false;    // optional ablation; coupled directional information is the default
  double normal_balance_target_fraction_ = 0.35; // soft cap per normal-direction bin
  double normal_balance_min_weight_ = 0.20; // lower bound so dominant planes still contribute
  double normal_balance_mean_weight_ = 1.0; // diagnostics: mean per-constraint normal-balance weight
  double dominant_normal_ratio_ = 0.0; // diagnostics: largest normal-bin count / effective constraints
  bool incidence_weight_en_ = false;  // optional ablation; disabled in the simplified default model
  double incidence_min_weight_ = 0.10; // lower bound for incidence weighting to avoid starving the update
  double incidence_weight_power_ = 1.0; // >1 penalizes grazing angle more aggressively
  double incidence_weight_mean_ = 1.0; // diagnostics: mean incidence weight of the last ESIKF iteration
  bool robust_kernel_en_ = true;      // continuous robust weighting for large point-to-plane residuals
  std::string robust_kernel_type_ = "cauchy"; // cauchy or huber, applied on normalized residual
  double robust_kernel_delta_ = 3.0;  // normalized residual scale where robust downweighting starts
  double robust_kernel_min_weight_ = 0.05; // lower bound to keep the linear system numerically populated
  double robust_kernel_mean_weight_ = 1.0; // diagnostics: mean residual robust-kernel weight
  double residual_consistency_mean_weight_ = 1.0; // diagnostics: single robust residual precision factor
  bool lio_hard_outlier_reject_en_ = true; // reject only high-confidence residual conflicts before applying the total floor
  double lio_hard_outlier_residual_sigma_ = 2.75;
  double lio_hard_outlier_dynamic_score_ = 0.40;
  int lio_hard_outlier_reject_num_ = 0;
  bool scan_distortion_weight_en_ = false; // optional ablation; process noise already models motion risk
  double scan_distortion_risk_ = 0.0; // latest IMU-derived scan distortion risk from ImuProcess
  double scan_distortion_time_span_ = 0.0; // scan duration in seconds, for point-time weighting
  double scan_distortion_noise_gain_ = 2.0; // precision shrink: w = 1 / (1 + gain * risk^2)
  double scan_distortion_min_weight_ = 0.30; // lower bound so high-motion frames still contribute
  double scan_distortion_mean_weight_ = 1.0; // diagnostics: mean scan-distortion weight
  bool lidar_return_quality_en_ = true; // downweight weak/saturated/far/sparse LiDAR constraints
  bool lidar_intensity_available_ = true; // false when the selected sensor has no return-intensity channel
  double lidar_intensity_ref_ = 80.0; // intensity that reaches full low-return score
  double lidar_saturation_ref_ = 240.0; // above this, specular/saturation penalty starts
  double lidar_range_ref_ = 35.0; // range with full score; farther points decay smoothly
  bool lidar_range_adaptive_en_ = true; // raise the range reference for high-altitude/far-view scans
  bool lidar_range_candidate_source_en_ = true; // use all finite downsampled candidates instead of matched-only ranges
  double lidar_range_ref_max_ = 80.0; // hard upper bound for adaptive range reference
  double lidar_range_quantile_ = 0.70; // robust active-source range quantile
  double lidar_range_ref_ema_alpha_ = 0.15; // per-frame adaptation rate
  double lidar_range_ref_effective_ = 35.0; // diagnostics/current range reference used by weighting
  double lidar_range_quantile_value_ = 35.0; // last active-source quantile accepted by the EMA update
  double lidar_range_candidate_quantile_ = -1.0; // current all-candidate range quantile (m)
  double lidar_range_matched_quantile_ = -1.0; // current successful-match range quantile (m)
  double lidar_range_selection_gap_ = -1.0; // candidate minus matched quantile (m)
  int lidar_range_candidate_count_ = 0; // finite positive candidate ranges in iteration zero
  double lidar_range_matched_coverage_ = 0.0; // matched range count / candidate range count
  bool lidar_range_ref_initialized_ = false;
  double lidar_density_ref_ = 20.0; // plane support points with full density score
  double lidar_return_min_weight_ = 0.25; // lower bound for return-quality weighting
  double lidar_return_quality_mean_weight_ = 1.0; // diagnostics: mean return-quality weight
  double lidar_return_intensity_score_ = 1.0; // diagnostics: mean intensity sub-score
  double lidar_return_density_score_ = 1.0; // diagnostics: mean local support sub-score
  bool map_write_gate_en_ = true; // gate unstable points before they pollute the voxel map
  double map_write_residual_sigma_ = 2.5; // normalized residual scale for matched-point write confidence
  double map_write_min_score_ = 0.35; // minimum map-write confidence
  double map_write_unmatched_risk_thresh_ = 0.45; // high-motion threshold for unmatched new-area writes
  double map_write_min_return_ = 0.20; // minimum return-quality score for any map write
  double map_write_motion_risk_ = 0.0; // self-motion/scan/vibration risk; also attenuates dynamic classification confidence
  double map_write_quality_risk_ = 0.0; // current-frame LIO pose-quality risk used only by map-write gating
  bool map_write_recovery_en_ = true; // bounded high-confidence frontier recovery after sustained write starvation
  int map_write_recovery_trigger_frames_ = 8;
  double map_write_recovery_budget_ratio_ = 0.05;
  double map_write_recovery_min_return_ = 0.55;
  double map_write_recovery_max_dynamic_ = 0.25;
  double map_write_recovery_max_risk_ = 0.85;
  int map_write_starvation_frames_ = 0;
  bool map_write_recovery_active_ = false;
  int map_write_recovery_accept_num_ = 0;
  std::unordered_map<VOXEL_LOCATION, MapWriteCandidateVoxel> map_write_candidate_voxels_;
  int map_write_candidate_point_num_ = 0;
  int map_write_candidate_promoted_voxels_ = 0;
  int map_write_candidate_expired_voxels_ = 0;
  bool map_write_external_freeze_ = false; // estimator-level recovery freezes the trusted map independently
  bool map_write_formal_map_frozen_ = false;
  int map_write_accept_num_ = 0; // diagnostics: accepted points in last map update
  int map_write_reject_num_ = 0; // diagnostics: rejected points in last map update
  double map_write_accept_ratio_ = 1.0; // diagnostics: accepted / input points
  double map_write_mean_score_ = 1.0; // diagnostics: mean write confidence over input points
  bool dynamic_object_filter_en_ = true; // suppress transient objects conflicting with stable voxel planes
  double dynamic_residual_sigma_ = 3.0; // normalized residual where dynamic conflict begins
  double dynamic_map_reject_score_ = 0.65; // reject map writes above this dynamic score
  double dynamic_motion_gain_ = 0.50; // high-motion attenuation of single-frame dynamic classification confidence
  double dynamic_mean_score_ = 0.0; // diagnostics: mean dynamic score among LIO constraints
  double dynamic_mean_weight_ = 1.0; // diagnostics: mean dynamic precision weight
  int dynamic_reject_num_ = 0; // diagnostics: map writes rejected by dynamic conflict gate
  double total_lio_min_weight_ = 0.03; // floor for all multiplicative LIO precision weights combined
  bool plane_quality_weight_en_ = false; // optional ablation; avoid a second structural precision penalty
  double total_lio_effective_weight_mean_ = 1.0; // diagnostics: final combined weight after all factors
  double total_lio_low_weight_ratio_ = 0.0; // diagnostics: fraction of constraints near the total floor
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<int> normal_bin_ids_buffer_;
  std::vector<double> candidate_range_buffer_;
  std::vector<double> matched_range_buffer_;
  std::vector<PointToPlane> residual_ptpl_buffer_;
  std::vector<unsigned char> residual_useful_buffer_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr state_estimation_world_lidar_scratch_;

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    state_estimation_world_lidar_scratch_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  };
  ~VoxelMapManager();

  void resetRuntimeState();

  void StateEstimation(StatesGroup &state_propagat);
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  void BuildVoxelMap();
  V3F RGBFromVoxel(const V3D &input_point);

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);
  void RefreshMapWriteMetadata(std::vector<pointWithVar> &input_points);
  double computeLidarReturnWeight(const double intensity, const double range, const double density_score,
                                  double *intensity_score_out = nullptr, double *density_score_out = nullptr,
                                  const bool apply_min_weight = true) const;
  void updateAdaptiveLidarRangeReference();
  double computeDynamicConflictScore(const double residual_norm, const double plane_quality,
                                     const double density_score) const;

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  void pubVoxelMap();

  void mapSliding();
  void clearMemOutOfMap(int64_t x_max, int64_t x_min, int64_t y_max, int64_t y_min, int64_t z_max, int64_t z_min);

private:
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string &plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_
