/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VIO_H_
#define VIO_H_

#include "voxel_map.h"
#include "feature.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <unordered_set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

struct SubSparseMap
{
  vector<float> propa_errors;
  vector<float> errors;
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<VisualPoint *> voxel_points;
  vector<double> inv_expo_list;
  vector<double> point_weights;
  vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    point_weights.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    point_weights.clear();
    add_from_voxel_map.clear();
  }
};

class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};

class VIOManager
{
public:
  int grid_size = 0;
  vk::AbstractCamera *cam = nullptr;
  vk::PinholeCamera *pinhole_cam = nullptr;
  StatesGroup *state = nullptr;
  StatesGroup *state_propagat = nullptr;
  M3D Rli = M3D::Identity(), Rci = M3D::Identity(), Rcl = M3D::Identity(), Rcw = M3D::Identity();
  M3D Jdphi_dR = M3D::Zero(), Jdp_dt = M3D::Zero(), Jdp_dR = M3D::Zero();
  V3D Pli = V3D::Zero(), Pci = V3D::Zero(), Pcl = V3D::Zero(), Pcw = V3D::Zero();
  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;
  bool normal_en = false, inverse_composition_en = false, exposure_estimate_en = false;
  bool raycast_en = false, has_ref_patch_cache = false;
  bool ncc_en = false, colmap_output_en = false;
  bool verbose_runtime_log = false;

  int width = 0, height = 0, grid_n_width = 0, grid_n_height = 0, length = 0;
  double image_resize_factor = 1.0;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  int patch_pyrimid_level = 0, patch_size = 0, patch_size_total = 0;
  int patch_size_half = 0, border = 0, warp_len = 0;
  int max_iterations = 1, total_points = 0;

  double img_point_cov = 100.0, outlier_threshold = 1000.0, ncc_thre = 0.8;
  
  SubSparseMap *visual_submap = nullptr;
  std::vector<std::vector<V3D>> rays_with_sample_points;

  double compute_jacobian_time = 0.0, update_ekf_time = 0.0;
  double ave_total = 0;
  double last_frame_time = 0.0; // diagnostics: total processing time of the last VIO frame

  // MODIFICATIONS: degradation-aware adaptive VIO (Innovations 2, 3 & 5)
  bool adaptive_vio_cov_en = true;      // enable per-frame adaptive visual measurement covariance
  double current_frame_img_cov = 100.0; // effective img_point_cov actually used this frame
  bool vio_cov_fusion_en = true;        // fuse correlated temporal/cross-modal covariance risks without multiplying both
  double vio_cov_scale_max = 100.0;     // final covariance upper bound relative to img_point_cov
  double base_frame_img_cov = 100.0;    // diagnostics: image/motion/LIO covariance before auxiliary risks
  double fused_cov_scale = 1.0;         // diagnostics: final auxiliary temporal/cross-modal scale
  bool vio_cov_clamped = false;         // diagnostics: final covariance hit configured upper bound
  double vio_covariance_reset_angle = 0.0; // final SO(3) covariance reset angle (rad)
  double vio_covariance_min_eigenvalue = 0.0; // committed 19x19 covariance minimum eigenvalue
  double vio_covariance_symmetry_error = 0.0; // relative asymmetry before final regularization
  bool vio_covariance_numerical_failure = false; // current VIO update was atomically rolled back
  bool vio_update_valid = false;       // at least one fixed-support visual linearization was accepted and committed
  double illum_quality = 1.0;           // 0~1, composite image quality
  double exposure_quality = 1.0;        // 0~1, exposure sub-score (over/under-exposure)
  double texture_quality  = 1.0;        // 0~1, texture richness sub-score
  double blur_quality     = 1.0;        // 0~1, sharpness / motion-blur sub-score
  double visual_quality = 1.0;          // illum_quality combined with motion blur risk
  double last_inv_expo = 1.0;           // previous frame's estimated inverse exposure time
  double motion_degrade = 0.0;          // 0~1, motion excitation risk fed from ImuProcess
  double lio_info_ratio = 1.0;          // legacy scalar diagnostic; no longer scales the whole VIO modality
  bool directional_lio_vio_fusion_en = true;
  Eigen::Matrix<double, 6, 6> lio_state_directions = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 1> lio_direction_factors = Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> visual_direction_factors = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> fused_direction_factors = Eigen::Matrix<double, 6, 1>::Ones();
  bool directional_fusion_active = false;
  double fused_direction_min = 1.0;
  double min_gen_quality = 0.15;        // below this visual quality, skip generating new visual map points
  double min_update_quality = 0.18;     // below this, skip updateVisualMapPoints (new observation writes)
  double min_ref_update_quality = 0.20; // below this visual quality, skip reference-patch updates
  double map_starving_min_quality = 0.08; // absolute quality floor before starvation can write visual map
  bool visual_lifecycle_en = true;      // point-level map-pollution suppression
  int visual_lifecycle_max_fail = 6;    // consecutive failures before a point is disabled
  double visual_lifecycle_min_quality = 0.18; // low score plus failures disables a point
  int lifecycle_good_points = 0;        // diagnostics: good tracked points this frame
  int lifecycle_failed_points = 0;      // diagnostics: failed visual point updates this frame
  int lifecycle_new_bad_points = 0;     // diagnostics: points newly marked bad this frame
  int lifecycle_active_bad_points = 0;  // diagnostics: bad points seen in current visual submap
  enum VisualRecoveryState
  {
    VISUAL_RECOVERY_NORMAL = 0,
    VISUAL_RECOVERY_SEEDING = 1,
    VISUAL_RECOVERY_VALIDATING = 2,
    VISUAL_RECOVERY_COOLDOWN = 3
  };
  VisualRecoveryState visual_recovery_state = VISUAL_RECOVERY_NORMAL;
  int visual_tracking_starvation_frames = 0;
  int visual_recovery_attempts = 0;
  int visual_recovery_validation_streak = 0;
  int visual_recovery_cooldown_frames = 0;
  int visual_recovery_success_count = 0;
  int visual_recovery_failure_count = 0;
  int visual_recovery_seeded_points = 0;
  bool visual_recovery_ever_valid = false;
  // Camera-visible voxels admitted by a validated recovery remain eligible for
  // projection even when the reduced current-scan LiDAR list misses their key.
  std::unordered_set<VOXEL_LOCATION> recovered_visual_voxels;
  bool visual_view_weight_en = true;    // point-level view/warp quality weighting in VIO EKF
  double visual_view_min_weight = 0.20; // lower bound for poor view-angle observations
  double visual_view_min_cos = 0.17;    // grazing incidence below this is considered unreliable
  double visual_affine_det_ref = 3.0;   // determinant deformation where affine score is about half
  double visual_view_weight_mean = 1.0; // diagnostics: mean point-level VIO weight
  double visual_view_incidence_mean = 1.0; // diagnostics: mean current/ref incidence score
  double visual_affine_score_mean = 1.0; // diagnostics: mean affine warp deformation score
  int visual_low_view_points = 0;       // diagnostics: low view/warp weighted points
  bool cross_modal_gate_en = true;      // fuse latest LIO health with visual health before trusting VIO/map writes
  double lio_health_score = 1.0;        // latest 0~1 LIO health score fed from LIVMapper
  double cross_modal_cov_gain = 3.0;    // extra visual covariance inflation when LIO and VIO both degrade
  double cross_modal_map_risk_thresh = 0.60; // block visual map writes above this double-degradation risk
  int cross_modal_min_track_points = 30; // tracked points needed for full visual observation health
  double cross_modal_visual_health = 1.0; // diagnostics: combined image/view/track health
  double cross_modal_degrade = 0.0;     // diagnostics: joint LIO+VIO degradation risk
  double cross_modal_cov_scale = 1.0;   // diagnostics: covariance scale applied by cross-modal gate
  bool cross_modal_map_write_blocked = false; // diagnostics: visual map writes blocked this frame
  bool temporal_degrade_en = true;      // image-time/exposure/motion consistency gate
  double temporal_jitter_ref = 0.02;    // seconds of frame-time jitter that saturates temporal risk
  double temporal_lio_vio_gap_ref = 0.05; // seconds of LIO/VIO time gap that saturates risk
  double temporal_exposure_jump_ref = 0.35; // log inverse-exposure jump that saturates risk
  double temporal_cov_gain = 2.0;       // extra VIO covariance gain under temporal degradation
  double temporal_motion_gain = 0.75;   // high UAV motion amplifies temporal/exposure risk
  double temporal_map_risk_thresh = 0.65; // block visual map writes above this temporal risk
  double lio_vio_time_gap = 0.0;        // latest absolute time gap between paired LIO and VIO updates
  double last_img_time = -1.0;          // previous image timestamp used for jitter estimation
  double nominal_img_dt = -1.0;         // online nominal image frame interval
  double temporal_degrade = 0.0;        // diagnostics: fused temporal/exposure/motion risk
  double temporal_jitter_score = 0.0;   // diagnostics: frame interval jitter score
  double temporal_lio_vio_gap_score = 0.0; // diagnostics: LIO/VIO timing skew score
  double temporal_exposure_jump_score = 0.0; // diagnostics: inverse-exposure jump score
  double temporal_cov_scale = 1.0;      // diagnostics: covariance scale from temporal gate
  bool temporal_map_write_blocked = false; // diagnostics: visual map writes blocked by temporal gate
  bool ref_patch_adaptive_en = true;    // quality-aware reference patch selection and aging
  int ref_patch_max_age = 120;          // max frame age before a reference patch is considered stale
  double ref_patch_min_score = 0.30;    // skip tracking if the best reference is below this score
  double ref_patch_min_weight = 0.30;   // lower bound when reference quality weights VIO residuals
  double ref_patch_view_cos_min = 0.35; // minimum current/reference view cosine for full scoring
  double ref_patch_exposure_ref = 0.35; // log inverse-exposure jump that halves exposure score
  double ref_patch_switch_margin = 0.05; // keep current reference unless another is better by this
  double ref_patch_mean_score = 1.0;    // diagnostics: mean selected reference-patch score
  int ref_patch_switch_count = 0;       // diagnostics: references switched this frame
  int ref_patch_stale_count = 0;        // diagnostics: selected/current references past max age
  int ref_patch_low_score_count = 0;    // diagnostics: skipped points with low reference score
  double assessImageQuality(const cv::Mat &img);
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag = false;

  Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;
  // During recovery feat_map is the quarantined candidate submap and this map
  // owns the frozen last-known formal visual submap.
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> frozen_feat_map;
  unordered_map<VOXEL_LOCATION, int> sub_feat_map; 
  vector<VisualPoint *> retrieve_voxel_points;
  vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_;
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();
  VIOManager(const VIOManager &) = delete;
  VIOManager &operator=(const VIOManager &) = delete;
  void updateStateInverse(cv::Mat img, int level);
  void updateState(cv::Mat img, int level);
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg,
                               const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  void initializeVIO();
  void resetRuntimeState();
  void updateVisualRecoveryState();
  void beginVisualRecovery();
  void promoteVisualRecovery();
  void abortVisualRecovery();
  bool computeDirectionalFusionProjection(
      const Eigen::Matrix<double, 6, 6> &prior_pose_covariance,
      const Eigen::Matrix<double, 6, 6> &visual_pose_information,
      Eigen::Matrix<double, 6, 6> &projection);
  bool getImagePatch(const cv::Mat &img, const V2D &pc, float *patch_tmp, int level);
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  void computeJacobianAndUpdateEKF(cv::Mat img);
  void resetGrid();
  void updateVisualMapPoints(cv::Mat img);
  bool getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref,
                           const SE3 &T_cur_ref, const int level_ref, const int pyramid_level, const int halfpatch_size,
                           Matrix2d &A_cur_ref);
  bool getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3 &T_cur_ref,
                                     const int level_ref, Matrix2d &A_cur_ref);
  bool warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref,
                  const int search_level, const int pyramid_level, const int halfpatch_size, float *patch);
  void insertPointIntoVoxelMap(VisualPoint *pt_new);
  void plotTrackedPoints();
  void updateFrameState(StatesGroup state);
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void precomputeReferencePatches(int level);
  bool isVisualPointUsable(const VisualPoint *pt) const;
  double computeReferencePatchScore(const VisualPoint *pt, const Feature *ref_ftr) const;
  bool selectReferencePatch(VisualPoint *pt, Feature *&ref_ftr, double &ref_score, bool &switched, bool &stale);
  double computeViewObservationWeight(const VisualPoint *pt, const Feature *ref_ftr, const Matrix2d &A_cur_ref,
                                      const double ncc_score, double *incidence_score_out = nullptr,
                                      double *affine_score_out = nullptr) const;
  void recordVisualPointLifecycle(VisualPoint *pt, bool success, double obs_quality);
  void updateVisualPointLifecycle();
  void dumpDataForColmap();
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  V3F getInterpolatedPixel(const cv::Mat &img, const V2D &pc);
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;

private:
  struct VisualPointFrameSnapshot
  {
    VisualPoint *point = nullptr;
    bool is_bad = false;
    int success_count = 0;
    int fail_count = 0;
    double quality_score = 1.0;
    Feature *ref_patch = nullptr;
    bool has_ref_patch = false;
  };

  bool solveVioLinearization(const Matrix<double, DIM_STATE, DIM_STATE> &measurement_information,
                             const Matrix<double, DIM_STATE, 1> &measurement_gradient,
                             Matrix<double, DIM_STATE, 1> &solution);
  bool commitVioCovarianceCandidate();
  void markVioCovarianceFailure(const char *reason);
  bool isDirectPatchInFrame(const cv::Mat &img, const V2D &pc, int pyramid_level,
                            bool require_gradient) const;
  void beginVisualMapTransaction();
  void snapshotVisualPoint(VisualPoint *point);
  void commitVisualMapTransaction();
  void rollbackVisualMapTransaction();

  StatesGroup vio_update_entry_state_;
  StatesGroup vio_covariance_anchor_state_;
  Matrix<double, DIM_STATE, DIM_STATE> vio_update_prior_covariance_ =
      Matrix<double, DIM_STATE, DIM_STATE>::Identity();
  Matrix<double, DIM_STATE, DIM_STATE> vio_covariance_candidate_ =
      Matrix<double, DIM_STATE, DIM_STATE>::Identity();
  bool vio_update_context_active_ = false;
  bool vio_covariance_candidate_valid_ = false;
  bool visual_map_transaction_active_ = false;
  std::vector<VisualPointFrameSnapshot> visual_point_snapshots_;
  std::unordered_map<VisualPoint *, size_t> visual_point_snapshot_indices_;
  std::vector<VisualPoint *> pending_bad_point_cleanup_;
  std::vector<unsigned char> vio_final_support_mask_;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

#endif // VIO_H_
