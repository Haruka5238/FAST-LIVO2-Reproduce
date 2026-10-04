/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "recovery_scan_matcher.h"
#include "recovery_supervisor.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <vikit/camera_loader.h>
#include <atomic>
#include <limits>

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  bool stateEstimationAndMapping();
  void handleVIO();
  bool handleLIO();
  void publishLioDiagnostics(double frame_time);
  void savePCD();
  ImuProcessStatus processImu();
  void rebasePendingLidarAfterRejectedLio();
  void stashRejectedImuForNextLio();
  void mergeRejectedImuCarry(MeasureGroup &measure, double target_time);
  void requestCoordinatedReset(const char *reason, bool preserve_pending_sensor_data = false);
  void performCoordinatedReset();
  bool applyRecoveryCovarianceCheckpoint(StatesGroup &state, const char *reason);
  void captureRecoveryCheckpoint();
  void updateEstimatorRecoveryAfterValidLio();
  void stageEstimatorRecoveryMapCandidates();
  bool tryRecoveryScanMatch(StatesGroup &state);
  void updateRecoveryScanReference();
  bool sanitizeUndistortedCloudForVoxelGrid();
  
  bool sync_packages(LidarMeasureGroup &meas);
  bool prop_imu_once(StatesGroup &imu_prop_state, double dt, const V3D &acc_avr, const V3D &angvel_avr);
  void imu_prop_callback(const ros::TimerEvent &e);
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  void readParameters(ros::NodeHandle &nh);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, b_gyr_cov = 0.0001, b_acc_cov = 0.0001, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  int preprocess_point_filter_num_base_ = 1;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  bool allow_unbounded_pcd_save_ = false;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;
  int hilti_frame_counter_ = 0;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time = 0.0;
  double last_prop_time_offset_ = 0.0;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;
  double max_imu_propagation_interval_ = 0.5;
  double max_pose_covariance_trace_ = 1.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, ba_bg_est_en = true;
  std::atomic<bool> coordinated_reset_requested_{false};
  std::atomic<bool> coordinated_reset_preserve_pending_{false};
  struct RecoveryCheckpoint
  {
    StatesGroup state;
    double stamp = -1.0;
    double pose_covariance_trace = std::numeric_limits<double>::infinity();
    size_t formal_map_voxels = 0;
  };
  EstimatorRecoverySupervisor recovery_supervisor_;
  std::deque<RecoveryCheckpoint> recovery_checkpoint_cache_;
  RecoveryCheckpoint recovery_checkpoint_;
  bool recovery_checkpoint_valid_ = false;
  RecoveryScanMatcher recovery_scan_matcher_;
  PointCloudXYZI::Ptr recovery_scan_reference_cloud_;
  StatesGroup recovery_scan_reference_state_;
  bool recovery_scan_reference_valid_ = false;
  bool recovery_scan_fallback_active_ = false;
  double recovery_scan_fitness_ = std::numeric_limits<double>::infinity();
  double recovery_scan_rmse_ = std::numeric_limits<double>::infinity();
  int recovery_scan_correspondences_ = 0;
  double recovery_scan_min_direction_factor_ = 0.0;
  int recovery_scan_success_count_ = 0;
  bool skip_paired_vio_after_failed_lio_ = false;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size = 5, patch_size = 8, grid_n_width = 0, grid_n_height = 17, patch_pyrimid_level = 3;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  deque<sensor_msgs::Imu::ConstPtr> rejected_imu_carry_;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  ros::Publisher plane_pub;
  ros::Publisher voxel_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubNormal;
  ros::Publisher pubSubVisualMap;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubLaserCloudMap;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudDyn;
  ros::Publisher pubLaserCloudDynRmed;
  ros::Publisher pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  ros::Publisher mavros_pose_publisher;
  ros::Publisher pubLioDiag; // diagnostics: per-scan LIO quality metrics
  ros::Publisher pubVioDiag; // diagnostics: per-frame VIO quality metrics
  ros::Timer imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
  bool verbose_runtime_log_ = false;
  bool state_log_en_ = false;
  bool imu_log_en_ = false;
  bool pose_file_opened_ = false; // guards pose output file across rosbag replays

  // MODIFICATIONS: Innovation 5 — covariance-driven LIO->VIO trust coupling
  double lio_delta_info_ = 0.0;    // information gain of the last LIO update, tr(P_post^-1) - tr(P_pred^-1)
  double lio_info_nominal_ = -1.0; // EMA of typical combined info gain, calibrated online (kept for diag continuity)
  double lio_info_nominal_rot_ = -1.0; // EMA of rotation info gain (rad^-2)
  double lio_info_nominal_pos_ = -1.0; // EMA of translation info gain (m^-2)
  double vio_trust_factor_ = 1.0;  // clamped ratio fed to VIOManager::lio_info_ratio
  double lio_raw_trust_factor_ = 1.0; // unclamped-by-warmup LIO info ratio for diagnostics
  double lio_rot_info_ratio_ = 1.0; // rotation information gain ratio vs EMA baseline
  double lio_pos_info_ratio_ = 1.0; // translation information gain ratio vs EMA baseline
  bool lio_info_warmup_active_ = false; // true while the online info-gain baseline is still calibrating
  double cross_modal_lio_health_ = 1.0; // fused 0~1 LIO health used by cross-modal VIO/map-write gate

  // MODIFICATIONS: adaptive-degradation configuration (read in readParameters)
  bool adaptive_degeneracy_en_ = true;
  bool adaptive_coupled_degeneracy_en_ = true;
  double adaptive_degeneracy_alpha_ = 10.0;
  double adaptive_degeneracy_tau_ = 0.05;
  double adaptive_degeneracy_min_factor_ = 0.08;
  double adaptive_degeneracy_prior_eigen_ratio_floor_ = 1e-9;
  bool adaptive_normal_anisotropy_weight_en_ = true;
  bool adaptive_normal_balance_en_ = false;
  double adaptive_normal_balance_target_fraction_ = 0.35;
  double adaptive_normal_balance_min_weight_ = 0.20;
  bool adaptive_incidence_weight_en_ = false;
  double adaptive_incidence_min_weight_ = 0.10;
  double adaptive_incidence_weight_power_ = 1.0;
  bool adaptive_robust_kernel_en_ = true;
  std::string adaptive_robust_kernel_type_ = "cauchy";
  double adaptive_robust_kernel_delta_ = 3.0;
  double adaptive_robust_kernel_min_weight_ = 0.05;
  bool adaptive_lio_hard_outlier_reject_en_ = true;
  double adaptive_lio_hard_outlier_residual_sigma_ = 2.75;
  double adaptive_lio_hard_outlier_dynamic_score_ = 0.40;
  bool adaptive_scan_distortion_weight_en_ = false;
  double adaptive_scan_distortion_rot_ref_ = 0.25;
  double adaptive_scan_distortion_trans_ref_ = 0.20;
  double adaptive_scan_distortion_noise_gain_ = 2.0;
  double adaptive_scan_distortion_min_weight_ = 0.30;
  bool adaptive_lidar_return_quality_en_ = true;
  double adaptive_lidar_intensity_ref_ = 80.0;
  double adaptive_lidar_saturation_ref_ = 240.0;
  double adaptive_lidar_range_ref_ = 35.0;
  bool adaptive_lidar_range_adaptive_en_ = true;
  bool adaptive_lidar_range_candidate_source_en_ = true;
  double adaptive_lidar_range_ref_max_ = 80.0;
  double adaptive_lidar_range_quantile_ = 0.70;
  double adaptive_lidar_range_ref_ema_alpha_ = 0.15;
  double adaptive_lidar_density_ref_ = 20.0;
  double adaptive_lidar_return_min_weight_ = 0.25;
  bool adaptive_map_write_gate_en_ = true;
  double adaptive_map_write_residual_sigma_ = 2.5;
  double adaptive_map_write_min_score_ = 0.35;
  double adaptive_map_write_unmatched_risk_thresh_ = 0.45;
  double adaptive_map_write_min_return_ = 0.20;
  bool adaptive_map_write_recovery_en_ = true;
  int adaptive_map_write_recovery_trigger_frames_ = 8;
  double adaptive_map_write_recovery_budget_ratio_ = 0.05;
  double adaptive_map_write_recovery_min_return_ = 0.55;
  double adaptive_map_write_recovery_max_dynamic_ = 0.25;
  double adaptive_map_write_recovery_max_risk_ = 0.85;
  bool adaptive_dynamic_object_filter_en_ = true;
  double adaptive_dynamic_residual_sigma_ = 3.0;
  double adaptive_dynamic_map_reject_score_ = 0.65;
  double adaptive_dynamic_motion_gain_ = 0.50;
  double adaptive_total_lio_min_weight_ = 0.03;
  bool adaptive_plane_quality_weight_en_ = false;
  bool adaptive_vio_quality_en_ = true;
  bool adaptive_vio_cov_fusion_en_ = true;
  double adaptive_vio_cov_scale_max_ = 100.0;
  double adaptive_min_gen_quality_ = 0.15;
  double adaptive_min_update_quality_ = 0.18;
  double adaptive_min_ref_quality_ = 0.20;
  int adaptive_lio_info_warmup_frames_ = 50;
  double adaptive_map_starving_min_quality_ = 0.08;
  bool adaptive_visual_lifecycle_en_ = true;
  int adaptive_visual_lifecycle_max_fail_ = 6;
  double adaptive_visual_lifecycle_min_quality_ = 0.18;
  bool adaptive_visual_view_weight_en_ = true;
  double adaptive_visual_view_min_weight_ = 0.20;
  double adaptive_visual_view_min_cos_ = 0.17;
  double adaptive_visual_affine_det_ref_ = 3.0;
  bool adaptive_cross_modal_gate_en_ = true;
  double adaptive_cross_modal_cov_gain_ = 3.0;
  double adaptive_cross_modal_map_risk_thresh_ = 0.60;
  int adaptive_cross_modal_min_lio_points_ = 80;
  int adaptive_cross_modal_min_track_points_ = 30;
  bool adaptive_temporal_degrade_en_ = true;
  double adaptive_temporal_jitter_ref_ = 0.02;
  double adaptive_temporal_lio_vio_gap_ref_ = 0.05;
  double adaptive_temporal_exposure_jump_ref_ = 0.35;
  double adaptive_temporal_cov_gain_ = 2.0;
  double adaptive_temporal_motion_gain_ = 0.75;
  double adaptive_temporal_map_risk_thresh_ = 0.65;
  bool adaptive_ref_patch_adaptive_en_ = true;
  int adaptive_ref_patch_max_age_ = 120;
  double adaptive_ref_patch_min_score_ = 0.30;
  double adaptive_ref_patch_min_weight_ = 0.30;
  double adaptive_ref_patch_view_cos_min_ = 0.35;
  double adaptive_ref_patch_exposure_ref_ = 0.35;
  double adaptive_ref_patch_switch_margin_ = 0.05;
  bool adaptive_motion_degrade_en_ = true;
  double adaptive_motion_vel_max_ = 5.0;
  double adaptive_motion_gyr_max_ = 2.0;
  double adaptive_motion_jerk_ref_ = 50.0;
  double adaptive_motion_risk_tau_ = 0.10;
  double adaptive_motion_noise_gain_ = 2.0;
  bool adaptive_vibration_degrade_en_ = true;
  double adaptive_vibration_acc_ref_ = 4.0;
  double adaptive_vibration_gyr_ref_ = 0.6;
  double adaptive_vibration_noise_gain_ = 1.5;
  double adaptive_vibration_lpf_tau_ = 0.05;
  // Innovation 6 — preprocess adaptive filter parameters
  bool adaptive_preprocess_en_ = false;
  bool adaptive_preprocess_intensity_quality_en_ = true;
  int  adaptive_max_filter_num_ = 6;
  double adaptive_near_dist_ = 5.0;   // m
  double adaptive_far_dist_  = 30.0;  // m
  int adaptive_preprocess_bad_hysteresis_frames_ = 2;
  int adaptive_preprocess_good_hysteresis_frames_ = 6;
};
#endif
