/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include "common_lib.h"
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>
#include <fstream>

/// *************IMU Process and undistortion
enum class ImuProcessStatus
{
  Rejected,
  Initializing,
  PropagatedOnly,
  Ready
};

class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov_scale(const V3D &scaler);
  void set_acc_cov_scale(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_inv_expo_cov(const double &inv_expo);
  void set_imu_init_frame_num(const int &num);
  void set_max_propagation_interval(const double seconds);
  void set_imu_log_enabled(const bool enabled);
  void disable_imu();
  void disable_gravity_est();
  void disable_bias_est();
  void disable_exposure_est();
  ImuProcessStatus Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  bool UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  ofstream fout_imu;
  double IMU_mean_acc_norm = 1.0;
  V3D unbiased_gyr = V3D::Zero();

  // MODIFICATIONS: Innovation 3 — motion excitation degradation metrics.
  // Updated during forward propagation; consumed by LIVMapper/VIOManager to inflate
  // process noise and visual covariance under fast flight / aggressive rotation.
  bool motion_degrade_en = true; // enable velocity/gyro/jerk degradation modeling
  double motion_degrade = 0.0;  // 0~1 smoothed motion excitation risk
  double motion_vel_max = 5.0;  // m/s that saturates the velocity term
  double motion_gyr_max = 2.0;  // rad/s that saturates the angular-rate term
  double motion_jerk_ref = 50.0; // m/s^3 that saturates the jerk term
  double motion_risk_tau = 0.10; // seconds, rate-independent motion-risk smoothing
  double motion_noise_gain = 2.0; // process-noise inflation gain: Q *= 1 + gain * md^2
  bool vibration_degrade_en = true; // enable high-frequency IMU vibration degradation modeling
  double vibration_degrade = 0.0; // 0~1 high-frequency IMU vibration risk
  double vibration_acc_risk = 0.0; // acceleration high-pass sub-score
  double vibration_gyr_risk = 0.0; // gyro high-pass sub-score
  double vibration_acc_ref = 4.0; // m/s^2 high-pass acceleration residual that saturates vibration risk
  double vibration_gyr_ref = 0.6; // rad/s high-pass gyro residual that saturates vibration risk
  double vibration_noise_gain = 1.5; // additional process-noise inflation gain from vibration
  double vibration_lpf_tau = 0.05; // seconds, rate-independent low-pass time constant
  V3D acc_vib_lp_ = V3D::Zero(); // low-pass acceleration for vibration high-pass residual
  V3D gyr_vib_lp_ = V3D::Zero(); // low-pass gyro for vibration high-pass residual
  bool vib_lp_valid_ = false; // avoids a false vibration spike on the first propagated IMU sample
  V3D acc_last_ = V3D::Zero();   // previous-step acc for jerk (da/dt) computation
  bool acc_last_valid_ = false;   // avoids a false jerk spike on the first propagated IMU sample
  double scan_distortion_risk = 0.0;       // 0~1 intra-scan motion/distortion risk for the latest LIO frame
  double scan_distortion_rot_risk = 0.0;   // rotational component of scan_distortion_risk
  double scan_distortion_trans_risk = 0.0; // translational component of scan_distortion_risk
  double scan_time_span = 0.0;             // seconds covered by the current LiDAR scan segment
  double scan_distortion_rot_ref = 0.25;   // rad/scan that saturates rotational distortion risk
  double scan_distortion_trans_ref = 0.20; // m/scan that saturates translational distortion risk

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double cov_inv_expo;
  double first_lidar_time = 0.0;
  bool imu_time_init = false;
  bool imu_need_init = true;
  M3D Eye3d;
  V3D Zero3d;
  int lidar_type = AVIA;

private:
  bool IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
  bool Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  PointCloudXYZI pcl_wait_proc;
  sensor_msgs::ImuConstPtr last_imu;
  PointCloudXYZI::Ptr cur_pcl_un_;
  vector<Pose6D> IMUpose;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double last_prop_end_time;
  double time_last_scan;
  int init_iter_num = 0, MAX_INI_COUNT = 20;
  bool b_first_frame = true;
  bool imu_en = true;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  bool exposure_estimate_en = true;
  double max_propagation_interval_ = 0.5;
  bool imu_log_enabled_ = false;
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif
