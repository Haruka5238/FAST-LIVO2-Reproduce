/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "IMU_Processing.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
constexpr double kImuTimeTolerance = 1e-6;
constexpr double kPointTimeToleranceMs = 1e-3;

bool pointTimeLess(const PointType &lhs, const PointType &rhs)
{
  return lhs.curvature < rhs.curvature;
}

bool imuSampleFinite(const sensor_msgs::ImuConstPtr &sample)
{
  if (!sample || !std::isfinite(sample->header.stamp.toSec())) return false;
  const auto &acc = sample->linear_acceleration;
  const auto &gyr = sample->angular_velocity;
  return std::isfinite(acc.x) && std::isfinite(acc.y) && std::isfinite(acc.z) &&
         std::isfinite(gyr.x) && std::isfinite(gyr.y) && std::isfinite(gyr.z);
}

bool stateFinite(const StatesGroup &state)
{
  return state.rot_end.allFinite() && state.pos_end.allFinite() && state.vel_end.allFinite() &&
         state.bias_g.allFinite() && state.bias_a.allFinite() && state.gravity.allFinite() &&
         std::isfinite(state.inv_expo_time) && state.cov.allFinite();
}
}

ImuProcess::ImuProcess() : Eye3d(M3D::Identity()),
                           Zero3d(0, 0, 0), b_first_frame(true), imu_need_init(true)
{
  init_iter_num = 0;
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_acc = V3D(0.1, 0.1, 0.1);
  cov_inv_expo = 0.2;
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  last_prop_end_time = 0.0;
  time_last_scan = 0.0;
  Lid_offset_to_IMU = Zero3d;
  Lid_rot_to_IMU = Eye3d;
  last_imu.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
  // MODIFICATIONS: Innovation 3 — explicit initialization so the first frame never
  // inherits undefined motion_degrade from heap memory.
  motion_degrade = 0.0;
  vibration_degrade = 0.0;
  vibration_acc_risk = 0.0;
  vibration_gyr_risk = 0.0;
  scan_distortion_risk = 0.0;
  scan_distortion_rot_risk = 0.0;
  scan_distortion_trans_risk = 0.0;
  scan_time_span = 0.0;
  acc_vib_lp_ = Zero3d;
  gyr_vib_lp_ = Zero3d;
  vib_lp_valid_ = false;
  acc_last_ = Zero3d;
  acc_last_valid_ = false;
}

ImuProcess::~ImuProcess() {}

void ImuProcess::set_max_propagation_interval(const double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0.0)
  {
    throw std::invalid_argument("IMU maximum propagation interval must be finite and positive");
  }
  max_propagation_interval_ = seconds;
}

void ImuProcess::set_imu_log_enabled(const bool enabled)
{
  imu_log_enabled_ = enabled;
  if (!imu_log_enabled_ && fout_imu.is_open()) fout_imu.close();
}

void ImuProcess::Reset()
{
  ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  last_prop_end_time = 0.0;
  time_last_scan = 0.0;
  imu_time_init = false;
  imu_need_init = imu_en;
  b_first_frame = true;
  init_iter_num = 0;
  IMUpose.clear();
  pcl_wait_proc.clear();
  last_imu.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
  IMU_mean_acc_norm = 1.0;
  unbiased_gyr = Zero3d;
  // MODIFICATIONS: Innovation 3 — clear motion state on reset so a re-localisation or
  // rosbag rewind does not carry forward stale high-motion history into the new session.
  motion_degrade = 0.0;
  vibration_degrade = 0.0;
  vibration_acc_risk = 0.0;
  vibration_gyr_risk = 0.0;
  scan_distortion_risk = 0.0;
  scan_distortion_rot_risk = 0.0;
  scan_distortion_trans_risk = 0.0;
  scan_time_span = 0.0;
  acc_vib_lp_ = Zero3d;
  gyr_vib_lp_ = Zero3d;
  vib_lp_valid_ = false;
  acc_last_ = Zero3d;
  acc_last_valid_ = false;
}

void ImuProcess::disable_imu()
{
  cout << "IMU Disabled !!!!!" << endl;
  imu_en = false;
  imu_need_init = false;
}

void ImuProcess::disable_gravity_est()
{
  cout << "Online Gravity Estimation Disabled !!!!!" << endl;
  gravity_est_en = false;
}

void ImuProcess::disable_bias_est()
{
  cout << "Bias Estimation Disabled !!!!!" << endl;
  ba_bg_est_en = false;
}

void ImuProcess::disable_exposure_est()
{
  cout << "Online Time Offset Estimation Disabled !!!!!" << endl;
  exposure_estimate_en = false;
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  Lid_offset_to_IMU = T.block<3, 1>(0, 3);
  Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU = rot;
}

void ImuProcess::set_gyr_cov_scale(const V3D &scaler) { cov_gyr = scaler; }

void ImuProcess::set_acc_cov_scale(const V3D &scaler) { cov_acc = scaler; }

void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_inv_expo_cov(const double &inv_expo) { cov_inv_expo = inv_expo; }

void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

void ImuProcess::set_imu_init_frame_num(const int &num) { MAX_INI_COUNT = std::max(num, 1); }

bool ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  if (meas.imu.empty()) return false;
  for (const auto &imu : meas.imu)
  {
    if (!imuSampleFinite(imu)) return false;
  }

  const bool first_frame = b_first_frame;
  if (first_frame)
  {
    Reset();
    // first_lidar_time = meas.lidar_frame_beg_time;
    // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
  }

  int next_count = first_frame ? 0 : N;
  V3D next_mean_acc = first_frame ? Zero3d : mean_acc;
  V3D next_mean_gyr = first_frame ? Zero3d : mean_gyr;

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    V3D cur_acc, cur_gyr;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    next_count++;
    next_mean_acc += (cur_acc - next_mean_acc) / next_count;
    next_mean_gyr += (cur_gyr - next_mean_gyr) / next_count;

    // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc -
    // mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N); cov_gyr
    // = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr -
    // mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

  }
  const double mean_acc_norm = next_mean_acc.norm();
  if (!std::isfinite(mean_acc_norm) || mean_acc_norm <= 1e-6) return false;
  StatesGroup initialized_state = state_inout;
  initialized_state.gravity = -next_mean_acc / mean_acc_norm * G_m_s2;
  initialized_state.rot_end = Eye3d;
  initialized_state.bias_g = Zero3d;
  if (!stateFinite(initialized_state)) return false;

  mean_acc = next_mean_acc;
  mean_gyr = next_mean_gyr;
  N = next_count;
  b_first_frame = false;
  IMU_mean_acc_norm = mean_acc_norm;
  last_imu = meas.imu.back();
  state_inout = initialized_state;
  return true;
}

bool ImuProcess::Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  pcl_out.clear();
  if (!meas.lidar || meas.lidar->empty() || !std::isfinite(meas.lidar_frame_beg_time)) return false;
  pcl_out = *(meas.lidar);
  for (const auto &point : pcl_out.points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !std::isfinite(point.curvature))
    {
      pcl_out.clear();
      return false;
    }
  }
  /*** sort point clouds by offset time ***/
  const double &pcl_beg_time = meas.lidar_frame_beg_time;
  sort(pcl_out.points.begin(), pcl_out.points.end(), pointTimeLess);
  const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  if (!std::isfinite(pcl_end_time))
  {
    pcl_out.clear();
    return false;
  }
  const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);
  scan_time_span = std::max(pcl_end_offset_time, 0.0);
  scan_distortion_risk = 0.0;
  scan_distortion_rot_risk = 0.0;
  scan_distortion_trans_risk = 0.0;
  motion_degrade = 0.0;
  vibration_degrade = 0.0;
  vibration_acc_risk = 0.0;
  vibration_gyr_risk = 0.0;

  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt = 0;

  const bool first_frame = b_first_frame;
  if (first_frame)
  {
    dt = 0.1;
  }
  else { dt = pcl_beg_time - time_last_scan; }

  if (!std::isfinite(dt) || dt < -kImuTimeTolerance)
  {
    pcl_out.clear();
    return false;
  }
  dt = std::max(dt, 0.0);
  // for (size_t i = 0; i < pcl_out->points.size(); i++) {
  //   if (dt < pcl_out->points[i].curvature) {
  //     dt = pcl_out->points[i].curvature;
  //   }
  // }
  // dt = dt / (double)1000;
  // std::cout << "dt:" << dt << std::endl;
  // double dt = pcl_out->points.back().curvature / double(1000);

  /* covariance propagation */
  // M3D acc_avr_skew;
  M3D Exp_f = Exp(state_inout.bias_g, dt);

  F_x.setIdentity();
  cov_w.setZero();

  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  F_x.block<3, 3>(0, 10) = Eye3d * dt;
  F_x.block<3, 3>(3, 7) = Eye3d * dt;
  // F_x.block<3, 3>(6, 0)  = - R_imu * acc_avr_skew * dt;
  // F_x.block<3, 3>(6, 12) = - R_imu * dt;
  // F_x.block<3, 3>(6, 15) = Eye3d * dt;

  cov_w.block<3, 3>(10, 10).diagonal() = cov_gyr * dt * dt; // for omega in constant model
  cov_w.block<3, 3>(7, 7).diagonal() = cov_acc * dt * dt; // for velocity in constant model
  // cov_w.block<3, 3>(6, 6) =
  //     R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
  // cov_w.block<3, 3>(9, 9).diagonal() =
  //     cov_bias_gyr * dt * dt; // bias gyro covariance
  // cov_w.block<3, 3>(12, 12).diagonal() =
  //     cov_bias_acc * dt * dt; // bias acc covariance

  // std::cout << "before propagete:" << state_inout.cov.diagonal().transpose()
  //           << std::endl;
  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
  // std::cout << "cov_w:" << cov_w.diagonal().transpose() << std::endl;
  // std::cout << "after propagete:" << state_inout.cov.diagonal().transpose()
  //           << std::endl;
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;

  if (!stateFinite(state_inout))
  {
    pcl_out.clear();
    return false;
  }
  b_first_frame = false;
  time_last_scan = pcl_beg_time;
  meas.last_lio_update_time = pcl_end_time;

  if (lidar_type != L515)
  {
    auto it_pcl = pcl_out.points.end() - 1;
    double dt_j = 0.0;
    for(; it_pcl != pcl_out.points.begin(); it_pcl--)
    {
        dt_j= pcl_end_offset_time - it_pcl->curvature/double(1000);
        M3D R_jk(Exp(state_inout.bias_g, - dt_j));
        V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
        // Using rotation and translation to un-distort points
        V3D p_jk;
        p_jk = - state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;
  
        V3D P_compensate =  R_jk * P_j + p_jk;
  
        /// save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);
    }
  }
  return true;
}


bool ImuProcess::UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  double t0 = omp_get_wtime();
  pcl_out.clear();
  if (lidar_meas.measures.empty())
  {
    ROS_WARN_THROTTLE(1.0, "Reject IMU processing: empty MeasureGroup deque");
    return false;
  }
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  MeasureGroup &meas = lidar_meas.measures.back();
  if (lidar_meas.lio_vio_flg != LIO && lidar_meas.lio_vio_flg != VIO)
  {
    ROS_WARN_THROTTLE(1.0, "Reject IMU processing: unsupported propagation mode %d",
                      static_cast<int>(lidar_meas.lio_vio_flg));
    return false;
  }
  const double prop_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;
  if (!std::isfinite(prop_end_time) || !std::isfinite(last_prop_end_time) ||
      prop_end_time < last_prop_end_time - kImuTimeTolerance)
  {
    ROS_WARN_THROTTLE(1.0, "Reject IMU processing: invalid/backward propagation interval %.9f -> %.9f",
                      last_prop_end_time, prop_end_time);
    return false;
  }
  const double propagation_interval = prop_end_time - last_prop_end_time;
  if (propagation_interval > max_propagation_interval_ + kImuTimeTolerance)
  {
    ROS_WARN_THROTTLE(1.0, "Reject IMU processing: forward propagation gap %.6f s exceeds %.6f s",
                      propagation_interval, max_propagation_interval_);
    return false;
  }
  const bool same_time_vio = lidar_meas.lio_vio_flg == VIO &&
                             !imu_need_init &&
                             std::fabs(meas.vio_time - meas.lio_time) <= kImuTimeTolerance &&
                             std::fabs(prop_end_time - last_prop_end_time) <= kImuTimeTolerance;
  if (meas.imu.empty() && !same_time_vio)
  {
    ROS_WARN_THROTTLE(1.0, "Reject IMU processing: no new IMU sample for propagation");
    return false;
  }
  if (!imuSampleFinite(last_imu))
  {
    ROS_WARN_THROTTLE(1.0, "Reject IMU processing: invalid previous IMU sample");
    return false;
  }
  double previous_imu_time = last_imu->header.stamp.toSec();
  bool has_new_imu = false;
  for (const auto &imu : meas.imu)
  {
    if (!imuSampleFinite(imu))
    {
      ROS_WARN_THROTTLE(1.0, "Reject IMU processing: null or non-finite IMU sample");
      return false;
    }
    const double imu_time = imu->header.stamp.toSec();
    if (imu_time < previous_imu_time - kImuTimeTolerance || imu_time > prop_end_time + kImuTimeTolerance)
    {
      ROS_WARN_THROTTLE(1.0, "Reject IMU processing: non-monotonic/out-of-window IMU timestamp %.9f",
                        imu_time);
      return false;
    }
    if (imu_time > last_prop_end_time + kImuTimeTolerance) has_new_imu = true;
    previous_imu_time = imu_time;
  }
  if (!has_new_imu && !same_time_vio)
  {
    ROS_WARN_THROTTLE(1.0, "Reject propagation: IMU batch has no sample newer than the last propagation");
    return false;
  }
  const double mean_acc_norm = mean_acc.norm();
  if (!std::isfinite(mean_acc_norm) || mean_acc_norm <= 1e-6 || !stateFinite(state_inout))
  {
    ROS_WARN_THROTTLE(1.0, "Reject IMU processing: invalid propagation state or acceleration scale");
    return false;
  }
  // cout<<"meas.imu.size: "<<meas.imu.size()<<endl;
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double prop_beg_time = last_prop_end_time;
  // printf("[ IMU ] undistort input size: %zu \n", lidar_meas.pcl_proc_cur->points.size());
  // printf("[ IMU ] IMU data sequence size: %zu \n", meas.imu.size());
  // printf("[ IMU ] lidar_scan_index_now: %d \n", lidar_meas.lidar_scan_index_now);

  scan_time_span = std::max(prop_end_time - prop_beg_time, 0.0);
  scan_distortion_risk = 0.0;
  scan_distortion_rot_risk = 0.0;
  scan_distortion_trans_risk = 0.0;

  /*** cut lidar point based on the propagation-start time and required
   * propagation-end time ***/
  // const double pcl_offset_time = (prop_end_time -
  // lidar_meas.lidar_frame_beg_time) * 1000.; // the offset time w.r.t scan
  // start time auto pcl_it = lidar_meas.pcl_proc_cur->points.begin() +
  // lidar_meas.lidar_scan_index_now; auto pcl_it_end =
  // lidar_meas.lidar->points.end(); printf("[ IMU ] pcl_it->curvature: %lf
  // pcl_offset_time: %lf \n", pcl_it->curvature, pcl_offset_time); while
  // (pcl_it != pcl_it_end && pcl_it->curvature <= pcl_offset_time)
  // {
  //   pcl_wait_proc.push_back(*pcl_it);
  //   pcl_it++;
  //   lidar_meas.lidar_scan_index_now++;
  // }

  // cout<<"pcl_out.size(): "<<pcl_out.size()<<endl;
  // cout<<"pcl_offset_time:  "<<pcl_offset_time<<"pcl_it->curvature:
  // "<<pcl_it->curvature<<endl;
  // cout<<"lidar_meas.lidar_scan_index_now:"<<lidar_meas.lidar_scan_index_now<<endl;

  // printf("[ IMU ] last propagation end time: %lf \n", lidar_meas.last_lio_update_time);
  if (lidar_meas.lio_vio_flg == LIO)
  {
    pcl_wait_proc.clear();
    if (lidar_meas.pcl_proc_cur)
    {
      pcl_wait_proc.reserve(lidar_meas.pcl_proc_cur->size());
      int invalid_point_count = 0;
      int out_of_window_point_count = 0;
      const double max_point_offset_ms = propagation_interval * 1000.0;
      for (const auto &point : lidar_meas.pcl_proc_cur->points)
      {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.curvature))
        {
          invalid_point_count++;
          continue;
        }
        if (point.curvature < -kPointTimeToleranceMs ||
            point.curvature > max_point_offset_ms + kPointTimeToleranceMs)
        {
          out_of_window_point_count++;
          continue;
        }
        PointType canonical_point = point;
        canonical_point.curvature = static_cast<float>(
            std::max(0.0, std::min(static_cast<double>(point.curvature), max_point_offset_ms)));
        pcl_wait_proc.push_back(canonical_point);
      }
      if (invalid_point_count > 0)
      {
        ROS_WARN_THROTTLE(1.0, "Discard %d non-finite LiDAR points before undistortion", invalid_point_count);
      }
      if (out_of_window_point_count > 0)
      {
        ROS_WARN_THROTTLE(1.0, "Discard %d LiDAR points outside the current propagation window",
                          out_of_window_point_count);
      }
      std::sort(pcl_wait_proc.points.begin(), pcl_wait_proc.points.end(), pointTimeLess);
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "Propagate without LIO correction: null current point cloud");
    }
    lidar_meas.lidar_scan_index_now = 0;
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
  }

  // printf("[ IMU ] pcl_wait_proc size: %zu \n", pcl_wait_proc.points.size());

  // sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // lidar_meas.debug_show();
  // cout<<"UndistortPcl [ IMU ]: Process lidar from "<<prop_beg_time<<" to
  // "<<prop_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to
  //          "<<imu_end_time<<endl;
  // cout<<"[ IMU ]: point size: "<<lidar_meas.lidar->points.size()<<endl;

  /*** Initialize IMU pose ***/
  // IMUpose.clear();

  /*** forward propagation at each imu point ***/
  V3D acc_imu(acc_s_last), angvel_avr(angvel_last), acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  // cout << "[ IMU ] input state: " << state_inout.vel_end.transpose() << " " << state_inout.pos_end.transpose() << endl;
  M3D R_imu(state_inout.rot_end);
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt, dt_all = 0.0;
  double offs_t;
  // double imu_time;
  double tau;
  if (!imu_time_init)
  {
    // imu_time = v_imu.front()->header.stamp.toSec() - first_lidar_time;
    // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);
    tau = 1.0;
    imu_time_init = true;
  }
  else
  {
    tau = state_inout.inv_expo_time;
    // ROS_ERROR("tau: %.6f !!!!!!", tau);
  }
  // state_inout.cov(6, 6) = 0.01;

  // ROS_ERROR("lidar_meas.lio_vio_flg");
  // cout<<"lidar_meas.lio_vio_flg: "<<lidar_meas.lio_vio_flg<<endl;
  switch (lidar_meas.lio_vio_flg)
  {
  case LIO:
  case VIO:
    dt = 0;
    for (size_t i = 0; i + 1 < v_imu.size(); i++)
    {
      auto head = v_imu[i];
      auto tail = v_imu[i + 1];

      if (tail->header.stamp.toSec() < prop_beg_time) continue;

      angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x), 0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
          0.5 * (head->angular_velocity.z + tail->angular_velocity.z);

      // angvel_avr<<tail->angular_velocity.x, tail->angular_velocity.y,
      // tail->angular_velocity.z;

      acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x), 0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
          0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

      // cout<<"angvel_avr: "<<angvel_avr.transpose()<<endl;
      // cout<<"acc_avr: "<<acc_avr.transpose()<<endl;

      // #ifdef DEBUG_PRINT
      if (imu_log_enabled_ && fout_imu.is_open())
      {
        fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " "
                 << angvel_avr.transpose() << " " << acc_avr.transpose() << '\n';
      }
      // #endif

      // imu_time = head->header.stamp.toSec() - first_lidar_time;

      angvel_avr -= state_inout.bias_g;
      acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

      if (head->header.stamp.toSec() < prop_beg_time)
      {
        // printf("00 \n");
        dt = tail->header.stamp.toSec() - last_prop_end_time;
        offs_t = tail->header.stamp.toSec() - prop_beg_time;
      }
      else if (i + 2 < v_imu.size())
      {
        // printf("11 \n");
        dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        offs_t = tail->header.stamp.toSec() - prop_beg_time;
      }
      else
      {
        // printf("22 \n");
        dt = prop_end_time - head->header.stamp.toSec();
        offs_t = prop_end_time - prop_beg_time;
      }

      if (!std::isfinite(dt) || !std::isfinite(offs_t) || dt < -kImuTimeTolerance ||
          dt > max_propagation_interval_ + kImuTimeTolerance)
      {
        ROS_WARN_THROTTLE(1.0, "Reject IMU processing: invalid integration interval %.9f", dt);
        pcl_wait_proc.clear();
        IMUpose.clear();
        return false;
      }
      dt = std::max(dt, 0.0);

      dt_all += dt;
      // printf("[ LIO Propagation ] dt: %lf \n", dt);

      /* covariance propagation */
      M3D acc_avr_skew;
      M3D Exp_f = Exp(angvel_avr, dt);
      acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

      F_x.setIdentity();
      cov_w.setZero();

      F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
      if (ba_bg_est_en) F_x.block<3, 3>(0, 10) = -Eye3d * dt;
      // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
      F_x.block<3, 3>(3, 7) = Eye3d * dt;
      F_x.block<3, 3>(7, 0) = -R_imu * acc_avr_skew * dt;
      if (ba_bg_est_en) F_x.block<3, 3>(7, 13) = -R_imu * dt;
      if (gravity_est_en) F_x.block<3, 3>(7, 16) = Eye3d * dt;

      // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);
      // F_x(6,6) = 0.25 * 2 * CV_PI * 0.5 * cos(2 * CV_PI * 0.5 * imu_time) * (-tau*tau); F_x(18,18) = 0.00001;

      // MODIFICATIONS: Innovation 3 — motion-excitation adaptive process noise.
      // High velocity / angular rate increases scan distortion and motion blur; the
      // constant-noise model then over-trusts propagation. Inflate gyro/acc process
      // noise quadratically with the smoothed excitation risk.
      {
        double vel_risk  = std::min(vel_imu.norm() / std::max(motion_vel_max, 1e-3), 1.0);
        double gyro_risk = std::min(angvel_avr.norm() / std::max(motion_gyr_max, 1e-3), 1.0);
        // jerk (da/dt) captures sudden acceleration changes (sharp turns,
        // emergency stops) invisible to velocity / angular-rate magnitude alone
        V3D acc_jerk = acc_last_valid_ ? (acc_avr - acc_last_) / std::max(dt, 1e-6) : Zero3d;
        double jerk_risk = std::min(acc_jerk.norm() / std::max(motion_jerk_ref, 1e-3), 1.0);
        double vibration_risk = 0.0;
        if (vibration_degrade_en && vib_lp_valid_)
        {
          const double acc_ref = std::max(vibration_acc_ref, 1e-3);
          const double gyr_ref = std::max(vibration_gyr_ref, 1e-3);
          double acc_hf_risk = std::min((acc_avr - acc_vib_lp_).norm() / acc_ref, 1.0);
          double gyr_hf_risk = std::min((angvel_avr - gyr_vib_lp_).norm() / gyr_ref, 1.0);
          vibration_acc_risk = acc_hf_risk;
          vibration_gyr_risk = gyr_hf_risk;
          vibration_risk = 0.55 * acc_hf_risk + 0.45 * gyr_hf_risk;
          const double safe_dt = std::max(dt, 0.0);
          const double lp_tau = std::max(vibration_lpf_tau, 1e-3);
          const double lp_alpha = std::exp(-safe_dt / lp_tau);
          acc_vib_lp_ = lp_alpha * acc_vib_lp_ + (1.0 - lp_alpha) * acc_avr;
          gyr_vib_lp_ = lp_alpha * gyr_vib_lp_ + (1.0 - lp_alpha) * angvel_avr;
        }
        else if (vibration_degrade_en)
        {
          acc_vib_lp_ = acc_avr;
          gyr_vib_lp_ = angvel_avr;
          vibration_acc_risk = 0.0;
          vibration_gyr_risk = 0.0;
          vib_lp_valid_ = true;
        }
        else
        {
          vibration_degrade = 0.0;
          vibration_acc_risk = 0.0;
          vibration_gyr_risk = 0.0;
          vib_lp_valid_ = false;
        }

        double md = 0.35 * vel_risk + 0.35 * gyro_risk + 0.30 * jerk_risk;
        if (vibration_degrade_en)
        {
          const double safe_dt = std::max(dt, 0.0);
          const double risk_tau = std::max(3.0 * vibration_lpf_tau, 0.05);
          const double risk_alpha = std::exp(-safe_dt / risk_tau);
          vibration_degrade = risk_alpha * vibration_degrade + (1.0 - risk_alpha) * vibration_risk;
        }
        if (motion_degrade_en)
        {
          const double motion_alpha = std::exp(-std::max(dt, 0.0) / std::max(motion_risk_tau, 1e-3));
          motion_degrade = motion_alpha * motion_degrade + (1.0 - motion_alpha) * md;
        }
        else
        {
          motion_degrade = 0.0;
        }
        acc_last_ = acc_avr;
        acc_last_valid_ = true;
      }
      double motion_scale = 1.0 + (motion_degrade_en ? std::max(motion_noise_gain, 0.0) *
                                                          motion_degrade * motion_degrade
                                                    : 0.0) +
                            (vibration_degrade_en ? std::max(vibration_noise_gain, 0.0) *
                                                        vibration_degrade * vibration_degrade
                                                  : 0.0);

      if (exposure_estimate_en) cov_w(6, 6) = cov_inv_expo * dt * dt;
      cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt * motion_scale;
      cov_w.block<3, 3>(7, 7) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt * motion_scale;
      cov_w.block<3, 3>(10, 10).diagonal() = cov_bias_gyr * dt * dt; // bias gyro covariance
      cov_w.block<3, 3>(13, 13).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance

      state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
      // state_inout.cov.block<18,18>(0,0) = F_x.block<18,18>(0,0) *
      // state_inout.cov.block<18,18>(0,0) * F_x.block<18,18>(0,0).transpose() +
      // cov_w.block<18,18>(0,0);

      // tau = tau + 0.25 * 2 * CV_PI * 0.5 * cos(2 * CV_PI * 0.5 * imu_time) *
      // (-tau*tau) * dt;

      // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);

      /* propogation of IMU attitude */
      R_imu = R_imu * Exp_f;

      /* Specific acceleration (global frame) of IMU */
      acc_imu = R_imu * acc_avr + state_inout.gravity;

      /* propogation of IMU */
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

      /* velocity of IMU */
      vel_imu = vel_imu + acc_imu * dt;

      /* save the poses at each IMU measurements */
      angvel_last = angvel_avr;
      acc_s_last = acc_imu;

      // cout<<setw(20)<<"offset_t: "<<offs_t<<"tail->header.stamp.toSec():
      // "<<tail->header.stamp.toSec()<<endl; printf("[ LIO Propagation ]
      // offs_t: %lf \n", offs_t);
      IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }

    // unbiased_gyr = V3D(IMUpose.back().gyr[0], IMUpose.back().gyr[1], IMUpose.back().gyr[2]);
    // cout<<"prop end - start: "<<prop_end_time - prop_beg_time<<" dt_all: "<<dt_all<<endl;
    lidar_meas.last_lio_update_time = prop_end_time;
    // dt = prop_end_time - imu_end_time;
    // printf("[ LIO Propagation ] dt: %lf \n", dt);
    break;
  }

  state_inout.vel_end = vel_imu;
  state_inout.rot_end = R_imu;
  state_inout.pos_end = pos_imu;
  state_inout.inv_expo_time = tau;

  // MODIFICATIONS: Innovation 11 - scan-distortion risk from intra-scan pose delta.
  // Fast flight, sharp yawing and vibration leave residual distortion even after
  // first-order IMU undistortion. Estimate the risk from the scan start/end pose
  // and feed it to LIO measurement weighting.
  if (lidar_meas.lio_vio_flg == LIO && !IMUpose.empty() && scan_time_span > 1e-4)
  {
    M3D R_start;
    V3D p_start;
    R_start << MAT_FROM_ARRAY(IMUpose.front().rot);
    p_start << VEC_FROM_ARRAY(IMUpose.front().pos);

    const M3D relative_rotation = R_start.transpose() * state_inout.rot_end;
    double rot_delta = Log(relative_rotation).norm();
    double trans_delta = (state_inout.pos_end - p_start).norm();
    double rot_ref = std::max(scan_distortion_rot_ref, 1e-3);
    double trans_ref = std::max(scan_distortion_trans_ref, 1e-3);

    scan_distortion_rot_risk = std::max(0.0, std::min(rot_delta / rot_ref, 1.0));
    scan_distortion_trans_risk = std::max(0.0, std::min(trans_delta / trans_ref, 1.0));
    double residual_motion_risk = motion_degrade_en ? 0.20 * motion_degrade : 0.0;
    if (vibration_degrade_en)
    {
      residual_motion_risk = (motion_degrade_en ? 0.12 * motion_degrade : 0.0) +
                             0.08 * vibration_degrade;
    }
    scan_distortion_risk = std::max(0.0, std::min(0.45 * scan_distortion_rot_risk +
                                                   0.35 * scan_distortion_trans_risk +
                                                   residual_motion_risk,
                                                   1.0));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  // if (imu_end_time>prop_beg_time)
  // {
  //   double note = prop_end_time > imu_end_time ? 1.0 : -1.0;
  //   dt = note * (prop_end_time - imu_end_time);
  //   state_inout.vel_end = vel_imu + note * acc_imu * dt;
  //   state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
  //   state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 *
  //   acc_imu * dt * dt;
  // }
  // else
  // {
  //   double note = prop_end_time > prop_beg_time ? 1.0 : -1.0;
  //   dt = note * (prop_end_time - prop_beg_time);
  //   state_inout.vel_end = vel_imu + note * acc_imu * dt;
  //   state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
  //   state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 *
  //   acc_imu * dt * dt;
  // }

  // cout<<"[ Propagation ] output state: "<<state_inout.vel_end.transpose() <<
  // state_inout.pos_end.transpose()<<endl;

  last_imu = v_imu.back();
  last_prop_end_time = prop_end_time;

  double t1 = omp_get_wtime();

  // auto pos_liD_e = state_inout.pos_end + state_inout.rot_end *
  // Lid_offset_to_IMU; auto R_liD_e   = state_inout.rot_end * Lidar_R_to_IMU;

  // cout<<"[ IMU ]: vel "<<state_inout.vel_end.transpose()<<" pos
  // "<<state_inout.pos_end.transpose()<<"
  // ba"<<state_inout.bias_a.transpose()<<" bg
  // "<<state_inout.bias_g.transpose()<<endl; cout<<"propagated cov:
  // "<<state_inout.cov.diagonal().transpose()<<endl;

  //   cout<<"UndistortPcl Time:";
  //   for (auto it = IMUpose.begin(); it != IMUpose.end(); ++it) {
  //     cout<<it->offset_time<<" ";
  //   }
  //   cout<<endl<<"UndistortPcl size:"<<IMUpose.size()<<endl;
  //   cout<<"Undistorted pcl_out.size: "<<pcl_out.size()
  //          <<"lidar_meas.size: "<<lidar_meas.lidar->points.size()<<endl;
  if (pcl_wait_proc.points.empty())
  {
    IMUpose.clear();
    return true;
  }

  /*** undistort each lidar point (backward propagation), ONLY working for LIO
   * update ***/
  if (lidar_meas.lio_vio_flg == LIO)
  {
    auto it_pcl = pcl_wait_proc.points.end() - 1;
    M3D extR_Ri(Lid_rot_to_IMU.transpose() * state_inout.rot_end.transpose());
    V3D exrR_extT(Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU);
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << MAT_FROM_ARRAY(head->rot);
      acc_imu << VEC_FROM_ARRAY(head->acc);
      // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      angvel_avr << VEC_FROM_ARRAY(head->gyr);

      // printf("head->offset_time: %lf \n", head->offset_time);
      // printf("it_pcl->curvature: %lf pt dt: %lf \n", it_pcl->curvature,
      // it_pcl->curvature / double(1000) - head->offset_time);

      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
      {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* Transform to the 'end' frame */
        M3D R_i(R_imu * Exp(angvel_avr, dt));
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - state_inout.pos_end);

        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        // V3D P_compensate = Lid_rot_to_IMU.transpose() *
        // (state_inout.rot_end.transpose() * (R_i * (Lid_rot_to_IMU * P_i +
        // Lid_offset_to_IMU) + T_ei) - Lid_offset_to_IMU);
        V3D P_compensate = (extR_Ri * (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) - exrR_extT);

        /// save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_wait_proc.points.begin()) break;
      }
    }
    pcl_out = pcl_wait_proc;
    pcl_wait_proc.clear();
    IMUpose.clear();
  }
  // printf("[ IMU ] time forward: %lf, backward: %lf.\n", t1 - t0, omp_get_wtime() - t1);
  return true;
}

ImuProcessStatus ImuProcess::Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat,
                                     PointCloudXYZI::Ptr cur_pcl_un_)
{
  if (!cur_pcl_un_)
  {
    ROS_ERROR_THROTTLE(1.0, "Reject IMU processing: null output point cloud");
    return ImuProcessStatus::Rejected;
  }
  cur_pcl_un_->clear();
  if (!imu_en)
  {
    StatesGroup propagated = stat;
    const double previous_update_time = lidar_meas.last_lio_update_time;
    const bool previous_first_frame = b_first_frame;
    const double previous_time_last_scan = time_last_scan;
    const double previous_scan_time_span = scan_time_span;
    const double previous_scan_distortion_risk = scan_distortion_risk;
    const double previous_scan_distortion_rot_risk = scan_distortion_rot_risk;
    const double previous_scan_distortion_trans_risk = scan_distortion_trans_risk;
    const double previous_motion_degrade = motion_degrade;
    const double previous_vibration_degrade = vibration_degrade;
    const double previous_vibration_acc_risk = vibration_acc_risk;
    const double previous_vibration_gyr_risk = vibration_gyr_risk;
    bool output_valid = Forward_without_imu(lidar_meas, propagated, *cur_pcl_un_) && stateFinite(propagated);
    if (output_valid)
    {
      for (const auto &point : cur_pcl_un_->points)
      {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.curvature))
        {
          output_valid = false;
          break;
        }
      }
    }
    if (!output_valid)
    {
      lidar_meas.last_lio_update_time = previous_update_time;
      cur_pcl_un_->clear();
      b_first_frame = previous_first_frame;
      time_last_scan = previous_time_last_scan;
      scan_time_span = previous_scan_time_span;
      scan_distortion_risk = previous_scan_distortion_risk;
      scan_distortion_rot_risk = previous_scan_distortion_rot_risk;
      scan_distortion_trans_risk = previous_scan_distortion_trans_risk;
      motion_degrade = previous_motion_degrade;
      vibration_degrade = previous_vibration_degrade;
      vibration_acc_risk = previous_vibration_acc_risk;
      vibration_gyr_risk = previous_vibration_gyr_risk;
      ROS_WARN_THROTTLE(1.0, "Reject no-IMU propagation: invalid LiDAR frame or propagated state");
      return ImuProcessStatus::Rejected;
    }
    stat = propagated;
    return ImuProcessStatus::Ready;
  }

  if (lidar_meas.measures.empty())
  {
    ROS_WARN_THROTTLE(1.0, "Reject IMU processing: empty MeasureGroup deque");
    return ImuProcessStatus::Rejected;
  }
  MeasureGroup meas = lidar_meas.measures.back();

  if (imu_need_init)
  {
    double pcl_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;
    if (!std::isfinite(pcl_end_time) || meas.imu.empty())
    {
      ROS_WARN_THROTTLE(1.0, "Skip IMU initialization: invalid target time or empty IMU batch");
      return ImuProcessStatus::Rejected;
    }
    const bool first_init_batch = b_first_frame;
    if (!first_init_batch && pcl_end_time < last_prop_end_time - kImuTimeTolerance)
    {
      ROS_WARN_THROTTLE(1.0, "Reject IMU initialization: target time moved backward");
      return ImuProcessStatus::Rejected;
    }
    if (!first_init_batch &&
        pcl_end_time - last_prop_end_time > max_propagation_interval_ + kImuTimeTolerance)
    {
      ROS_WARN_THROTTLE(1.0, "Reject IMU initialization: forward gap exceeds propagation safety window");
      return ImuProcessStatus::Rejected;
    }
    if (!first_init_batch && !imuSampleFinite(last_imu))
    {
      ROS_WARN_THROTTLE(1.0, "Reject IMU initialization: invalid previous IMU sample");
      return ImuProcessStatus::Rejected;
    }
    double previous_imu_time = first_init_batch ? -std::numeric_limits<double>::infinity() :
                                                   last_imu->header.stamp.toSec();
    bool has_new_imu = first_init_batch;
    for (const auto &imu : meas.imu)
    {
      if (!imuSampleFinite(imu))
      {
        ROS_WARN_THROTTLE(1.0, "Reject IMU initialization: null or non-finite sample");
        return ImuProcessStatus::Rejected;
      }
      const double imu_time = imu->header.stamp.toSec();
      if (imu_time < previous_imu_time - kImuTimeTolerance || imu_time > pcl_end_time + kImuTimeTolerance)
      {
        ROS_WARN_THROTTLE(1.0, "Reject IMU initialization: non-monotonic/out-of-window timestamp");
        return ImuProcessStatus::Rejected;
      }
      if (imu_time > last_prop_end_time + kImuTimeTolerance) has_new_imu = true;
      previous_imu_time = imu_time;
    }
    if (!has_new_imu)
    {
      ROS_WARN_THROTTLE(1.0, "Reject IMU initialization: batch does not advance time");
      return ImuProcessStatus::Rejected;
    }
    /// The very first lidar frame
    StatesGroup initialized = stat;
    if (!IMU_init(meas, initialized, init_iter_num) || !stateFinite(initialized))
    {
      ROS_WARN_THROTTLE(1.0, "Reject IMU initialization after invalid samples/state");
      return ImuProcessStatus::Rejected;
    }
    stat = initialized;

    imu_need_init = true;

    last_imu = meas.imu.back();
    last_prop_end_time = pcl_end_time;
    time_last_scan = std::isfinite(lidar_meas.lidar_frame_beg_time) ?
        lidar_meas.lidar_frame_beg_time : pcl_end_time;
    lidar_meas.last_lio_update_time = pcl_end_time;

    if (init_iter_num >= MAX_INI_COUNT)
    {
      // cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init = false;
      ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: "
               "%.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f \n",
               stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm(), cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1],
               cov_gyr[2]);
      ROS_INFO("IMU Initials: ba covarience: %.8f %.8f %.8f; bg covarience: "
               "%.8f %.8f %.8f",
               cov_bias_acc[0], cov_bias_acc[1], cov_bias_acc[2], cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2]);
      if (imu_log_enabled_ && !fout_imu.is_open())
      {
        fout_imu.clear();
        fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
      }
    }

    return ImuProcessStatus::Initializing;
  }

  StatesGroup propagated = stat;
  const double previous_update_time = lidar_meas.last_lio_update_time;
  const sensor_msgs::ImuConstPtr previous_last_imu = last_imu;
  const double previous_last_prop_end_time = last_prop_end_time;
  const bool previous_imu_time_init = imu_time_init;
  const V3D previous_angvel_last = angvel_last;
  const V3D previous_acc_s_last = acc_s_last;
  const std::vector<Pose6D> previous_imu_poses = IMUpose;
  const PointCloudXYZI previous_waiting_cloud = pcl_wait_proc;
  const double previous_motion_degrade = motion_degrade;
  const double previous_vibration_degrade = vibration_degrade;
  const double previous_vibration_acc_risk = vibration_acc_risk;
  const double previous_vibration_gyr_risk = vibration_gyr_risk;
  const V3D previous_acc_vib_lp = acc_vib_lp_;
  const V3D previous_gyr_vib_lp = gyr_vib_lp_;
  const bool previous_vib_lp_valid = vib_lp_valid_;
  const V3D previous_acc_last = acc_last_;
  const bool previous_acc_last_valid = acc_last_valid_;
  const double previous_scan_distortion_risk = scan_distortion_risk;
  const double previous_scan_distortion_rot_risk = scan_distortion_rot_risk;
  const double previous_scan_distortion_trans_risk = scan_distortion_trans_risk;
  const double previous_scan_time_span = scan_time_span;
  auto restoreRuntimeState = [&]() {
    last_imu = previous_last_imu;
    last_prop_end_time = previous_last_prop_end_time;
    imu_time_init = previous_imu_time_init;
    angvel_last = previous_angvel_last;
    acc_s_last = previous_acc_s_last;
    IMUpose = previous_imu_poses;
    pcl_wait_proc = previous_waiting_cloud;
    motion_degrade = previous_motion_degrade;
    vibration_degrade = previous_vibration_degrade;
    vibration_acc_risk = previous_vibration_acc_risk;
    vibration_gyr_risk = previous_vibration_gyr_risk;
    acc_vib_lp_ = previous_acc_vib_lp;
    gyr_vib_lp_ = previous_gyr_vib_lp;
    vib_lp_valid_ = previous_vib_lp_valid;
    acc_last_ = previous_acc_last;
    acc_last_valid_ = previous_acc_last_valid;
    scan_distortion_risk = previous_scan_distortion_risk;
    scan_distortion_rot_risk = previous_scan_distortion_rot_risk;
    scan_distortion_trans_risk = previous_scan_distortion_trans_risk;
    scan_time_span = previous_scan_time_span;
  };
  if (!UndistortPcl(lidar_meas, propagated, *cur_pcl_un_) || !stateFinite(propagated))
  {
    lidar_meas.last_lio_update_time = previous_update_time;
    cur_pcl_un_->clear();
    restoreRuntimeState();
    return ImuProcessStatus::Rejected;
  }
  if (lidar_meas.lio_vio_flg == LIO)
  {
    if (cur_pcl_un_->empty())
    {
      stat = propagated;
      return ImuProcessStatus::PropagatedOnly;
    }
    for (const auto &point : cur_pcl_un_->points)
    {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
          !std::isfinite(point.curvature))
      {
        cur_pcl_un_->clear();
        stat = propagated;
        ROS_WARN_THROTTLE(1.0, "Skip LIO correction/map write: non-finite undistorted point");
        return ImuProcessStatus::PropagatedOnly;
      }
    }
  }
  stat = propagated;
  // cout << "[ IMU ] undistorted point num: " << cur_pcl_un_->size() << endl;
  return ImuProcessStatus::Ready;
}
