/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "common_lib.h"
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

enum LiDARFeature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};
enum Surround
{
  Prev,
  Next
};
enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  LiDARFeature ftype;
  orgtype()
  {
    range = 0;
    dista = 0;
    angle[Prev] = 0;
    angle[Next] = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

/*** Velodyne ***/
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))
/****************/

/*** Ouster ***/
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  uint8_t ring;
  std::uint16_t ambient;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
/****************/

/*** Hesai_XT32 ***/
namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Hesai_Pandar128 ***/
namespace Pandar128_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace Pandar128_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Robosense_Airy ***/
namespace robosense_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);
  // MODIFICATIONS: Innovation 6 — adaptive preprocessing helpers
  void computePostDispatchStats();
  void decideNextFilterNum(double ext_motion_risk, double ext_degeneracy_factor = 1.0);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; // maximum 128 line lidar
  vector<orgtype> typess[128]; // maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;

  double blind, blind_sqr;
  bool feature_enabled, given_offset_time;
  ros::Publisher pub_full, pub_surf, pub_corn;

  // MODIFICATIONS: adaptive LiDAR preprocessing quality statistics (Innovation 6)
  // Updated in both process() overloads after each frame is dispatched to a handler.
  // All read-only from outside (LIVMapper reads them to decide the next filter num).
  int   last_raw_count   = 0;    // raw points entering the handler (before any filtering)
  int   last_valid_count = 0;    // handler-counted points that passed blind / tag filters before downsampling
  int   last_output_count = 0;   // points in pl_surf after downsampling
  double last_valid_ratio  = 1.0; // last_valid_count / last_raw_count
  double last_output_ratio = 1.0; // last_output_count / last_raw_count
  double last_near_ratio   = 0.0; // fraction of output points with range < near_dist_sq
  double last_far_ratio    = 0.0; // fraction of output points with range > far_dist_sq
  double last_intensity_mean = 128.0; // mean reflectivity of output points
  double last_lidar_quality  = 1.0;   // composite score in (0,1] used by LIVMapper

  // adaptive filter parameters — all configurable from yaml via LIVMapper
  bool   adaptive_filter_en = false;
  bool   intensity_quality_en = true;  // include return intensity in adaptive sampling quality
  bool   intensity_available = true;   // false for sensors whose input has no LiDAR return channel
  double intensity_ref = 80.0;         // shared with voxel-map return-quality weighting
  double saturation_ref = 240.0;
  bool   verbose_runtime_log = false;
  int    max_adaptive_filter_num = 6;   // hard upper bound on point_filter_num
  double near_dist_sq  = 25.0;          // points within sqrt(25)=5m are "near" (m²)
  double far_dist_sq   = 900.0;         // points beyond sqrt(900)=30m are "far" (m²)
  double motion_risk   = 0.0;           // 0~1, fed by LIVMapper from ImuProcess::motion_degrade
  int    bad_hysteresis_frames = 2;      // consecutive degraded frames before keeping more points
  int    good_hysteresis_frames = 6;     // consecutive healthy frames before coarsening sampling
  int    bad_filter_count = 0;           // diagnostics/state for adaptive filter hysteresis
  int    good_filter_count = 0;

private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void xt32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void Pandar128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void robosense_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_
