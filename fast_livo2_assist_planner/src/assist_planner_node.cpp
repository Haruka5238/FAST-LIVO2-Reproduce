// fast_livo2_assist_planner
//
// Human-in-the-loop local assist planner for FAST-LIVO2 online mapping.
// This node publishes suggested local viewpoints and sparse advisory paths
// only. It never publishes MAVROS/PX4/DJI control setpoints.

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class VoxelState
{
  UNKNOWN,
  FREE,
  OCCUPIED
};

struct PathEvidence
{
  int samples{0};
  int free_count{0};
  int occupied_count{0};
  int unknown_count{0};
  double unknown_ratio{1.0};
};

struct Candidate
{
  Point3 p;
  double yaw{0.0};
  double move_yaw{0.0};
  double view_yaw{0.0};
  double rel_yaw{0.0};
  double distance{0.0};
  double min_obstacle_distance{std::numeric_limits<double>::infinity()};
  double coverage_gain{0.0};
  double collision_risk{0.0};
  double health_penalty{0.0};
  double unknown_risk{0.0};
  double path_length_cost{0.0};
  double turn_cost{0.0};
  double task_progress{0.0};
  double score{-std::numeric_limits<double>::infinity()};
  PathEvidence path_evidence;
  int sector{-1};
  bool selected{false};
};

bool isFinite(double v)
{
  return std::isfinite(v);
}

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double normAngle(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

double yawFromQuat(const geometry_msgs::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::Quaternion quatFromYaw(double yaw)
{
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double distance2D(const Point3 &a, const Point3 &b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double distance3D(const Point3 &a, const Point3 &b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double pointSegmentDistance3D(const Point3 &p, const Point3 &a, const Point3 &b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double vz = b.z - a.z;
  const double wx = p.x - a.x;
  const double wy = p.y - a.y;
  const double wz = p.z - a.z;
  const double vv = vx * vx + vy * vy + vz * vz;
  if (vv < 1e-9)
    return distance3D(p, a);
  const double t = clamp((wx * vx + wy * vy + wz * vz) / vv, 0.0, 1.0);
  Point3 proj;
  proj.x = a.x + t * vx;
  proj.y = a.y + t * vy;
  proj.z = a.z + t * vz;
  return distance3D(p, proj);
}

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey &k) const
  {
    const std::size_t hx = static_cast<std::size_t>(k.x) * 73856093u;
    const std::size_t hy = static_cast<std::size_t>(k.y) * 19349663u;
    const std::size_t hz = static_cast<std::size_t>(k.z) * 83492791u;
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};

struct VoxelEvidence
{
  int free_hits{0};
  int occupied_hits{0};
  ros::Time last_free;
  ros::Time last_occupied;
};

class LocalEvidenceGrid
{
public:
  void configure(double voxel_size, double local_radius, double local_z_radius,
                 double free_ttl, double occupied_ttl, double max_ray_length,
                 double ray_step, int free_ray_stride, int min_occupied_hits,
                 int max_voxels, bool free_raycast_enabled)
  {
    voxel_size_ = std::max(0.1, voxel_size);
    inv_voxel_size_ = 1.0 / voxel_size_;
    local_radius_ = std::max(1.0, local_radius);
    local_z_radius_ = std::max(0.5, local_z_radius);
    free_ttl_ = std::max(0.1, free_ttl);
    occupied_ttl_ = std::max(0.1, occupied_ttl);
    max_ray_length_ = std::max(1.0, max_ray_length);
    ray_step_ = std::max(0.1, ray_step);
    free_ray_stride_ = std::max(1, free_ray_stride);
    min_occupied_hits_ = std::max(1, min_occupied_hits);
    max_voxels_ = std::max(1000, max_voxels);
    free_raycast_enabled_ = free_raycast_enabled;
  }

  void update(const Point3 &origin, const std::vector<Point3> &points, const ros::Time &now)
  {
    if (points.empty())
      return;
    markFree(origin, now);
    int index = 0;
    for (const auto &p : points)
    {
      if (!insideLocal(p, origin))
        continue;
      // Ray-cast FREE evidence along each beam only when enabled.  Set
      // free_raycast_enabled=false when /cloud_registered is an accumulated
      // map: ray-casting from the current pose toward historical points would
      // incorrectly label regions as free that were never actually observed from
      // this viewpoint.  OCCUPIED evidence is always marked regardless.
      if (free_raycast_enabled_ && index % free_ray_stride_ == 0)
        markRayFree(origin, p, now);
      markOccupied(p, now);
      ++index;
    }
    prune(origin, now);
  }

  VoxelState stateAt(const Point3 &p, const ros::Time &now) const
  {
    const auto it = voxels_.find(keyFor(p));
    if (it == voxels_.end())
      return VoxelState::UNKNOWN;
    return stateOf(it->second, now);
  }

  PathEvidence pathEvidence(const Point3 &start, const Point3 &goal, double step,
                            const ros::Time &now) const
  {
    PathEvidence out;
    const double dist = distance3D(start, goal);
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / std::max(step, voxel_size_))));
    for (int i = 0; i <= steps; ++i)
    {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      Point3 p;
      p.x = start.x + t * (goal.x - start.x);
      p.y = start.y + t * (goal.y - start.y);
      p.z = start.z + t * (goal.z - start.z);
      ++out.samples;
      const VoxelState s = stateAt(p, now);
      if (s == VoxelState::OCCUPIED)
        ++out.occupied_count;
      else if (s == VoxelState::FREE)
        ++out.free_count;
      else
        ++out.unknown_count;
    }
    out.unknown_ratio = out.samples > 0 ? static_cast<double>(out.unknown_count) / out.samples : 1.0;
    return out;
  }

  double minDistanceToOccupiedPoint(const Point3 &p, const ros::Time &now) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (const auto &entry : voxels_)
    {
      if (stateOf(entry.second, now) != VoxelState::OCCUPIED)
        continue;
      const Point3 c = centerOf(entry.first);
      // AABB pruning: if any single axis already exceeds best, the 3-D distance
      // cannot be better — skip the sqrt entirely.
      if (std::fabs(p.x - c.x) >= best || std::fabs(p.y - c.y) >= best ||
          std::fabs(p.z - c.z) >= best)
        continue;
      best = std::min(best, distance3D(p, c));
    }
    return best;
  }

  double minDistanceToOccupiedSegment(const Point3 &a, const Point3 &b, const ros::Time &now) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (const auto &entry : voxels_)
    {
      if (stateOf(entry.second, now) != VoxelState::OCCUPIED)
        continue;
      const Point3 c = centerOf(entry.first);
      // Dynamic AABB: skip voxels whose centre falls outside a box expanded
      // by the current best distance around the segment's own bounding box.
      // This avoids the costly pointSegmentDistance3D + sqrt for most voxels.
      if (c.x < std::min(a.x, b.x) - best || c.x > std::max(a.x, b.x) + best ||
          c.y < std::min(a.y, b.y) - best || c.y > std::max(a.y, b.y) + best ||
          c.z < std::min(a.z, b.z) - best || c.z > std::max(a.z, b.z) + best)
        continue;
      best = std::min(best, pointSegmentDistance3D(c, a, b));
    }
    return best;
  }

  bool nearestOccupied(const Point3 &p, double z_radius, const ros::Time &now, Point3 *nearest) const
  {
    double best_sq = std::numeric_limits<double>::infinity();
    bool found = false;
    for (const auto &entry : voxels_)
    {
      if (stateOf(entry.second, now) != VoxelState::OCCUPIED)
        continue;
      const Point3 c = centerOf(entry.first);
      if (std::fabs(c.z - p.z) > z_radius)
        continue;
      const double dx = c.x - p.x;
      const double dy = c.y - p.y;
      const double d2 = dx * dx + dy * dy;
      // Use 1e-6 instead of 0.04 (0.2 m).  The 0.2 m threshold was a blind zone:
      // when the UAV is closer than 0.2 m to a wall the nearest obstacle was
      // silently skipped, causing warningRetreatBaseYaw to point toward the
      // structure rather than away from it.  Only skip the voxel that is
      // exactly at the query point (floating-point zero guard).
      if (d2 < 1e-6 || d2 >= best_sq)
        continue;
      best_sq = d2;
      if (nearest)
        *nearest = c;
      found = true;
    }
    return found;
  }

  double surfaceFrontierGain(const Point3 &viewpoint, double view_yaw, const ros::Time &now,
                             double min_standoff, double ideal_standoff, double max_standoff,
                             double view_fov_rad, int frontier_target_voxels) const
  {
    if (voxels_.empty())
      return 0.0;
    const double half_fov = std::max(0.1, view_fov_rad * 0.5);
    const double target = std::max(1, frontier_target_voxels);
    double gain = 0.0;
    for (const auto &entry : voxels_)
    {
      if (stateOf(entry.second, now) != VoxelState::OCCUPIED)
        continue;
      const Point3 surface = centerOf(entry.first);
      // AABB pre-filter: skip voxels clearly outside the standoff sphere on any
      // axis before computing the more expensive 3-D distance and atan2.
      if (std::fabs(viewpoint.x - surface.x) > max_standoff ||
          std::fabs(viewpoint.y - surface.y) > max_standoff ||
          std::fabs(viewpoint.z - surface.z) > max_standoff)
        continue;
      const double dist = distance3D(viewpoint, surface);
      if (dist < min_standoff || dist > max_standoff)
        continue;
      const double bearing = std::atan2(surface.y - viewpoint.y, surface.x - viewpoint.x);
      const double angle = std::fabs(normAngle(bearing - view_yaw));
      if (angle > half_fov)
        continue;
      const int unknown_neighbors = unknownNeighborCount(entry.first, now);
      if (unknown_neighbors <= 0)
        continue;
      const double standoff_span = dist < ideal_standoff ?
          std::max(ideal_standoff - min_standoff, 1e-3) :
          std::max(max_standoff - ideal_standoff, 1e-3);
      const double standoff_score = 1.0 - clamp(std::fabs(dist - ideal_standoff) / standoff_span, 0.0, 1.0);
      const double angle_score = 1.0 - clamp(angle / half_fov, 0.0, 1.0);
      const double frontier_score = clamp(static_cast<double>(unknown_neighbors) / 6.0, 0.0, 1.0);
      gain += (0.30 + 0.70 * frontier_score) * (0.45 + 0.55 * standoff_score) *
              (0.45 + 0.55 * angle_score);
    }
    return clamp(gain / target, 0.0, 1.0);
  }

  std::size_t voxelCount() const { return voxels_.size(); }

  int countState(VoxelState state, const ros::Time &now) const
  {
    int count = 0;
    for (const auto &entry : voxels_)
    {
      if (stateOf(entry.second, now) == state)
        ++count;
    }
    return count;
  }

  void clear() { voxels_.clear(); }

private:
  VoxelKey keyFor(const Point3 &p) const
  {
    VoxelKey k;
    k.x = static_cast<int>(std::floor(p.x * inv_voxel_size_));
    k.y = static_cast<int>(std::floor(p.y * inv_voxel_size_));
    k.z = static_cast<int>(std::floor(p.z * inv_voxel_size_));
    return k;
  }

  Point3 centerOf(const VoxelKey &k) const
  {
    Point3 p;
    p.x = (static_cast<double>(k.x) + 0.5) * voxel_size_;
    p.y = (static_cast<double>(k.y) + 0.5) * voxel_size_;
    p.z = (static_cast<double>(k.z) + 0.5) * voxel_size_;
    return p;
  }

  bool insideLocal(const Point3 &p, const Point3 &center) const
  {
    // Avoid sqrt: compare squared 2-D distance against squared radius.
    const double dx = p.x - center.x;
    const double dy = p.y - center.y;
    return (dx * dx + dy * dy) <= local_radius_ * local_radius_ &&
           std::fabs(p.z - center.z) <= local_z_radius_;
  }

  VoxelState stateOf(const VoxelEvidence &v, const ros::Time &now) const
  {
    if (!v.last_occupied.isZero() && v.occupied_hits >= min_occupied_hits_)
    {
      const double age = (now - v.last_occupied).toSec();
      if (age >= 0.0 && age <= occupied_ttl_)
        return VoxelState::OCCUPIED;
    }
    if (!v.last_free.isZero())
    {
      const double age = (now - v.last_free).toSec();
      if (age >= 0.0 && age <= free_ttl_)
        return VoxelState::FREE;
    }
    return VoxelState::UNKNOWN;
  }

  void markFree(const Point3 &p, const ros::Time &now)
  {
    VoxelEvidence &v = voxels_[keyFor(p)];
    v.free_hits += 1;
    v.last_free = now;
  }

  void markOccupied(const Point3 &p, const ros::Time &now)
  {
    VoxelEvidence &v = voxels_[keyFor(p)];
    v.occupied_hits += 1;
    v.last_occupied = now;
  }

  void markRayFree(const Point3 &origin, const Point3 &hit, const ros::Time &now)
  {
    const double full_dist = distance3D(origin, hit);
    const double ray_dist = std::min(full_dist, max_ray_length_);
    if (ray_dist < voxel_size_ || full_dist < 1e-6)
      return;
    const int steps = std::max(1, static_cast<int>(std::floor(ray_dist / ray_step_)));
    for (int i = 1; i < steps; ++i)
    {
      const double along = static_cast<double>(i) * ray_step_;
      if (along > ray_dist * 0.92)
        break;
      const double t = along / full_dist;
      Point3 p;
      p.x = origin.x + t * (hit.x - origin.x);
      p.y = origin.y + t * (hit.y - origin.y);
      p.z = origin.z + t * (hit.z - origin.z);
      markFree(p, now);
    }
  }

  int unknownNeighborCount(const VoxelKey &k, const ros::Time &now) const
  {
    static const int offsets[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    int count = 0;
    for (const auto &o : offsets)
    {
      VoxelKey n;
      n.x = k.x + o[0];
      n.y = k.y + o[1];
      n.z = k.z + o[2];
      const auto it = voxels_.find(n);
      if (it == voxels_.end() || stateOf(it->second, now) == VoxelState::UNKNOWN)
        ++count;
    }
    return count;
  }

  void prune(const Point3 &center, const ros::Time &now)
  {
    for (auto it = voxels_.begin(); it != voxels_.end(); )
    {
      const Point3 p = centerOf(it->first);
      const double last_age_free = it->second.last_free.isZero() ? std::numeric_limits<double>::infinity() :
          (now - it->second.last_free).toSec();
      const double last_age_occ = it->second.last_occupied.isZero() ? std::numeric_limits<double>::infinity() :
          (now - it->second.last_occupied).toSec();
      const bool stale = last_age_free > free_ttl_ && last_age_occ > occupied_ttl_;
      if (!insideLocal(p, center) || stale)
        it = voxels_.erase(it);
      else
        ++it;
    }

    // Batch overflow eviction: one O(N) pass with nth_element instead of
    // O(K * N) repeated single-voxel scans.
    const int overflow = static_cast<int>(voxels_.size()) - max_voxels_;
    if (overflow > 0)
    {
      // Evict a little more than the strict overflow to reduce how often
      // this triggers on subsequent frames.
      const int erase_count = std::min(
          overflow + std::max(1, max_voxels_ / 20),
          static_cast<int>(voxels_.size()));

      std::vector<std::pair<double, VoxelKey>> scored;
      scored.reserve(voxels_.size());
      for (const auto &entry : voxels_)
      {
        const Point3 p = centerOf(entry.first);
        const double dx = p.x - center.x;
        const double dy = p.y - center.y;
        // Use squared ratio to avoid sqrt; distance weight is 70%.
        const double dist_sq_ratio = (dx * dx + dy * dy) /
                                     std::max(local_radius_ * local_radius_, 1e-6);
        const double free_age = entry.second.last_free.isZero() ? occupied_ttl_ :
            std::max(0.0, (now - entry.second.last_free).toSec());
        const double occ_age = entry.second.last_occupied.isZero() ? occupied_ttl_ :
            std::max(0.0, (now - entry.second.last_occupied).toSec());
        const double age_score = std::min(free_age, occ_age) /
                                 std::max(occupied_ttl_, free_ttl_);
        scored.push_back({0.70 * dist_sq_ratio + 0.30 * age_score, entry.first});
      }
      // Partial sort: bring the erase_count largest-score voxels to the front.
      std::nth_element(scored.begin(), scored.begin() + erase_count, scored.end(),
                       [](const std::pair<double, VoxelKey> &a,
                          const std::pair<double, VoxelKey> &b) {
                         return a.first > b.first;
                       });
      for (int i = 0; i < erase_count; ++i)
        voxels_.erase(scored[static_cast<std::size_t>(i)].second);
    }
  }

  double voxel_size_{0.5};
  double inv_voxel_size_{2.0};
  double local_radius_{15.0};
  double local_z_radius_{5.0};
  double free_ttl_{4.0};
  double occupied_ttl_{20.0};
  double max_ray_length_{12.0};
  double ray_step_{0.5};
  int free_ray_stride_{3};
  int min_occupied_hits_{1};
  int max_voxels_{40000};
  bool free_raycast_enabled_{true};
  std::unordered_map<VoxelKey, VoxelEvidence, VoxelKeyHash> voxels_;
};

std::map<std::string, std::string> parseKv(const std::string &text)
{
  std::map<std::string, std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ';'))
  {
    const std::size_t pos = item.find('=');
    if (pos == std::string::npos)
      continue;
    std::string key = item.substr(0, pos);
    std::string value = item.substr(pos + 1);
    key.erase(0, key.find_first_not_of(" \t\r\n"));
    key.erase(key.find_last_not_of(" \t\r\n") + 1);
    value.erase(0, value.find_first_not_of(" \t\r\n"));
    value.erase(value.find_last_not_of(" \t\r\n") + 1);
    out[key] = value;
  }
  return out;
}

std::string trim(std::string value)
{
  const std::size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return "";
  const std::size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
  return value;
}
}  // namespace

class AssistPlanner
{
public:
  AssistPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    pnh.param<std::string>("topics/odom", odom_topic_, "/aft_mapped_to_init");
    pnh.param<std::string>("topics/cloud", cloud_topic_, "/cloud_registered");
    pnh.param<std::string>("topics/health_advice", health_advice_topic_, "/fast_livo2_health/advice");
    pnh.param<std::string>("topics/pilot_intent", pilot_intent_topic_, "/fast_livo2_assist/pilot_intent");
    pnh.param<std::string>("frame/world_frame", world_frame_, "camera_init");

    pnh.param<double>("runtime/odom_timeout_s", odom_timeout_s_, 1.0);
    pnh.param<double>("runtime/health_timeout_s", health_timeout_s_, 3.0);

    pnh.param<double>("map/local_radius_m", local_radius_m_, 15.0);
    pnh.param<double>("map/local_z_radius_m", local_z_radius_m_, 5.0);
    pnh.param<double>("map/cloud_timeout_s", cloud_timeout_s_, 1.0);
    pnh.param<bool>("map/require_fresh_cloud", require_fresh_cloud_, true);
    pnh.param<bool>("map/suppress_on_frame_mismatch", suppress_on_frame_mismatch_, true);
    pnh.param<int>("map/max_cloud_points", max_cloud_points_, 6000);
    pnh.param<int>("map/sector_count", sector_count_, 16);
    pnh.param<int>("map/target_sector_points", target_sector_points_, 180);
    pnh.param<int>("map/min_sector_points_for_strong_gain", min_sector_points_for_strong_gain_, 20);
    pnh.param<double>("map/update_rate_hz", update_rate_hz_, 2.0);

    pnh.param<bool>("evidence_grid/enabled", evidence_grid_enabled_, true);
    pnh.param<bool>("evidence_grid/free_raycast_enabled", grid_free_raycast_enabled_, true);
    pnh.param<double>("evidence_grid/voxel_size_m", grid_voxel_size_m_, 0.5);
    pnh.param<double>("evidence_grid/free_ttl_s", grid_free_ttl_s_, 4.0);
    pnh.param<double>("evidence_grid/occupied_ttl_s", grid_occupied_ttl_s_, 20.0);
    pnh.param<double>("evidence_grid/max_ray_length_m", grid_max_ray_length_m_, 12.0);
    pnh.param<double>("evidence_grid/ray_step_m", grid_ray_step_m_, 0.5);
    pnh.param<int>("evidence_grid/free_ray_stride", grid_free_ray_stride_, 3);
    pnh.param<int>("evidence_grid/min_occupied_hits", grid_min_occupied_hits_, 1);
    pnh.param<int>("evidence_grid/max_voxels", grid_max_voxels_, 40000);

    pnh.param<double>("candidate/sample_radius_m", sample_radius_m_, 3.0);
    pnh.param<int>("candidate/sample_count", sample_count_, 16);
    pnh.param<double>("candidate/min_goal_distance_m", min_goal_distance_m_, 1.0);
    pnh.param<double>("candidate/max_goal_distance_m", max_goal_distance_m_, 5.0);
    pnh.param<double>("candidate/min_obstacle_distance_m", min_obstacle_distance_m_, 1.2);
    pnh.param<double>("candidate/max_yaw_change_rad", max_yaw_change_rad_, 1.2);
    pnh.param<double>("candidate/path_check_step_m", path_check_step_m_, 0.35);
    if (!pnh.getParam("candidate/z_offsets_m", z_offsets_m_))
    {
      z_offsets_m_.clear();
      z_offsets_m_.push_back(0.0);
      z_offsets_m_.push_back(0.75);
      z_offsets_m_.push_back(-0.75);
    }

    pnh.param<double>("health_policy/watch_sample_radius_scale", watch_sample_radius_scale_, 0.7);
    pnh.param<double>("health_policy/watch_obstacle_distance_scale", watch_obstacle_distance_scale_, 1.3);
    pnh.param<double>("health_policy/warning_retreat_distance_m", warning_retreat_distance_m_, 2.0);
    pnh.param<double>("health_policy/recovering_max_goal_distance_m", recovering_max_goal_distance_m_, 1.5);
    pnh.param<bool>("health_policy/critical_publish_hold_goal", critical_publish_hold_goal_, true);

    pnh.param<bool>("confidence_policy/very_low_suppresses", confidence_very_low_suppresses_, true);
    pnh.param<bool>("confidence_policy/low_forces_conservative", confidence_low_forces_conservative_, true);

    pnh.param<std::string>("intent/default_intent", default_pilot_intent_, "FREE");
    pnh.param<double>("intent/message_default_duration_s", pilot_intent_message_default_duration_s_, 8.0);
    pnh.param<double>("intent/corridor_deg", intent_corridor_deg_, 60.0);
    pnh.param<double>("intent/orbit_corridor_deg", orbit_intent_corridor_deg_, 70.0);
    pnh.param<double>("intent/vertical_corridor_deg", vertical_intent_corridor_deg_, 90.0);
    pnh.param<bool>("intent/free_mode_publishes_suggestions", free_mode_publishes_suggestions_, false);

    pnh.param<double>("scoring/w_task_progress", w_task_progress_, 1.2);
    pnh.param<double>("scoring/w_coverage", w_coverage_, 1.0);
    pnh.param<double>("scoring/w_collision", w_collision_, 1.5);
    pnh.param<double>("scoring/w_health", w_health_, 2.0);
    pnh.param<double>("scoring/w_unknown", w_unknown_, 2.0);
    pnh.param<double>("scoring/w_path_length", w_path_length_, 0.3);
    pnh.param<double>("scoring/w_turn", w_turn_, 0.4);
    pnh.param<double>("scoring/repeat_sector_penalty", repeat_sector_penalty_, 0.1);
    pnh.param<double>("scoring/repeat_sector_penalty_cap", repeat_sector_penalty_cap_, 0.9);
    // Per-mode minimum accept scores.  The legacy single min_accept_score is
    // kept as a global fallback; per-mode values override it when set.
    pnh.param<double>("scoring/min_accept_score", min_accept_score_, -0.25);
    pnh.param<double>("scoring/min_accept_score_explore",      min_accept_score_explore_,      0.05);
    pnh.param<double>("scoring/min_accept_score_conservative", min_accept_score_conservative_, 0.00);
    pnh.param<double>("scoring/min_accept_score_recover",      min_accept_score_recover_,     -0.05);
    pnh.param<double>("scoring/min_accept_score_retreat",      min_accept_score_retreat_,     -0.10);

    pnh.param<double>("safety/max_unknown_path_ratio", max_unknown_path_ratio_, 0.30);
    pnh.param<double>("safety/warning_max_unknown_path_ratio", warning_max_unknown_path_ratio_, 0.18);
    pnh.param<double>("safety/recovering_max_unknown_path_ratio", recovering_max_unknown_path_ratio_, 0.22);

    pnh.param<double>("coverage/min_standoff_m", min_standoff_m_, 1.5);
    pnh.param<double>("coverage/ideal_standoff_m", ideal_standoff_m_, 3.0);
    pnh.param<double>("coverage/max_standoff_m", max_standoff_m_, 6.0);
    pnh.param<double>("coverage/view_fov_rad", coverage_view_fov_rad_, 1.5708);
    pnh.param<int>("coverage/frontier_target_voxels", frontier_target_voxels_, 30);
    pnh.param<double>("coverage/surface_exhausted_density_fallback_scale",
                      surface_exhausted_density_fallback_scale_, 0.35);

    pnh.param<double>("ui/marker_scale", marker_scale_, 0.35);
    pnh.param<bool>("ui/show_all_candidates", show_all_candidates_, true);
    pnh.param<double>("ui/status_print_period", status_print_period_, 3.0);

    pnh.param<bool>("stability/enable_goal_commitment", enable_goal_commitment_, true);
    pnh.param<double>("stability/min_goal_hold_s", min_goal_hold_s_, 2.5);
    pnh.param<double>("stability/max_goal_hold_s", max_goal_hold_s_, 6.0);
    pnh.param<double>("stability/switch_score_margin", switch_score_margin_, 0.20);
    pnh.param<double>("stability/switch_bearing_margin_deg", switch_bearing_margin_deg_, 18.0);

    pnh.param<bool>("logging/enable_csv", csv_enabled_, true);
    pnh.param<std::string>("logging/csv_path", csv_path_, "/tmp/fast_livo2_assist.csv");

    validateParams();
    evidence_grid_.configure(grid_voxel_size_m_, local_radius_m_, local_z_radius_m_,
                             grid_free_ttl_s_, grid_occupied_ttl_s_, grid_max_ray_length_m_,
                             grid_ray_step_m_, grid_free_ray_stride_, grid_min_occupied_hits_,
                             grid_max_voxels_, grid_free_raycast_enabled_);
    sector_visits_.assign(sector_count_, 0);
    openCsv();

    sub_odom_ = nh.subscribe(odom_topic_, 50, &AssistPlanner::odomCb, this);
    sub_cloud_ = nh.subscribe(cloud_topic_, 5, &AssistPlanner::cloudCb, this);
    sub_health_ = nh.subscribe(health_advice_topic_, 20, &AssistPlanner::healthCb, this);
    sub_intent_ = nh.subscribe(pilot_intent_topic_, 10, &AssistPlanner::pilotIntentCb, this);

    pub_path_ = nh.advertise<nav_msgs::Path>("/fast_livo2_assist/recommended_path", 10);
    pub_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/fast_livo2_assist/recommended_goal", 10);
    pub_candidates_ = nh.advertise<geometry_msgs::PoseArray>("/fast_livo2_assist/candidate_viewpoints", 10);
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/fast_livo2_assist/marker_array", 10);
    pub_status_ = nh.advertise<std_msgs::String>("/fast_livo2_assist/status", 10);

    timer_ = nh.createTimer(ros::Duration(1.0 / update_rate_hz_), &AssistPlanner::evaluate, this);

    ROS_INFO("[assist_planner] started. odom=%s cloud=%s health=%s intent=%s",
             odom_topic_.c_str(), cloud_topic_.c_str(), health_advice_topic_.c_str(),
             pilot_intent_topic_.c_str());
    ROS_WARN("[assist_planner] advisory mode only: no MAVROS/PX4/DJI control topics are published.");
  }

  ~AssistPlanner()
  {
    if (csv_.is_open())
      csv_.close();
  }

private:
  void validateParams()
  {
    if (update_rate_hz_ <= 0.0) update_rate_hz_ = 2.0;
    if (sector_count_ < 4) sector_count_ = 4;
    if (sample_count_ < 4) sample_count_ = 4;
    if (max_cloud_points_ < 100) max_cloud_points_ = 100;
    local_radius_m_ = std::max(1.0, local_radius_m_);
    sample_radius_m_ = std::max(0.2, sample_radius_m_);
    min_goal_distance_m_ = std::max(0.0, min_goal_distance_m_);
    max_goal_distance_m_ = std::max(min_goal_distance_m_ + 0.1, max_goal_distance_m_);
    min_obstacle_distance_m_ = std::max(0.1, min_obstacle_distance_m_);
    path_check_step_m_ = std::max(0.05, path_check_step_m_);
    odom_timeout_s_ = std::max(0.1, odom_timeout_s_);
    health_timeout_s_ = std::max(0.1, health_timeout_s_);
    grid_voxel_size_m_ = std::max(0.1, grid_voxel_size_m_);
    grid_free_ttl_s_ = std::max(0.1, grid_free_ttl_s_);
    grid_occupied_ttl_s_ = std::max(0.1, grid_occupied_ttl_s_);
    grid_max_ray_length_m_ = std::max(1.0, grid_max_ray_length_m_);
    grid_ray_step_m_ = std::max(0.1, grid_ray_step_m_);
    grid_free_ray_stride_ = std::max(1, grid_free_ray_stride_);
    grid_min_occupied_hits_ = std::max(1, grid_min_occupied_hits_);
    grid_max_voxels_ = std::max(1000, grid_max_voxels_);
    default_pilot_intent_ = canonicalIntent(default_pilot_intent_);
    pilot_intent_ = default_pilot_intent_;
    pilot_intent_message_default_duration_s_ = std::max(0.0, pilot_intent_message_default_duration_s_);
    intent_corridor_deg_ = clamp(intent_corridor_deg_, 5.0, 180.0);
    orbit_intent_corridor_deg_ = clamp(orbit_intent_corridor_deg_, 5.0, 180.0);
    vertical_intent_corridor_deg_ = clamp(vertical_intent_corridor_deg_, 5.0, 180.0);
    w_task_progress_ = std::max(0.0, w_task_progress_);
    repeat_sector_penalty_ = std::max(0.0, repeat_sector_penalty_);
    repeat_sector_penalty_cap_ = clamp(repeat_sector_penalty_cap_, 0.0, 5.0);
    max_unknown_path_ratio_ = clamp(max_unknown_path_ratio_, 0.0, 1.0);
    warning_max_unknown_path_ratio_ = clamp(warning_max_unknown_path_ratio_, 0.0, 1.0);
    recovering_max_unknown_path_ratio_ = clamp(recovering_max_unknown_path_ratio_, 0.0, 1.0);
    min_standoff_m_ = std::max(0.1, min_standoff_m_);
    ideal_standoff_m_ = std::max(min_standoff_m_, ideal_standoff_m_);
    max_standoff_m_ = std::max(ideal_standoff_m_ + 0.1, max_standoff_m_);
    coverage_view_fov_rad_ = clamp(coverage_view_fov_rad_, 0.2, 2.0 * kPi);
    frontier_target_voxels_ = std::max(1, frontier_target_voxels_);
    surface_exhausted_density_fallback_scale_ = clamp(surface_exhausted_density_fallback_scale_, 0.0, 1.0);
    min_goal_hold_s_ = std::max(0.0, min_goal_hold_s_);
    max_goal_hold_s_ = std::max(min_goal_hold_s_ + 0.1, max_goal_hold_s_);
    switch_score_margin_ = std::max(0.0, switch_score_margin_);
    switch_bearing_margin_deg_ = clamp(switch_bearing_margin_deg_, 0.0, 180.0);
    if (z_offsets_m_.empty())
      z_offsets_m_.push_back(0.0);
    bool has_zero_offset = false;
    for (double &z : z_offsets_m_)
    {
      if (!isFinite(z))
        z = 0.0;
      z = clamp(z, -local_z_radius_m_, local_z_radius_m_);
      if (std::fabs(z) < 1e-6)
        has_zero_offset = true;
    }
    if (!has_zero_offset)
      z_offsets_m_.insert(z_offsets_m_.begin(), 0.0);
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.z = msg->pose.pose.position.z;
    current_yaw_ = yawFromQuat(msg->pose.pose.orientation);
    have_odom_ = isFinite(current_pos_.x) && isFinite(current_pos_.y) && isFinite(current_pos_.z) && isFinite(current_yaw_);
    last_odom_time_ = ros::Time::now();
  }

  void healthCb(const std_msgs::String::ConstPtr &msg)
  {
    const auto kv = parseKv(msg->data);
    auto get = [&](const std::string &key, const std::string &fallback) {
      const auto it = kv.find(key);
      return it == kv.end() ? fallback : it->second;
    };
    health_state_ = get("state", health_state_);
    health_action_code_ = get("action_code", health_action_code_);
    health_primary_reason_ = get("primary_reason", "");
    motion_flags_ = get("motion_flags", motion_flags_);
    confidence_level_ = get("confidence_level", confidence_level_);
    const std::string score = get("reliability_score", "");
    if (!score.empty())
    {
      try
      {
        reliability_score_ = std::stod(score);
      }
      catch (...)
      {
        reliability_score_ = -1.0;
      }
    }
    const std::string confidence = get("confidence_score", "");
    if (!confidence.empty())
    {
      try
      {
        confidence_score_ = std::stod(confidence);
      }
      catch (...)
      {
        confidence_score_ = -1.0;
      }
    }
    have_health_ = true;
    last_health_time_ = ros::Time::now();
  }

  void pilotIntentCb(const std_msgs::String::ConstPtr &msg)
  {
    const auto kv = parseKv(msg->data);
    std::string raw_intent;
    const auto it = kv.find("intent");
    if (it != kv.end())
      raw_intent = it->second;
    else
      raw_intent = msg->data;

    pilot_intent_ = canonicalIntent(raw_intent);
    pilot_intent_strength_ = parseDouble(kv, "strength", 1.0);
    pilot_intent_strength_ = clamp(pilot_intent_strength_, 0.0, 1.0);
    pilot_intent_duration_s_ = parseDouble(kv, "duration_s", pilot_intent_message_default_duration_s_);
    pilot_intent_duration_s_ = std::max(0.0, pilot_intent_duration_s_);

    const double yaw_deg = parseDouble(kv, "yaw_deg", std::numeric_limits<double>::quiet_NaN());
    if (isFinite(yaw_deg))
    {
      latched_intent_yaw_ = normAngle(yaw_deg * kPi / 180.0);
    }
    else if (odomFresh(ros::Time::now()))
    {
      // Use current yaw only if odom is fresh — have_odom_ alone means "ever
      // received", not "still valid".  A stale yaw would lock FORWARD_SCAN to
      // an outdated heading that persists until the intent expires.
      latched_intent_yaw_ = current_yaw_;
    }
    else
    {
      // Without odom we have no valid yaw to latch.  Accepting the intent with
      // yaw = 0 would send the planner towards camera_init x-axis regardless of
      // where the UAV is actually pointing — safer to discard and re-send once
      // FAST-LIVO2 is publishing odometry.
      ROS_WARN("[assist_planner] intent '%s' ignored: odom not yet available "
               "and no explicit yaw_deg in message; re-send once odometry is live",
               pilot_intent_.c_str());
      return;
    }

    last_intent_time_ = ros::Time::now();
    have_intent_msg_ = true;
    have_committed_goal_ = false;

    ROS_INFO("[assist_planner] pilot_intent=%s strength=%.2f duration=%.1fs yaw=%.1fdeg",
             pilot_intent_.c_str(), pilot_intent_strength_, pilot_intent_duration_s_,
             latched_intent_yaw_ * 180.0 / kPi);
  }

  double parseDouble(const std::map<std::string, std::string> &kv,
                     const std::string &key, double fallback) const
  {
    const auto it = kv.find(key);
    if (it == kv.end() || it->second.empty())
      return fallback;
    try
    {
      return std::stod(it->second);
    }
    catch (...)
    {
      return fallback;
    }
  }

  std::string canonicalIntent(const std::string &raw) const
  {
    const std::string value = upper(trim(raw));
    if (value.empty() || value == "FREE" || value == "NONE" || value == "LOCAL_ONLY")
      return "FREE";
    if (value == "FORWARD" || value == "FORWARD_SCAN" || value == "FWD")
      return "FORWARD_SCAN";
    if (value == "LEFT" || value == "LEFT_ORBIT" || value == "LEFT_SWEEP")
      return "LEFT_ORBIT";
    if (value == "RIGHT" || value == "RIGHT_ORBIT" || value == "RIGHT_SWEEP")
      return "RIGHT_ORBIT";
    if (value == "UP" || value == "ASCEND" || value == "UP_SCAN")
      return "UP_SCAN";
    if (value == "DOWN" || value == "DESCEND" || value == "DOWN_SCAN")
      return "DOWN_SCAN";
    if (value == "PULL_BACK" || value == "RETREAT" || value == "BACK_OFF")
      return "PULL_BACK";
    if (value == "HOLD" || value == "HOLD_STABLE" || value == "STOP")
      return "HOLD_STABLE";
    if (value == "RESCAN" || value == "RESCAN_LOCAL")
      return "RESCAN_LOCAL";
    ROS_WARN_THROTTLE(2.0, "[assist_planner] unknown pilot intent '%s', using FREE", raw.c_str());
    return "FREE";
  }

  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    last_cloud_frame_ = msg->header.frame_id;
    if (!msg->header.frame_id.empty() && normalizeFrame(msg->header.frame_id) != normalizeFrame(world_frame_))
    {
      ROS_WARN_THROTTLE(2.0,
                        "[assist_planner] cloud frame_id=%s differs from world_frame=%s. "
                        "Suppressing assist planning until the frame contract is fixed.",
                        msg->header.frame_id.c_str(), world_frame_.c_str());
      cloud_frame_ok_ = false;
      local_cloud_.clear();
      have_cloud_ = true;
      last_cloud_time_ = ros::Time::now();
      return;
    }
    cloud_frame_ok_ = true;

    std::vector<Point3> parsed;
    parsed.reserve(static_cast<std::size_t>(std::min<int>(max_cloud_points_, msg->width * std::max<uint32_t>(msg->height, 1))));
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      const int total_points = static_cast<int>(msg->width * std::max<uint32_t>(msg->height, 1));
      const int stride = std::max(1, total_points / std::max(1, max_cloud_points_));
      int index = 0;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++index)
      {
        if (index % stride != 0)
          continue;
        const double x = *iter_x;
        const double y = *iter_y;
        const double z = *iter_z;
        if (!isFinite(x) || !isFinite(y) || !isFinite(z))
          continue;
        Point3 p{x, y, z};
        if (have_odom_)
        {
          if (distance2D(p, current_pos_) > local_radius_m_)
            continue;
          if (std::fabs(p.z - current_pos_.z) > local_z_radius_m_)
            continue;
        }
        parsed.push_back(p);
        if (static_cast<int>(parsed.size()) >= max_cloud_points_)
          break;
      }
    }
    catch (const std::exception &e)
    {
      ROS_WARN_THROTTLE(2.0, "[assist_planner] cannot parse cloud xyz fields: %s", e.what());
      return;
    }

    local_cloud_.swap(parsed);
    have_cloud_ = true;
    last_cloud_time_ = ros::Time::now();
    if (evidence_grid_enabled_ && have_odom_)
      evidence_grid_.update(current_pos_, local_cloud_, last_cloud_time_);
  }

  void evaluate(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();

    // ROS time reset detection — common when a rosbag is restarted with
    // /use_sim_time.  Clear all stateful evidence so stale TTL comparisons and
    // committed goals from the previous run cannot bleed into the new run.
    if (!last_eval_time_.isZero() && now < last_eval_time_)
    {
      evidence_grid_.clear();
      sector_visits_.assign(sector_count_, 0);
      have_committed_goal_ = false;
      have_intent_msg_ = false;
      last_eval_time_ = now;
      publishSuppressed(now, "WAITING", "TIME_RESET",
                        "ROS时间回退，已清理辅助规划状态");
      return;
    }
    last_eval_time_ = now;
    decaySectorVisits(now);

    if (!have_odom_)
    {
      publishSuppressed(now, "WAITING", "WAITING_FOR_ODOM", "等待 FAST-LIVO2 odom 输出");
      return;
    }
    if (!odomFresh(now))
    {
      publishSuppressed(now, "DISCONNECTED", "ODOM_STALE",
                        "odom 输出过期，不发布辅助路径，请保持人工控制");
      return;
    }
    last_retreat_base_yaw_ = normAngle(current_yaw_ + kPi);
    last_retreat_uses_obstacle_ = false;

    std::string state = currentHealthState(now);
    if (state == "STARTING" || state == "ENDED" || state == "CRITICAL" || state == "DISCONNECTED")
    {
      const std::string reason = state == "CRITICAL" ? "CRITICAL_HEALTH" :
                                 state == "DISCONNECTED" ? "HEALTH_STALE" :
                                 state == "ENDED" ? "RUN_ENDED" : "WAITING_FOR_HEALTH";
      const std::string prompt = state == "CRITICAL" ? "请悬停或停止贴近，检查建图可靠性" :
                                 state == "DISCONNECTED" ? "健康监测数据过期，请保持人工控制" :
                                 state == "ENDED" ? "本次运行已结束，不发布辅助路径" : "等待在线建图健康状态";
      publishSuppressed(now, state, reason, prompt);
      return;
    }

    if (confidenceVeryLow())
    {
      publishSuppressed(now, state, "CONFIDENCE_VERY_LOW",
                        "建图健康判断证据置信度极低，请悬停并检查诊断话题");
      return;
    }

    if (require_fresh_cloud_ && suppress_on_frame_mismatch_ && !cloud_frame_ok_)
    {
      publishSuppressed(now, state, "CLOUD_FRAME_MISMATCH",
                        "点云坐标系与 world_frame 不一致，不使用错误坐标生成路径");
      return;
    }

    if (require_fresh_cloud_ && !cloudFresh(now))
    {
      publishSuppressed(now, state, "CLOUD_STALE",
                        "点云证据缺失或过期，不发布安全辅助路径");
      return;
    }

    const std::string planning_state = actionAdjustedPlanningState(confidenceAdjustedState(state));
    last_planning_state_ = planning_state;  // expose to publishStatus/writeCsv
    if (planning_state == "CRITICAL")
    {
      publishSuppressed(now, state, "ACTION_CODE_GATE",
                        "健康行动码要求悬停，不发布辅助路径");
      return;
    }
    const std::string intent = activePilotIntent(now);
    if (intent == "HOLD_STABLE")
    {
      publishSuppressed(now, state, "PILOT_HOLD", "Pilot requested HOLD_STABLE; health HUD only.");
      return;
    }
    if (suppressForNoIntent(planning_state, intent))
    {
      publishSuppressed(now, state, "NO_PILOT_INTENT",
                        "No pilot intent selected; health HUD only.");
      return;
    }

    const std::string mode = modeForState(planning_state);
    std::vector<Candidate> candidates = generateCandidates(planning_state, now);
    if (candidates.empty())
    {
      publishSuppressed(now, state, "NO_SAFE_CANDIDATE", noCandidatePrompt(mode));
      return;
    }

    auto best_it = std::max_element(candidates.begin(), candidates.end(),
                                   [](const Candidate &a, const Candidate &b) { return a.score < b.score; });
    if (best_it->score < minAcceptScoreForMode(mode))
    {
      publishSuppressed(now, state, "LOW_ACCEPT_SCORE",
                        "Best local assist score is too low; hold and re-check.");
      return;
    }
    Candidate committed = committedCandidate(*best_it, planning_state, mode, now);
    // Re-check score after goal commitment: the commitment layer must not
    // bypass the quality/safety gate that was already applied to raw_best.
    if (committed.score < minAcceptScoreForMode(mode))
    {
      have_committed_goal_ = false;
      publishSuppressed(now, state, "COMMITTED_SCORE_LOW",
                        "Committed goal quality dropped; hold and re-check.");
      return;
    }
    for (auto &c : candidates)
      c.selected = false;
    committed.selected = true;
    updateSectorVisit(committed, now);

    publishCandidateOutputs(now, state, mode, committed, candidates);
  }

  std::string currentHealthState(const ros::Time &now) const
  {
    if (!have_health_)
      return "STARTING";
    const double age = (now - last_health_time_).toSec();
    if (age < 0.0 || age > health_timeout_s_)
      return "DISCONNECTED";
    return health_state_.empty() ? "STARTING" : health_state_;
  }

  std::string normalizeFrame(const std::string &frame) const
  {
    if (!frame.empty() && frame[0] == '/')
      return frame.substr(1);
    return frame;
  }

  bool cloudFresh(const ros::Time &now) const
  {
    if (!have_cloud_)
      return false;
    if (local_cloud_.empty())
      return false;
    if (suppress_on_frame_mismatch_ && !cloud_frame_ok_)
      return false;
    const double age = (now - last_cloud_time_).toSec();
    return age >= 0.0 && age <= cloud_timeout_s_;
  }

  bool odomFresh(const ros::Time &now) const
  {
    if (!have_odom_)
      return false;
    const double age = (now - last_odom_time_).toSec();
    return age >= 0.0 && age <= odom_timeout_s_;
  }

  std::string activePilotIntent(const ros::Time &now) const
  {
    if (!have_intent_msg_)
      return default_pilot_intent_;
    if (pilot_intent_duration_s_ > 0.0)
    {
      const double age = (now - last_intent_time_).toSec();
      if (age < 0.0 || age > pilot_intent_duration_s_)
        return default_pilot_intent_;
    }
    return pilot_intent_;
  }

  double intentAgeSeconds(const ros::Time &now) const
  {
    if (!have_intent_msg_ || last_intent_time_.isZero())
      return -1.0;
    return (now - last_intent_time_).toSec();
  }

  double activePilotIntentStrength(const ros::Time &now) const
  {
    const std::string intent = activePilotIntent(now);
    if (!directionalIntent(intent))
      return 0.0;
    if (!have_intent_msg_)
      return 1.0;
    if (pilot_intent_duration_s_ > 0.0)
    {
      const double age = (now - last_intent_time_).toSec();
      if (age < 0.0 || age > pilot_intent_duration_s_)
        return directionalIntent(default_pilot_intent_) ? 1.0 : 0.0;
    }
    return pilot_intent_strength_;
  }

  bool directionalIntent(const std::string &intent) const
  {
    return intent == "FORWARD_SCAN" || intent == "LEFT_ORBIT" || intent == "RIGHT_ORBIT" ||
           intent == "UP_SCAN" || intent == "DOWN_SCAN" || intent == "PULL_BACK";
  }

  std::string pilotModeForIntent(const std::string &intent) const
  {
    if (intent == "FREE")
      return "FREE_ASSIST";
    if (intent == "RESCAN_LOCAL")
      return "RESCAN_ASSIST";
    if (intent == "HOLD_STABLE")
      return "HOLD";
    if (intent == "PULL_BACK")
      return "RECOVERY_ASSIST";
    return "INTENT_ASSIST";
  }

  bool suppressForNoIntent(const std::string &state, const std::string &intent) const
  {
    if (free_mode_publishes_suggestions_ || intent != "FREE")
      return false;
    return state == "NORMAL" || state == "WATCH" || state == "RECOVERING";
  }

  double surfaceAwayYaw(const ros::Time &now, bool *found_surface = nullptr) const
  {
    if (found_surface)
      *found_surface = false;
    Point3 nearest;
    if (evidence_grid_enabled_ &&
        evidence_grid_.nearestOccupied(current_pos_, local_z_radius_m_, now, &nearest))
    {
      if (found_surface)
        *found_surface = true;
      return std::atan2(current_pos_.y - nearest.y, current_pos_.x - nearest.x);
    }
    return normAngle(latched_intent_yaw_ + kPi);
  }

  double intentYaw(const std::string &intent, const std::string &state, const ros::Time &now) const
  {
    if (intent == "LEFT_ORBIT")
    {
      bool found_surface = false;
      const double normal_yaw = surfaceAwayYaw(now, &found_surface);
      return found_surface ? normAngle(normal_yaw + 0.5 * kPi) : normAngle(latched_intent_yaw_ + 0.5 * kPi);
    }
    if (intent == "RIGHT_ORBIT")
    {
      bool found_surface = false;
      const double normal_yaw = surfaceAwayYaw(now, &found_surface);
      return found_surface ? normAngle(normal_yaw - 0.5 * kPi) : normAngle(latched_intent_yaw_ - 0.5 * kPi);
    }
    if (intent == "PULL_BACK")
      return warningRetreatBaseYaw(now, nullptr);
    if (state == "WARNING")
      return warningRetreatBaseYaw(now, nullptr);
    return latched_intent_yaw_;
  }

  double intentCorridorRad(const std::string &intent) const
  {
    if (intent == "LEFT_ORBIT" || intent == "RIGHT_ORBIT")
      return orbit_intent_corridor_deg_ * kPi / 180.0;
    if (intent == "UP_SCAN" || intent == "DOWN_SCAN")
      return vertical_intent_corridor_deg_ * kPi / 180.0;
    return intent_corridor_deg_ * kPi / 180.0;
  }

  bool intentAllowsCandidate(const Candidate &c, const std::string &state,
                             const std::string &intent, double intent_yaw) const
  {
    if (!directionalIntent(intent) || state == "WARNING")
      return true;
    if (intent == "UP_SCAN" && c.p.z <= current_pos_.z + 1e-3)
      return false;
    if (intent == "DOWN_SCAN" && c.p.z >= current_pos_.z - 1e-3)
      return false;
    return std::fabs(normAngle(c.move_yaw - intent_yaw)) <= intentCorridorRad(intent);
  }

  // Return the largest absolute z offset in z_offsets_m_ as the normalization
  // denominator for UP/DOWN task progress.  Using the full local_z_radius_m_
  // (5 m) against a 0.75 m step gives z_gain = 0.15 and makes UP/DOWN intent
  // nearly invisible in the score.
  double maxConfiguredZStep() const
  {
    double m = 0.0;
    for (double z : z_offsets_m_)
      m = std::max(m, std::fabs(z));
    return std::max(m, 1e-3);
  }

  // Return the per-mode minimum acceptable score.  EXPLORE/INTENT modes require
  // a positive contribution to justify publishing a suggestion; retreat/recover
  // modes accept lower scores because the primary goal is safety, not coverage.
  double minAcceptScoreForMode(const std::string &mode) const
  {
    if (mode == "EXPLORE")      return min_accept_score_explore_;
    if (mode == "CONSERVATIVE") return min_accept_score_conservative_;
    if (mode == "RECOVER")      return min_accept_score_recover_;
    if (mode == "RETREAT_OR_HOLD") return min_accept_score_retreat_;
    return min_accept_score_;  // global fallback
  }

  double taskWeightForState(const std::string &state) const
  {
    if (state == "WARNING")
      return 0.0;
    if (state == "WATCH")
      return 0.70 * w_task_progress_;
    if (state == "RECOVERING")
      return 0.40 * w_task_progress_;
    return w_task_progress_;
  }

  double taskProgress(const Candidate &c, const std::string &state,
                      const std::string &intent, double intent_yaw) const
  {
    if (!directionalIntent(intent) || state == "WARNING")
      return 0.0;
    const double yaw_alignment = 0.5 + 0.5 * std::cos(normAngle(c.move_yaw - intent_yaw));
    if (intent == "UP_SCAN")
    {
      // Normalize by the largest configured z step, not by the full local
      // z radius — otherwise a 0.75 m step against 5.0 m radius gives
      // z_gain = 0.15, making UP/DOWN intent nearly invisible in the score.
      const double z_gain = clamp((c.p.z - current_pos_.z) / maxConfiguredZStep(), 0.0, 1.0);
      return clamp(0.65 * z_gain + 0.35 * yaw_alignment, 0.0, 1.0);
    }
    if (intent == "DOWN_SCAN")
    {
      const double z_gain = clamp((current_pos_.z - c.p.z) / maxConfiguredZStep(), 0.0, 1.0);
      return clamp(0.65 * z_gain + 0.35 * yaw_alignment, 0.0, 1.0);
    }
    return clamp(yaw_alignment, 0.0, 1.0);
  }

  double scoreCandidate(const Candidate &c, const std::string &state) const
  {
    return taskWeightForState(state) * c.task_progress +
           w_coverage_ * c.coverage_gain -
           w_collision_ * c.collision_risk -
           w_health_ * c.health_penalty -
           w_unknown_ * c.unknown_risk -
           w_path_length_ * c.path_length_cost -
           w_turn_ * c.turn_cost;
  }

  bool confidenceVeryLow() const
  {
    return confidence_very_low_suppresses_ && confidence_level_ == "VERY_LOW";
  }

  std::string confidenceAdjustedState(const std::string &state) const
  {
    if (confidence_low_forces_conservative_ && confidence_level_ == "LOW" && state == "NORMAL")
      return "WATCH";
    return state;
  }

  // Translate action_code from the health monitor into a planning-level state
  // override.  This keeps health_state as-is in logs while still letting the
  // monitor act as an upper-level safety supervisor for the planner.
  std::string actionAdjustedPlanningState(const std::string &state) const
  {
    // HOVER: health monitor demands the UAV stay still → treat as CRITICAL for planning.
    if (health_action_code_ == "HOVER")
      return "CRITICAL";

    // CHECK_EVIDENCE / KEEP_STABLE_CHECK_EVIDENCE: pause aggressive scanning.
    // With low/very-low confidence, suppress entirely; otherwise fall back to WATCH.
    if (health_action_code_ == "CHECK_EVIDENCE" ||
        health_action_code_ == "KEEP_STABLE_CHECK_EVIDENCE")
    {
      if (confidence_level_ == "VERY_LOW" || confidence_level_ == "LOW")
        return "CRITICAL";
      return state == "NORMAL" ? "WATCH" : state;
    }

    // REDUCE_MOTION: at least conservative planning.
    if (health_action_code_ == "REDUCE_MOTION")
      return state == "NORMAL" ? "WATCH" : state;

    // RECOVER_SLOW: switch to the RECOVERING planning mode.
    if (health_action_code_ == "RECOVER_SLOW")
      return "RECOVERING";

    return state;
  }

  std::string modeForState(const std::string &state) const
  {
    if (state == "NORMAL") return "EXPLORE";
    if (state == "WATCH") return "CONSERVATIVE";
    if (state == "WARNING") return "RETREAT_OR_HOLD";
    if (state == "RECOVERING") return "RECOVER";
    return "SUPPRESSED";
  }

  std::string promptForMode(const std::string &mode) const
  {
    if (mode == "EXPLORE") return "建议按推荐方向进行局部补扫";
    if (mode == "CONSERVATIVE") return "请减速，按短距离保守路径补扫";
    if (mode == "RETREAT_OR_HOLD") return "请拉远或侧移，停止继续贴近扫描";
    if (mode == "RECOVER") return "请低速平稳恢复，不要立即贴近";
    return "请悬停或停止贴近，人工检查建图状态";
  }

  std::string noCandidatePrompt(const std::string &mode) const
  {
    if (mode == "RETREAT_OR_HOLD")
      return "后退或侧移方向也不安全，请悬停并人工判断退出方向";
    if (mode == "CONSERVATIVE")
      return "保守补扫方向暂不安全，请减速悬停并重新观察";
    if (mode == "RECOVER")
      return "恢复路径暂不安全，请继续悬停等待建图稳定";
    return "未找到安全辅助路径，请悬停或人工调整";
  }

  bool reasonSuggestsViewChange() const
  {
    // Only VISUAL_DEGRADED_LIO_STABLE explicitly indicates that LIO is stable
    // and only the visual front-end is weak — a targeted view-angle change is
    // then a reasonable response.
    //
    // VISUAL_POINTS_LOW and VISUAL_TRACK_POOR do NOT imply LIO is stable; they
    // can be caused by motion blur, proximity, occlusion, or poor lighting.
    // In WARNING state those conditions call for retreat, not more scanning.
    return health_primary_reason_ == "VISUAL_DEGRADED_LIO_STABLE";
  }

  bool reasonSuggestsStableMotion() const
  {
    return motion_flags_ != "NONE" && !motion_flags_.empty();
  }

  std::vector<Candidate> generateCandidates(const std::string &state, const ros::Time &now)
  {
    std::vector<Candidate> candidates;
    candidates.reserve(sample_count_);

    double radius = sample_radius_m_;
    double obstacle_margin = min_obstacle_distance_m_;
    double base_move_yaw = current_yaw_;
    double rel_start = -kPi;
    double rel_span = 2.0 * kPi;
    const std::string intent = activePilotIntent(now);
    const double intent_yaw = intentYaw(intent, state, now);
    last_retreat_base_yaw_ = normAngle(current_yaw_ + kPi);
    last_retreat_uses_obstacle_ = false;

    if (state == "WATCH")
    {
      radius = clamp(sample_radius_m_ * watch_sample_radius_scale_, min_goal_distance_m_, max_goal_distance_m_);
      obstacle_margin *= watch_obstacle_distance_scale_;
      rel_start = -0.5 * kPi;
      rel_span = kPi;
    }
    else if (state == "WARNING")
    {
      obstacle_margin *= watch_obstacle_distance_scale_;
      if (reasonSuggestsViewChange())
      {
        // LIO is confirmed stable; only the visual front-end is weak.
        // Allow a small view-angle change in the forward hemisphere instead
        // of a full retreat — the pilot may recover visual features by shifting
        // laterally or tilting slightly.
        radius = clamp(sample_radius_m_ * watch_sample_radius_scale_,
                       min_goal_distance_m_, max_goal_distance_m_);
        base_move_yaw = current_yaw_;
        rel_start = -0.5 * kPi;
        rel_span = kPi;
      }
      else
      {
        // Default WARNING strategy: retreat away from the nearest obstacle.
        // motion_flags != NONE (fast/jerky motion) shrinks the step radius and
        // tightens the corridor for stability, but does NOT redirect toward
        // forward motion — motion intensity is a diagnostic observation, not a
        // reason to keep approaching a surface.
        bool retreat_uses_obstacle = false;
        base_move_yaw = warningRetreatBaseYaw(now, &retreat_uses_obstacle);
        last_retreat_uses_obstacle_ = retreat_uses_obstacle;
        last_retreat_base_yaw_ = base_move_yaw;
        if (reasonSuggestsStableMotion())
        {
          // Reduce step size for smoother motion, but stay on the retreat vector.
          radius = clamp(warning_retreat_distance_m_ * 0.5,
                         min_goal_distance_m_, max_goal_distance_m_);
          rel_start = -kPi / 4.0;
          rel_span = kPi / 2.0;
        }
        else
        {
          radius = clamp(warning_retreat_distance_m_,
                         min_goal_distance_m_, max_goal_distance_m_);
          rel_start = -kPi / 3.0;
          rel_span = 2.0 * kPi / 3.0;
        }
      }
    }
    else if (state == "RECOVERING")
    {
      radius = clamp(std::min(sample_radius_m_ * watch_sample_radius_scale_, recovering_max_goal_distance_m_),
                     min_goal_distance_m_, max_goal_distance_m_);
      obstacle_margin *= watch_obstacle_distance_scale_;
      rel_start = -0.25 * kPi;
      rel_span = 0.5 * kPi;
    }

    const double current_min_dist = minDistanceToPoint(current_pos_, now);
    const int denom = std::max(1, sample_count_ - 1);
    for (int i = 0; i < sample_count_; ++i)
    {
      const double sample_rel = sample_count_ == 1 ? 0.0 : rel_start + rel_span * static_cast<double>(i) / denom;
      const double move_yaw = normAngle(base_move_yaw + sample_rel);
      const double rel = normAngle(move_yaw - current_yaw_);
      const double view_yaw_seed = (state == "WARNING" || state == "RECOVERING") ? current_yaw_ : move_yaw;
      for (const double z_offset : z_offsets_m_)
      {
        Candidate c;
        c.rel_yaw = normAngle(rel);
        c.move_yaw = move_yaw;
        c.p.x = current_pos_.x + radius * std::cos(move_yaw);
        c.p.y = current_pos_.y + radius * std::sin(move_yaw);
        c.p.z = current_pos_.z + z_offset;
        if (!intentAllowsCandidate(c, state, intent, intent_yaw))
          continue;
        c.view_yaw = preferredViewYaw(c.p, view_yaw_seed, state, now);
        c.yaw = c.view_yaw;
        c.distance = distance3D(current_pos_, c.p);
        if (c.distance < min_goal_distance_m_ || c.distance > max_goal_distance_m_)
          continue;

        const double point_min = minDistanceToPoint(c.p, now);
        const double path_min = minDistanceToSegment(current_pos_, c.p, now);
        c.min_obstacle_distance = std::min(point_min, path_min);

        if (evidence_grid_enabled_)
        {
          c.path_evidence = evidence_grid_.pathEvidence(current_pos_, c.p, path_check_step_m_, now);
          c.unknown_risk = c.path_evidence.unknown_ratio;
          if (c.path_evidence.occupied_count > 0)
            continue;
          if (c.path_evidence.unknown_ratio > maxUnknownPathRatioForState(state))
            continue;
        }

        if (isFinite(c.min_obstacle_distance) && c.min_obstacle_distance < obstacle_margin)
          continue;
        if (state == "WARNING" && isFinite(current_min_dist) && isFinite(c.min_obstacle_distance) &&
            c.min_obstacle_distance + 0.05 < current_min_dist)
          continue;
        if (state == "RECOVERING" && std::fabs(c.rel_yaw) > max_yaw_change_rad_ * 0.75)
          continue;

        c.sector = sectorIndex(c.move_yaw);
        c.coverage_gain = coverageGain(c, now);
        c.collision_risk = collisionRisk(c.min_obstacle_distance, obstacle_margin, now);
        c.health_penalty = healthPenalty(state, c.min_obstacle_distance, c.rel_yaw, current_min_dist);
        c.path_length_cost = c.distance / std::max(max_goal_distance_m_, 1e-6);
        c.turn_cost = std::fabs(c.rel_yaw) / kPi;
        c.task_progress = activePilotIntentStrength(now) * taskProgress(c, state, intent, intent_yaw);
        c.score = scoreCandidate(c, state);
        candidates.push_back(c);
      }
    }
    applySurfaceGainFallback(candidates, state);
    return candidates;
  }

  void applySurfaceGainFallback(std::vector<Candidate> &candidates, const std::string &state) const
  {
    if (!evidence_grid_enabled_ || candidates.empty() || surface_exhausted_density_fallback_scale_ <= 0.0)
      return;
    double best_surface_gain = 0.0;
    for (const auto &c : candidates)
      best_surface_gain = std::max(best_surface_gain, c.coverage_gain);
    if (best_surface_gain > 1e-6)
      return;

    for (auto &c : candidates)
    {
      c.coverage_gain = clamp(surface_exhausted_density_fallback_scale_ *
                                  densityHeuristicGain(c.move_yaw, c.sector),
                              0.0, 1.0);
      c.score = scoreCandidate(c, state);
    }
  }

  double warningRetreatBaseYaw(const ros::Time &now, bool *from_obstacle) const
  {
    if (from_obstacle)
      *from_obstacle = false;
    Point3 nearest;
    if (evidence_grid_enabled_ &&
        evidence_grid_.nearestOccupied(current_pos_, local_z_radius_m_, now, &nearest))
    {
      if (from_obstacle)
        *from_obstacle = true;
      return std::atan2(current_pos_.y - nearest.y, current_pos_.x - nearest.x);
    }

    double best_sq = std::numeric_limits<double>::infinity();
    bool found = false;
    for (const auto &q : local_cloud_)
    {
      if (std::fabs(q.z - current_pos_.z) > local_z_radius_m_)
        continue;
      const double dx = q.x - current_pos_.x;
      const double dy = q.y - current_pos_.y;
      const double d2 = dx * dx + dy * dy;
      // Mirror the fix in nearestOccupied: 0.04 (0.2 m) was a blind zone that
      // skipped the nearest raw-cloud point when the UAV was very close to a wall.
      if (d2 < 1e-6 || d2 >= best_sq)
        continue;
      best_sq = d2;
      nearest = q;
      found = true;
    }

    if (found)
    {
      if (from_obstacle)
        *from_obstacle = true;
      return std::atan2(current_pos_.y - nearest.y, current_pos_.x - nearest.x);
    }
    return normAngle(current_yaw_ + kPi);
  }

  double preferredViewYaw(const Point3 &candidate_pos, double fallback_yaw,
                          const std::string &state, const ros::Time &now) const
  {
    if (state == "WARNING" || state == "RECOVERING")
      return fallback_yaw;
    Point3 nearest;
    if (evidence_grid_enabled_ &&
        evidence_grid_.nearestOccupied(candidate_pos, local_z_radius_m_, now, &nearest))
    {
      return std::atan2(nearest.y - candidate_pos.y, nearest.x - candidate_pos.x);
    }
    return fallback_yaw;
  }

  double maxUnknownPathRatioForState(const std::string &state) const
  {
    if (state == "WARNING")
      return warning_max_unknown_path_ratio_;
    if (state == "RECOVERING")
      return recovering_max_unknown_path_ratio_;
    if (state == "WATCH")
      return std::min(max_unknown_path_ratio_, 0.25);
    return max_unknown_path_ratio_;
  }

  Candidate committedCandidate(const Candidate &raw_best, const std::string &state,
                               const std::string &mode, const ros::Time &now)
  {
    if (!enable_goal_commitment_)
      return commitNewCandidate(raw_best, state, mode, now);

    const std::string intent = activePilotIntent(now);
    if (!have_committed_goal_ || committed_state_ != state || committed_mode_ != mode ||
        committed_intent_ != intent)
      return commitNewCandidate(raw_best, state, mode, now);

    Candidate kept = committed_candidate_;
    if (!refreshCommittedCandidate(kept, state, now))
      return commitNewCandidate(raw_best, state, mode, now);

    const double age = (now - committed_since_).toSec();
    const double bearing_delta_deg = std::fabs(normAngle(raw_best.move_yaw - kept.move_yaw)) * 180.0 / kPi;
    const bool much_better = raw_best.score > kept.score + switch_score_margin_;
    const bool meaningfully_different = bearing_delta_deg >= switch_bearing_margin_deg_;

    if (age < min_goal_hold_s_)
      return kept;
    if (age > max_goal_hold_s_ && much_better)
      return commitNewCandidate(raw_best, state, mode, now);
    if (much_better && meaningfully_different)
      return commitNewCandidate(raw_best, state, mode, now);
    return kept;
  }

  Candidate commitNewCandidate(const Candidate &candidate, const std::string &state,
                               const std::string &mode, const ros::Time &now)
  {
    committed_candidate_ = candidate;
    committed_state_ = state;
    committed_mode_ = mode;
    committed_intent_ = activePilotIntent(now);
    committed_since_ = now;
    have_committed_goal_ = true;
    return committed_candidate_;
  }

  bool refreshCommittedCandidate(Candidate &c, const std::string &state, const ros::Time &now) const
  {
    const double dx = c.p.x - current_pos_.x;
    const double dy = c.p.y - current_pos_.y;
    c.move_yaw = std::atan2(dy, dx);
    c.rel_yaw = normAngle(c.move_yaw - current_yaw_);
    c.distance = distance3D(current_pos_, c.p);
    if (c.distance < min_goal_distance_m_ * 0.45 || c.distance > max_goal_distance_m_ * 1.35)
      return false;

    const double obstacle_margin =
        min_obstacle_distance_m_ * ((state == "WATCH" || state == "WARNING" || state == "RECOVERING")
                                      ? watch_obstacle_distance_scale_ : 1.0);
    const double point_min = minDistanceToPoint(c.p, now);
    const double path_min = minDistanceToSegment(current_pos_, c.p, now);
    c.min_obstacle_distance = std::min(point_min, path_min);
    if (isFinite(c.min_obstacle_distance) && c.min_obstacle_distance < obstacle_margin)
      return false;

    if (evidence_grid_enabled_)
    {
      c.path_evidence = evidence_grid_.pathEvidence(current_pos_, c.p, path_check_step_m_, now);
      c.unknown_risk = c.path_evidence.unknown_ratio;
      if (c.path_evidence.occupied_count > 0)
        return false;
      if (c.path_evidence.unknown_ratio > maxUnknownPathRatioForState(state))
        return false;
    }

    const double current_min_dist = minDistanceToPoint(current_pos_, now);
    c.sector = sectorIndex(c.move_yaw);
    c.coverage_gain = coverageGain(c, now);
    c.collision_risk = collisionRisk(c.min_obstacle_distance, obstacle_margin, now);
    c.health_penalty = healthPenalty(state, c.min_obstacle_distance, c.rel_yaw, current_min_dist);
    c.path_length_cost = c.distance / std::max(max_goal_distance_m_, 1e-6);
    c.turn_cost = std::fabs(c.rel_yaw) / kPi;
    const std::string intent = activePilotIntent(now);
    const double current_intent_yaw = intentYaw(intent, state, now);
    if (!intentAllowsCandidate(c, state, intent, current_intent_yaw))
      return false;
    c.task_progress = activePilotIntentStrength(now) * taskProgress(c, state, intent, current_intent_yaw);
    c.score = scoreCandidate(c, state);
    // Committed goal must still clear the quality threshold even after a
    // refreshed score — the commitment layer must not bypass the safety gate.
    if (c.score < min_accept_score_)
      return false;
    return true;
  }

  double minDistanceToPoint(const Point3 &p, const ros::Time &now) const
  {
    if (evidence_grid_enabled_)
      return evidence_grid_.minDistanceToOccupiedPoint(p, now);
    if (local_cloud_.empty())
      return std::numeric_limits<double>::infinity();
    double best = std::numeric_limits<double>::infinity();
    for (const auto &q : local_cloud_)
    {
      best = std::min(best, distance3D(p, q));
    }
    return best;
  }

  double minDistanceToSegment(const Point3 &a, const Point3 &b, const ros::Time &now) const
  {
    if (evidence_grid_enabled_)
      return evidence_grid_.minDistanceToOccupiedSegment(a, b, now);
    if (local_cloud_.empty())
      return std::numeric_limits<double>::infinity();
    double best = std::numeric_limits<double>::infinity();
    for (const auto &q : local_cloud_)
    {
      best = std::min(best, pointSegmentDistance3D(q, a, b));
    }
    return best;
  }

  int sectorIndex(double yaw) const
  {
    double a = normAngle(yaw);
    if (a < 0.0) a += 2.0 * kPi;
    int idx = static_cast<int>(std::floor(a / (2.0 * kPi) * sector_count_));
    if (idx >= sector_count_) idx = sector_count_ - 1;
    if (idx < 0) idx = 0;
    return idx;
  }

  double coverageGain(const Candidate &candidate, const ros::Time &now) const
  {
    if (evidence_grid_enabled_)
    {
      const double surface_gain = evidence_grid_.surfaceFrontierGain(
          candidate.p, candidate.view_yaw, now,
          min_standoff_m_, ideal_standoff_m_, max_standoff_m_,
          coverage_view_fov_rad_, frontier_target_voxels_);
      return clamp(surface_gain - repeatPenalty(candidate.sector), 0.0, 1.0);
    }
    return densityHeuristicGain(candidate.move_yaw, candidate.sector);
  }

  double densityHeuristicGain(double yaw, int sector) const
  {
    if (!have_cloud_ || local_cloud_.empty())
      return 0.0;

    const double half_width = kPi / static_cast<double>(sector_count_);
    int count = 0;
    int near_count = 0;
    for (const auto &p : local_cloud_)
    {
      const double dx = p.x - current_pos_.x;
      const double dy = p.y - current_pos_.y;
      const double r = std::sqrt(dx * dx + dy * dy);
      if (r < min_goal_distance_m_ || r > local_radius_m_)
        continue;
      const double a = std::atan2(dy, dx);
      if (std::fabs(normAngle(a - yaw)) <= half_width)
      {
        ++count;
        if (r < min_obstacle_distance_m_ * 1.5)
          ++near_count;
      }
    }

    double gain = 0.0;
    if (count <= 0)
    {
      gain = 0.10;  // Weak candidate only. Sparse evidence is not true unknown space.
    }
    else if (count < min_sector_points_for_strong_gain_)
    {
      gain = 0.15 + 0.25 * static_cast<double>(count) / std::max(1, min_sector_points_for_strong_gain_);
    }
    else if (count <= target_sector_points_)
    {
      gain = 0.40 + 0.50 * static_cast<double>(count) / std::max(1, target_sector_points_);
    }
    else
    {
      const double excess = static_cast<double>(count - target_sector_points_) / std::max(1, target_sector_points_);
      gain = std::max(0.25, 0.90 - 0.35 * excess);
    }

    if (near_count > 0)
    {
      gain -= std::min(0.20, 0.02 * near_count);
    }
    if (sector >= 0 && sector < static_cast<int>(sector_visits_.size()))
    {
      gain -= repeatPenalty(sector);
    }
    return clamp(gain, 0.0, 1.0);
  }

  double repeatPenalty(int sector) const
  {
    if (sector >= 0 && sector < static_cast<int>(sector_visits_.size()))
    {
      const double visits = static_cast<double>(sector_visits_[sector]);
      return std::min(repeat_sector_penalty_cap_, repeat_sector_penalty_ * visits * visits);
    }
    return 0.0;
  }

  double collisionRisk(double min_dist, double obstacle_margin, const ros::Time &now) const
  {
    if (!isFinite(min_dist))
    {
      // When evidence_grid is enabled and minDistanceToOccupiedPoint returned ∞,
      // no occupied voxels exist in the local grid — calling countState(OCCUPIED)
      // would just confirm that with an O(N) traversal.  Skip it.
      return have_cloud_ ? 0.25 : 0.50;
    }
    const double buffer = std::max(obstacle_margin * 3.0, obstacle_margin + 0.5);
    if (min_dist >= buffer)
      return 0.0;
    return clamp((buffer - min_dist) / std::max(buffer - obstacle_margin, 1e-6), 0.0, 1.0);
  }

  double healthPenalty(const std::string &state, double min_dist, double rel_yaw, double current_min_dist) const
  {
    double penalty = 0.0;
    if (state == "WATCH")
      penalty += 0.20;
    else if (state == "WARNING")
      penalty += 0.60;
    else if (state == "RECOVERING")
      penalty += 0.40;

    if (isFinite(min_dist) && min_dist < min_obstacle_distance_m_ * 2.0)
      penalty += 0.25 * (1.0 - clamp((min_dist - min_obstacle_distance_m_) / min_obstacle_distance_m_, 0.0, 1.0));

    if (state == "WATCH" || state == "RECOVERING")
      penalty += 0.20 * clamp(std::fabs(rel_yaw) / kPi, 0.0, 1.0);
    if (reasonSuggestsStableMotion())
      penalty += 0.25 * clamp(std::fabs(rel_yaw) / kPi, 0.0, 1.0);

    if (state == "WARNING")
    {
      // Apply the forward-motion penalty only for generic retreat mode.
      // When the reason calls for a view-angle change (visual degradation) or
      // stable small motion, the candidate set already sits in the forward
      // hemisphere — adding +0.80 there would flatten all scores and destroy
      // the discrimination the scoring function is supposed to provide.
      if (!reasonSuggestsViewChange() && !reasonSuggestsStableMotion())
      {
        const bool forward = std::cos(rel_yaw) > 0.0;
        if (forward)
          penalty += 0.80;
      }
      if (isFinite(current_min_dist) && isFinite(min_dist) && min_dist < current_min_dist)
        penalty += 0.50;
    }

    // fast_livo2_health_monitor publishes reliability_score on a 0-100 scale.
    if (reliability_score_ >= 0.0 && reliability_score_ < 80.0)
      penalty += clamp((80.0 - reliability_score_) / 80.0, 0.0, 0.5);

    return clamp(penalty, 0.0, 2.0);
  }

  void updateSectorVisit(const Candidate &best, const ros::Time &now)
  {
    if (best.sector < 0 || best.sector >= static_cast<int>(sector_visits_.size()))
      return;
    if (best.sector != last_selected_sector_ || last_sector_visit_time_.isZero() ||
        (now - last_sector_visit_time_).toSec() > 5.0)
    {
      sector_visits_[best.sector] += 1;
      last_selected_sector_ = best.sector;
      last_sector_visit_time_ = now;
    }
  }

  void decaySectorVisits(const ros::Time &now)
  {
    if (!last_sector_decay_time_.isZero() && (now - last_sector_decay_time_).toSec() < 10.0)
      return;
    last_sector_decay_time_ = now;
    for (auto &v : sector_visits_)
    {
      if (v > 0)
        --v;
    }
  }

  nav_msgs::Path makeStraightPath(const Point3 &start, const Point3 &goal,
                                  double start_yaw, double goal_yaw,
                                  const ros::Time &stamp) const
  {
    nav_msgs::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = world_frame_;
    const double dist = distance3D(start, goal);
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / path_check_step_m_)));
    for (int i = 0; i <= steps; ++i)
    {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      Point3 p;
      p.x = start.x + t * (goal.x - start.x);
      p.y = start.y + t * (goal.y - start.y);
      p.z = start.z + t * (goal.z - start.z);
      const double yaw = normAngle(start_yaw + t * normAngle(goal_yaw - start_yaw));
      path.poses.push_back(poseStamped(p, yaw, stamp));
    }
    return path;
  }

  geometry_msgs::PoseStamped poseStamped(const Point3 &p, double yaw, const ros::Time &stamp) const
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = p.z;
    pose.pose.orientation = quatFromYaw(yaw);
    return pose;
  }

  void publishCandidateOutputs(const ros::Time &now, const std::string &state, const std::string &mode,
                               const Candidate &best, const std::vector<Candidate> &candidates)
  {
    nav_msgs::Path path;
    path = makeStraightPath(current_pos_, best.p, current_yaw_, best.view_yaw, now);
    pub_path_.publish(path);

    pub_goal_.publish(poseStamped(best.p, best.view_yaw, now));

    geometry_msgs::PoseArray poses;
    poses.header = path.header;
    for (const auto &c : candidates)
      poses.poses.push_back(poseStamped(c.p, c.yaw, now).pose);
    pub_candidates_.publish(poses);

    pub_markers_.publish(makeMarkers(now, state, mode, best, candidates));
    publishStatus(now, state, mode, "OK", promptForMode(mode), best, candidates.size());
    maybePrintStatus(now, state, mode, promptForMode(mode), candidates.size(), best.score);
  }

  void publishSuppressed(const ros::Time &now, const std::string &state,
                         const std::string &reason, const std::string &prompt)
  {
    have_committed_goal_ = false;

    nav_msgs::Path path;
    path.header.stamp = now;
    path.header.frame_id = world_frame_;
    pub_path_.publish(path);

    geometry_msgs::PoseArray poses;
    poses.header = path.header;
    pub_candidates_.publish(poses);

    if (critical_publish_hold_goal_ && odomFresh(now))
      pub_goal_.publish(poseStamped(current_pos_, current_yaw_, now));

    Candidate empty;
    empty.score = -1.0;
    empty.coverage_gain = -1.0;
    empty.collision_risk = -1.0;
    empty.health_penalty = -1.0;
    empty.distance = 0.0;
    pub_markers_.publish(makeSuppressedMarkers(now, state, reason, prompt));
    publishStatus(now, state, "SUPPRESSED", reason, prompt, empty, 0);
    maybePrintStatus(now, state, "SUPPRESSED", prompt, 0, -1.0);
  }

  visualization_msgs::MarkerArray makeMarkers(const ros::Time &now, const std::string &state,
                                              const std::string &mode, const Candidate &best,
                                              const std::vector<Candidate> &candidates) const
  {
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(deleteAllMarker(now));

    visualization_msgs::Marker line;
    line.header.frame_id = world_frame_;
    line.header.stamp = now;
    line.ns = "assist";
    line.id = 1;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.08;
    line.color = colorForMode(mode, 0.95);
    const nav_msgs::Path path = makeStraightPath(current_pos_, best.p, current_yaw_, best.view_yaw, now);
    for (const auto &pose : path.poses)
    {
      geometry_msgs::Point p;
      p.x = pose.pose.position.x;
      p.y = pose.pose.position.y;
      p.z = pose.pose.position.z;
      line.points.push_back(p);
    }
    arr.markers.push_back(line);

    visualization_msgs::Marker goal;
    goal.header.frame_id = world_frame_;
    goal.header.stamp = now;
    goal.ns = "assist";
    goal.id = 2;
    goal.type = visualization_msgs::Marker::SPHERE;
    goal.action = visualization_msgs::Marker::ADD;
    goal.pose.position.x = best.p.x;
    goal.pose.position.y = best.p.y;
    goal.pose.position.z = best.p.z;
    goal.pose.orientation.w = 1.0;
    goal.scale.x = 0.35;
    goal.scale.y = 0.35;
    goal.scale.z = 0.35;
    goal.color = colorForMode(mode, 0.95);
    arr.markers.push_back(goal);

    if (show_all_candidates_)
    {
      int id = 100;
      for (const auto &c : candidates)
      {
        visualization_msgs::Marker m;
        m.header.frame_id = world_frame_;
        m.header.stamp = now;
        m.ns = "assist";
        m.id = id++;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = c.p.x;
        m.pose.position.y = c.p.y;
        m.pose.position.z = c.p.z;
        m.pose.orientation.w = 1.0;
        m.scale.x = c.selected ? 0.32 : 0.18;
        m.scale.y = c.selected ? 0.32 : 0.18;
        m.scale.z = c.selected ? 0.32 : 0.18;
        m.color = c.selected ? colorForMode(mode, 0.95) : makeColor(0.1, 0.45, 0.85, 0.35);
        arr.markers.push_back(m);
      }
    }

    arr.markers.push_back(textMarker(now, promptForMode(mode), state));
    return arr;
  }

  visualization_msgs::MarkerArray makeSuppressedMarkers(const ros::Time &now, const std::string &state,
                                                        const std::string &, const std::string &prompt) const
  {
    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(deleteAllMarker(now));
    arr.markers.push_back(textMarker(now, prompt, state));
    return arr;
  }

  visualization_msgs::Marker deleteAllMarker(const ros::Time &now) const
  {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = now;
    m.ns = "assist";
    m.id = 0;
    m.action = visualization_msgs::Marker::DELETEALL;
    return m;
  }

  visualization_msgs::Marker textMarker(const ros::Time &now, const std::string &text, const std::string &state) const
  {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = now;
    m.ns = "assist_text";
    m.id = 10;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = current_pos_.x;
    m.pose.position.y = current_pos_.y;
    m.pose.position.z = current_pos_.z + 1.4;
    m.pose.orientation.w = 1.0;
    m.scale.z = marker_scale_;
    m.color = colorForState(state, 0.95);
    m.text = text;
    return m;
  }

  std_msgs::ColorRGBA makeColor(double r, double g, double b, double a) const
  {
    std_msgs::ColorRGBA c;
    c.r = static_cast<float>(r);
    c.g = static_cast<float>(g);
    c.b = static_cast<float>(b);
    c.a = static_cast<float>(a);
    return c;
  }

  std_msgs::ColorRGBA colorForState(const std::string &state, double alpha) const
  {
    if (state == "NORMAL") return makeColor(0.10, 0.55, 0.25, alpha);
    if (state == "WATCH") return makeColor(0.85, 0.55, 0.05, alpha);
    if (state == "WARNING") return makeColor(0.90, 0.35, 0.05, alpha);
    if (state == "CRITICAL") return makeColor(0.85, 0.05, 0.05, alpha);
    if (state == "RECOVERING") return makeColor(0.10, 0.35, 0.85, alpha);
    return makeColor(0.35, 0.40, 0.45, alpha);
  }

  std_msgs::ColorRGBA colorForMode(const std::string &mode, double alpha) const
  {
    if (mode == "EXPLORE") return makeColor(0.10, 0.55, 0.25, alpha);
    if (mode == "CONSERVATIVE") return makeColor(0.85, 0.55, 0.05, alpha);
    if (mode == "RETREAT_OR_HOLD") return makeColor(0.90, 0.35, 0.05, alpha);
    if (mode == "RECOVER") return makeColor(0.10, 0.35, 0.85, alpha);
    return makeColor(0.85, 0.05, 0.05, alpha);
  }

  void publishStatus(const ros::Time &now, const std::string &state, const std::string &mode,
                     const std::string &reason, const std::string &prompt,
                     const Candidate &best, std::size_t candidate_count)
  {
    // The web dashboard parses this topic as semicolon-delimited key-value pairs.
    // Keep values semicolon-free, or sanitize them before appending.
    const std::string intent = activePilotIntent(now);
    const double current_intent_yaw = intentYaw(intent, state, now);
    const double intent_bearing_deg = normAngle(current_intent_yaw - current_yaw_) * 180.0 / kPi;
    std::ostringstream os;
    os << std::fixed << std::setprecision(3)
       << "mode=" << mode
       << ";pilot_mode=" << pilotModeForIntent(intent)
       << ";pilot_intent=" << intent
       << ";pilot_intent_active=" << (intent == "FREE" ? 0 : 1)
       << ";pilot_intent_strength=" << activePilotIntentStrength(now)
       << ";intent_age_s=" << intentAgeSeconds(now)
       << ";intent_yaw_deg=" << current_intent_yaw * 180.0 / kPi
       << ";intent_bearing_deg=" << intent_bearing_deg
       << ";health_state=" << state
       << ";planning_state=" << kvValue(last_planning_state_)
       << ";health_action_code=" << kvValue(health_action_code_)
       << ";health_primary_reason=" << kvValue(health_primary_reason_)
       << ";motion_flags=" << kvValue(motion_flags_)
       << ";reliability_score=" << reliability_score_
       << ";confidence_score=" << confidence_score_
       << ";confidence_level=" << kvValue(confidence_level_)
       << ";pilot_prompt=" << kvValue(prompt)
       << ";suppressed_reason=" << (mode == "SUPPRESSED" ? kvValue(reason) : "")
       << ";candidate_count=" << candidate_count
       << ";top_score=" << best.score
       << ";coverage_gain=" << best.coverage_gain
       << ";collision_risk=" << best.collision_risk
       << ";health_penalty=" << best.health_penalty
       << ";unknown_risk=" << best.unknown_risk
       << ";task_progress=" << best.task_progress
       << ";path_unknown_ratio=" << best.path_evidence.unknown_ratio
       << ";path_occupied_count=" << best.path_evidence.occupied_count
       << ";target_distance_m=" << best.distance
       << ";target_bearing_deg=" << best.rel_yaw * 180.0 / kPi
       << ";cloud_points=" << local_cloud_.size()
       << ";grid_voxels=" << evidence_grid_.voxelCount()
       << ";grid_free=" << evidence_grid_.countState(VoxelState::FREE, now)
       << ";grid_occupied=" << evidence_grid_.countState(VoxelState::OCCUPIED, now)
       << ";odom_age_s=" << ageSeconds(last_odom_time_)
       << ";health_age_s=" << ageSeconds(last_health_time_)
       << ";cloud_age_s=" << ageSeconds(last_cloud_time_)
       << ";cloud_frame_ok=" << (cloud_frame_ok_ ? 1 : 0)
       << ";cloud_frame=" << kvValue(last_cloud_frame_)
       << ";retreat_base_yaw_deg=" << last_retreat_base_yaw_ * 180.0 / kPi
       << ";retreat_base_source=" << (last_retreat_uses_obstacle_ ? "nearest_obstacle" : "body_backward")
       << ";control_authority=pilot_only";
    std_msgs::String msg;
    msg.data = os.str();
    pub_status_.publish(msg);
    writeCsv(now, state, mode, reason, prompt, best, candidate_count);
  }

  void openCsv()
  {
    if (!csv_enabled_)
      return;
    csv_.open(csv_path_.c_str(), std::ios::out | std::ios::trunc);
    if (!csv_.is_open())
    {
      ROS_WARN("[assist_planner] cannot open csv: %s", csv_path_.c_str());
      return;
    }
    csv_ << "stamp,health_state,planning_state,mode,pilot_mode,pilot_intent,intent_age_s,intent_yaw_deg,"
            "intent_bearing_deg,pilot_intent_strength,reason,pilot_prompt,candidate_count,top_score,"
            "coverage_gain,collision_risk,health_penalty,unknown_risk,task_progress,"
            "path_unknown_ratio,path_occupied_count,"
            "path_length_cost,turn_cost,"
            "target_distance_m,target_bearing_deg,reliability_score,confidence_score,"
            "confidence_level,health_action_code,health_primary_reason,motion_flags,cloud_points,"
            "grid_voxels,grid_free,grid_occupied,"
            "odom_age_s,health_age_s,cloud_age_s,cloud_frame_ok,cloud_frame,"
            "retreat_base_yaw_deg,retreat_base_source,control_authority\n";
  }

  void writeCsv(const ros::Time &now, const std::string &state, const std::string &mode,
                const std::string &reason, const std::string &prompt,
                const Candidate &best, std::size_t candidate_count)
  {
    if (!csv_.is_open())
      return;
    const std::string intent = activePilotIntent(now);
    const double current_intent_yaw = intentYaw(intent, state, now);
    const double intent_bearing_deg = normAngle(current_intent_yaw - current_yaw_) * 180.0 / kPi;
    csv_ << std::fixed << std::setprecision(6)
         << now.toSec() << ','
         << csvCell(state) << ','
         << csvCell(last_planning_state_) << ','
         << csvCell(mode) << ','
         << csvCell(pilotModeForIntent(intent)) << ','
         << csvCell(intent) << ','
         << intentAgeSeconds(now) << ','
         << current_intent_yaw * 180.0 / kPi << ','
         << intent_bearing_deg << ','
         << activePilotIntentStrength(now) << ','
         << csvCell(reason) << ','
         << csvCell(prompt) << ','
         << candidate_count << ','
         << best.score << ','
         << best.coverage_gain << ','
         << best.collision_risk << ','
         << best.health_penalty << ','
         << best.unknown_risk << ','
         << best.task_progress << ','
         << best.path_evidence.unknown_ratio << ','
         << best.path_evidence.occupied_count << ','
         << best.path_length_cost << ','
         << best.turn_cost << ','
         << best.distance << ','
         << best.rel_yaw * 180.0 / kPi << ','
         << reliability_score_ << ','
         << confidence_score_ << ','
         << csvCell(confidence_level_) << ','
         << csvCell(health_action_code_) << ','
         << csvCell(health_primary_reason_) << ','
         << csvCell(motion_flags_) << ','
         << local_cloud_.size() << ','
         << evidence_grid_.voxelCount() << ','
         << evidence_grid_.countState(VoxelState::FREE, now) << ','
         << evidence_grid_.countState(VoxelState::OCCUPIED, now) << ','
         << ageSeconds(last_odom_time_) << ','
         << ageSeconds(last_health_time_) << ','
         << ageSeconds(last_cloud_time_) << ','
         << (cloud_frame_ok_ ? 1 : 0) << ','
         << csvCell(last_cloud_frame_) << ','
         << last_retreat_base_yaw_ * 180.0 / kPi << ','
         << csvCell(last_retreat_uses_obstacle_ ? "nearest_obstacle" : "body_backward") << ','
         << "pilot_only\n";
    csv_.flush();
  }

  std::string kvValue(std::string value) const
  {
    for (char &ch : value)
    {
      if (ch == ';' || ch == '\r' || ch == '\n')
        ch = ',';
    }
    return value;
  }

  std::string csvCell(std::string value) const
  {
    bool quote = false;
    for (char ch : value)
    {
      if (ch == ',' || ch == '"' || ch == '\r' || ch == '\n')
      {
        quote = true;
        break;
      }
    }
    if (!quote)
      return value;
    std::string out = "\"";
    for (char ch : value)
    {
      if (ch == '"')
        out += "\"\"";
      else if (ch == '\r' || ch == '\n')
        out += ' ';
      else
        out += ch;
    }
    out += '"';
    return out;
  }

  double ageSeconds(const ros::Time &stamp) const
  {
    if (stamp.isZero())
      return -1.0;
    return (ros::Time::now() - stamp).toSec();
  }

  void maybePrintStatus(const ros::Time &now, const std::string &state, const std::string &mode,
                        const std::string &prompt, std::size_t count, double score)
  {
    if (!last_print_time_.isZero() && (now - last_print_time_).toSec() < status_print_period_)
      return;
    last_print_time_ = now;
    ROS_INFO("[assist_planner] state=%s mode=%s candidates=%zu score=%.3f prompt=%s",
             state.c_str(), mode.c_str(), count, score, prompt.c_str());
  }

  ros::Subscriber sub_odom_, sub_cloud_, sub_health_, sub_intent_;
  ros::Publisher pub_path_, pub_goal_, pub_candidates_, pub_markers_, pub_status_;
  ros::Timer timer_;

  std::string odom_topic_, cloud_topic_, health_advice_topic_, pilot_intent_topic_, world_frame_;

  double odom_timeout_s_{1.0}, health_timeout_s_{3.0};
  double local_radius_m_{15.0}, local_z_radius_m_{5.0};
  double cloud_timeout_s_{1.0};
  bool require_fresh_cloud_{true};
  bool suppress_on_frame_mismatch_{true};
  int max_cloud_points_{6000}, sector_count_{16}, target_sector_points_{180};
  int min_sector_points_for_strong_gain_{20};
  double update_rate_hz_{2.0};

  bool evidence_grid_enabled_{true};
  bool grid_free_raycast_enabled_{true};
  double grid_voxel_size_m_{0.5}, grid_free_ttl_s_{4.0}, grid_occupied_ttl_s_{20.0};
  double grid_max_ray_length_m_{12.0}, grid_ray_step_m_{0.5};
  int grid_free_ray_stride_{3}, grid_min_occupied_hits_{1}, grid_max_voxels_{40000};
  LocalEvidenceGrid evidence_grid_;

  double sample_radius_m_{3.0};
  int sample_count_{16};
  double min_goal_distance_m_{1.0}, max_goal_distance_m_{5.0};
  double min_obstacle_distance_m_{1.2}, max_yaw_change_rad_{1.2}, path_check_step_m_{0.35};
  std::vector<double> z_offsets_m_;

  double watch_sample_radius_scale_{0.7}, watch_obstacle_distance_scale_{1.3};
  double warning_retreat_distance_m_{2.0}, recovering_max_goal_distance_m_{1.5};
  bool critical_publish_hold_goal_{true};
  bool confidence_very_low_suppresses_{true}, confidence_low_forces_conservative_{true};

  std::string default_pilot_intent_{"FREE"};
  double pilot_intent_message_default_duration_s_{8.0};
  double intent_corridor_deg_{60.0}, orbit_intent_corridor_deg_{70.0}, vertical_intent_corridor_deg_{90.0};
  bool free_mode_publishes_suggestions_{false};

  double w_task_progress_{1.2};
  double w_coverage_{1.0}, w_collision_{1.5}, w_health_{2.0}, w_unknown_{2.0}, w_path_length_{0.3}, w_turn_{0.4};
  double repeat_sector_penalty_{0.1}, repeat_sector_penalty_cap_{0.9}, min_accept_score_{-0.25};
  double min_accept_score_explore_{0.05}, min_accept_score_conservative_{0.00};
  double min_accept_score_recover_{-0.05}, min_accept_score_retreat_{-0.10};
  double max_unknown_path_ratio_{0.30}, warning_max_unknown_path_ratio_{0.18}, recovering_max_unknown_path_ratio_{0.22};
  double min_standoff_m_{1.5}, ideal_standoff_m_{3.0}, max_standoff_m_{6.0}, coverage_view_fov_rad_{1.5708};
  double surface_exhausted_density_fallback_scale_{0.35};
  int frontier_target_voxels_{30};

  double marker_scale_{0.35}, status_print_period_{3.0};
  bool show_all_candidates_{true};
  bool enable_goal_commitment_{true};
  double min_goal_hold_s_{2.5}, max_goal_hold_s_{6.0};
  double switch_score_margin_{0.20}, switch_bearing_margin_deg_{18.0};
  bool csv_enabled_{true};
  std::string csv_path_{"/tmp/fast_livo2_assist.csv"};

  Point3 current_pos_;
  double current_yaw_{0.0};
  bool have_odom_{false}, have_cloud_{false}, have_health_{false};
  bool cloud_frame_ok_{true};
  ros::Time last_odom_time_, last_cloud_time_, last_health_time_;
  std::string last_cloud_frame_;
  std::vector<Point3> local_cloud_;

  bool have_committed_goal_{false};
  Candidate committed_candidate_;
  std::string committed_state_, committed_mode_, committed_intent_;
  ros::Time committed_since_;

  std::string health_state_{"STARTING"};
  std::string health_action_code_{"STARTING"};
  std::string health_primary_reason_;
  std::string motion_flags_{"NONE"};
  double reliability_score_{-1.0};
  double confidence_score_{-1.0};
  std::string confidence_level_{"UNKNOWN"};
  bool have_intent_msg_{false};
  std::string pilot_intent_{"FREE"};
  double pilot_intent_strength_{1.0}, pilot_intent_duration_s_{0.0}, latched_intent_yaw_{0.0};
  ros::Time last_intent_time_;
  double last_retreat_base_yaw_{0.0};
  bool last_retreat_uses_obstacle_{false};

  std::vector<int> sector_visits_;
  int last_selected_sector_{-1};
  ros::Time last_sector_visit_time_;
  ros::Time last_sector_decay_time_;
  ros::Time last_print_time_;
  ros::Time last_eval_time_;        // for ROS time-reset detection
  std::string last_planning_state_; // cached planning_state for status/CSV output
  std::ofstream csv_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_livo2_assist_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  AssistPlanner planner(nh, pnh);
  ros::spin();
  return 0;
}
