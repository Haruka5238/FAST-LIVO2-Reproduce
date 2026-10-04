#include <gtest/gtest.h>

#include <cstddef>

#include "voxel_map.h"

namespace
{
VoxelMapConfig testConfig()
{
  VoxelMapConfig config{};
  config.max_voxel_size_ = 0.5;
  config.max_layer_ = 1;
  config.max_iterations_ = 2;
  config.layer_init_num_ = {5, 5};
  config.max_points_num_ = 50;
  config.planner_threshold_ = 0.01;
  config.beam_err_ = 0.02;
  config.dept_err_ = 0.05;
  config.sigma_num_ = 3.0;
  config.is_pub_plane_map_ = false;
  config.sliding_thresh = 8.0;
  config.map_sliding_en = false;
  config.half_map_size = 100;
  return config;
}

std::vector<pointWithVar> unmatchedPoints(const int count, const double dynamic_score)
{
  std::vector<pointWithVar> points;
  points.reserve(count);
  for (int index = 0; index < count; ++index)
  {
    pointWithVar point;
    point.point_w = V3D(0.05 * index, 0.01 * (index % 3), 1.0);
    point.var = M3D::Identity() * 1e-4;
    point.body_var = point.var;
    point.range = 10.0;
    point.intensity = 0.0;
    point.map_density_score = 1.0;
    point.map_match_success = false;
    point.map_dynamic_score = dynamic_score;
    points.push_back(point);
  }
  return points;
}

std::vector<pointWithVar> localizedUnmatchedPoints(const double x_offset, const int count,
                                                   const double dynamic_score = 0.0)
{
  std::vector<pointWithVar> points = unmatchedPoints(count, dynamic_score);
  for (int index = 0; index < count; ++index)
  {
    points[index].point_w = V3D(x_offset + 0.001 * index, 0.01, 1.0);
  }
  return points;
}

void configureRecovery(VoxelMapManager &manager)
{
  manager.map_write_gate_en_ = true;
  manager.map_write_min_score_ = 0.35;
  manager.map_write_unmatched_risk_thresh_ = 0.45;
  manager.map_write_min_return_ = 0.20;
  manager.map_write_motion_risk_ = 0.80;
  manager.map_write_quality_risk_ = 0.0;
  manager.map_write_recovery_en_ = true;
  manager.map_write_recovery_trigger_frames_ = 8;
  manager.map_write_starvation_frames_ = 8;
  manager.map_write_recovery_budget_ratio_ = 0.05;
  manager.map_write_recovery_min_return_ = 0.54;
  manager.map_write_recovery_max_dynamic_ = 0.25;
  manager.map_write_recovery_max_risk_ = 0.85;
  manager.dynamic_object_filter_en_ = true;
  manager.dynamic_map_reject_score_ = 0.65;
  manager.dynamic_motion_gain_ = 0.50;
  manager.lidar_return_quality_en_ = true;
  manager.lidar_intensity_available_ = true;
  manager.lidar_intensity_ref_ = 80.0;
  manager.lidar_range_ref_ = 35.0;
  manager.lidar_range_ref_effective_ = 35.0;
}

std::vector<pointWithVar> geometricPoints(const std::vector<V3D> &positions)
{
  std::vector<pointWithVar> points;
  points.reserve(positions.size());
  for (const V3D &position : positions)
  {
    pointWithVar point;
    point.point_w = position;
    point.var = M3D::Identity() * 1e-4;
    points.push_back(point);
  }
  return points;
}
} // namespace

TEST(VoxelPlaneInitialization, RejectsRepeatedSpectrumWithoutOutOfBoundsAccess)
{
  VoxelOctoTree tree(1, 0, 3, 50, 0.01f);
  VoxelPlane plane;
  tree.init_plane(geometricPoints({V3D::Zero(), V3D::Zero(), V3D::Zero(), V3D::Zero()}), &plane);

  EXPECT_FALSE(plane.is_plane_);
  EXPECT_TRUE(plane.plane_var_.allFinite());
}

TEST(VoxelPlaneInitialization, RejectsCollinearPointsWithAmbiguousNormal)
{
  VoxelOctoTree tree(1, 0, 3, 50, 0.01f);
  VoxelPlane plane;
  tree.init_plane(geometricPoints({V3D(-2.0, 0.0, 0.0), V3D(-1.0, 0.0, 0.0),
                                   V3D(0.0, 0.0, 0.0), V3D(1.0, 0.0, 0.0),
                                   V3D(2.0, 0.0, 0.0)}), &plane);

  EXPECT_FALSE(plane.is_plane_);
  EXPECT_TRUE(plane.plane_var_.allFinite());
}

TEST(VoxelPlaneInitialization, AcceptsFiniteTwoDimensionalPlane)
{
  VoxelOctoTree tree(1, 0, 3, 50, 0.01f);
  VoxelPlane plane;
  tree.init_plane(geometricPoints({V3D(-1.0, -1.0, 0.0), V3D(-1.0, 1.0, 0.0),
                                   V3D(0.0, 0.0, 0.0), V3D(1.0, -1.0, 0.0),
                                   V3D(1.0, 1.0, 0.0)}), &plane);

  EXPECT_TRUE(plane.is_plane_);
  EXPECT_TRUE(plane.normal_.allFinite());
  EXPECT_TRUE(plane.plane_var_.allFinite());
  EXPECT_NEAR(std::abs(plane.normal_.z()), 1.0, 1e-9);
}

TEST(MapWriteRecovery, RequiresThreeConsistentFramesBeforePromotion)
{
  VoxelMapConfig config = testConfig();
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  VoxelMapManager manager(config, voxel_map);
  configureRecovery(manager);

  manager.UpdateVoxelMap(unmatchedPoints(100, 0.0));

  EXPECT_TRUE(manager.map_write_recovery_active_);
  EXPECT_TRUE(manager.map_write_formal_map_frozen_);
  EXPECT_EQ(manager.map_write_recovery_accept_num_, 0);
  EXPECT_EQ(manager.map_write_accept_num_, 0);
  EXPECT_EQ(manager.map_write_reject_num_, 100);
  EXPECT_FALSE(manager.map_write_candidate_voxels_.empty());
  EXPECT_TRUE(manager.voxel_map_.empty());

  manager.UpdateVoxelMap(unmatchedPoints(100, 0.0));

  EXPECT_TRUE(manager.map_write_recovery_active_);
  EXPECT_EQ(manager.map_write_recovery_accept_num_, 0);
  EXPECT_TRUE(manager.voxel_map_.empty());

  // The non-external starvation path also requires its accumulated candidate
  // score to recover before promotion.
  manager.map_write_motion_risk_ = 0.20;
  manager.UpdateVoxelMap(unmatchedPoints(100, 0.0));

  EXPECT_FALSE(manager.map_write_recovery_active_);
  EXPECT_FALSE(manager.map_write_formal_map_frozen_);
  EXPECT_EQ(manager.map_write_recovery_accept_num_, 5);
  EXPECT_EQ(manager.map_write_accept_num_, 5);
  EXPECT_EQ(manager.map_write_reject_num_, 95);
  EXPECT_DOUBLE_EQ(manager.map_write_accept_ratio_, 0.05);
  EXPECT_EQ(manager.map_write_candidate_promoted_voxels_, 1);
  EXPECT_TRUE(manager.map_write_candidate_voxels_.empty());
  EXPECT_FALSE(manager.voxel_map_.empty());
}

TEST(MapWriteRecovery, NeverOverridesDynamicConflictRejection)
{
  VoxelMapConfig config = testConfig();
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  VoxelMapManager manager(config, voxel_map);
  configureRecovery(manager);

  manager.UpdateVoxelMap(unmatchedPoints(100, 1.0));

  EXPECT_TRUE(manager.map_write_recovery_active_);
  EXPECT_TRUE(manager.map_write_formal_map_frozen_);
  EXPECT_EQ(manager.map_write_recovery_accept_num_, 0);
  EXPECT_EQ(manager.map_write_accept_num_, 0);
  EXPECT_EQ(manager.map_write_reject_num_, 100);
  EXPECT_EQ(manager.dynamic_reject_num_, 100);
  EXPECT_TRUE(manager.map_write_candidate_voxels_.empty());
  EXPECT_TRUE(manager.voxel_map_.empty());
}

TEST(MapWriteRecovery, InconsistentCandidateGeometryRestartsTemporalSupport)
{
  VoxelMapConfig config = testConfig();
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  VoxelMapManager manager(config, voxel_map);
  configureRecovery(manager);

  manager.UpdateVoxelMap(localizedUnmatchedPoints(0.01, 20));
  ASSERT_EQ(manager.map_write_candidate_voxels_.size(), 1u);
  EXPECT_EQ(manager.map_write_candidate_voxels_.begin()->second.support_frames, 1);

  // Both centroids are in voxel x=0, but their separation exceeds the
  // 0.75*voxel consistency radius and therefore resets the candidate.
  manager.UpdateVoxelMap(localizedUnmatchedPoints(0.46, 20));
  ASSERT_EQ(manager.map_write_candidate_voxels_.size(), 1u);
  EXPECT_EQ(manager.map_write_candidate_voxels_.begin()->second.support_frames, 1);
  EXPECT_GE(manager.map_write_candidate_expired_voxels_, 1);
  EXPECT_TRUE(manager.voxel_map_.empty());
}

TEST(MapWriteRecovery, RuntimeResetClearsQuarantineWithoutPromotion)
{
  VoxelMapConfig config = testConfig();
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  VoxelMapManager manager(config, voxel_map);
  configureRecovery(manager);

  manager.UpdateVoxelMap(localizedUnmatchedPoints(0.01, 20));
  ASSERT_FALSE(manager.map_write_candidate_voxels_.empty());

  manager.resetRuntimeState();

  EXPECT_TRUE(manager.map_write_candidate_voxels_.empty());
  EXPECT_EQ(manager.map_write_candidate_point_num_, 0);
  EXPECT_FALSE(manager.map_write_formal_map_frozen_);
  EXPECT_TRUE(manager.voxel_map_.empty());
}

TEST(MapWriteRecovery, ExternalEstimatorRecoveryUsesMotionNotMissingPoseQuality)
{
  VoxelMapConfig config = testConfig();
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  VoxelMapManager manager(config, voxel_map);
  configureRecovery(manager);
  manager.map_write_starvation_frames_ = 0;
  manager.map_write_external_freeze_ = true;
  // Above the ordinary one-frame unmatched-write limit (0.45), but below the
  // recovery limit (0.85).  Temporal validation must be the deciding evidence.
  manager.map_write_motion_risk_ = 0.70;
  manager.map_write_quality_risk_ = 1.0;

  std::vector<pointWithVar> points = localizedUnmatchedPoints(0.01, 20);
  for (pointWithVar &point : points) point.intensity = 80.0;
  manager.UpdateVoxelMap(points);
  EXPECT_TRUE(manager.map_write_formal_map_frozen_);
  EXPECT_TRUE(manager.voxel_map_.empty());
  manager.UpdateVoxelMap(points);
  EXPECT_TRUE(manager.voxel_map_.empty());
  manager.UpdateVoxelMap(points);

  EXPECT_TRUE(manager.map_write_external_freeze_);
  EXPECT_TRUE(manager.map_write_formal_map_frozen_);
  EXPECT_EQ(manager.map_write_candidate_promoted_voxels_, 1);
  EXPECT_FALSE(manager.voxel_map_.empty());
}

TEST(MapWriteMetadata, RefreshClearsRejectedLinearizationStateWithoutChangingFilterState)
{
  VoxelMapConfig config = testConfig();
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  VoxelMapManager manager(config, voxel_map);
  manager.state_.pos_end = V3D(1.0, 2.0, 3.0);
  manager.state_.cov = MD(DIM_STATE, DIM_STATE)::Identity() * 0.25;
  const StatesGroup state_before = manager.state_;

  std::vector<pointWithVar> points = unmatchedPoints(1, 0.0);
  points.front().normal = V3D(0.0, 0.0, 1.0);
  points.front().map_match_success = true;
  points.front().map_residual_norm = 9.0;
  points.front().map_plane_quality = 0.1;
  points.front().map_density_score = 0.2;
  points.front().map_dynamic_score = 0.8;

  manager.RefreshMapWriteMetadata(points);

  EXPECT_FALSE(points.front().map_match_success);
  EXPECT_DOUBLE_EQ(points.front().map_residual_norm, 0.0);
  EXPECT_DOUBLE_EQ(points.front().map_plane_quality, 1.0);
  EXPECT_DOUBLE_EQ(points.front().map_density_score, 1.0);
  EXPECT_DOUBLE_EQ(points.front().map_dynamic_score, 0.0);
  EXPECT_TRUE(points.front().normal.isZero());
  EXPECT_TRUE((manager.state_.pos_end - state_before.pos_end).isZero());
  EXPECT_TRUE(manager.state_.cov.isApprox(state_before.cov, 0.0));
}

TEST(StateEstimationAcceptance, RejectsFrameWithoutUsablePointToPlaneConstraint)
{
  VoxelMapConfig config = testConfig();
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  VoxelMapManager manager(config, voxel_map);
  manager.state_.pos_end = V3D(1.0, 2.0, 3.0);
  manager.state_.cov = MD(DIM_STATE, DIM_STATE)::Identity() * 0.25;
  StatesGroup propagated_state = manager.state_;

  PointType point;
  point.x = 2.0f;
  point.y = 0.5f;
  point.z = 1.0f;
  point.intensity = 20.0f;
  point.curvature = 50.0f;
  manager.feats_down_body_->push_back(point);

  manager.StateEstimation(propagated_state);

  EXPECT_FALSE(manager.state_estimation_valid_);
  EXPECT_EQ(manager.effct_feat_num_, 0);
  EXPECT_TRUE(manager.state_.pos_end.isApprox(propagated_state.pos_end, 0.0));
  EXPECT_TRUE(manager.state_.cov.isApprox(propagated_state.cov, 0.0));
  EXPECT_TRUE(manager.voxel_map_.empty());
}

TEST(VoxelMapTransform, ReusesOutputCapacityAndClearsPreviousPoints)
{
  VoxelMapConfig config = testConfig();
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  VoxelMapManager manager(config, voxel_map);
  manager.extR_.setIdentity();
  manager.extT_.setZero();

  PointCloudXYZI::Ptr input(new PointCloudXYZI());
  input->resize(3);
  input->points[0].x = 1.0f;
  input->points[0].y = 2.0f;
  input->points[0].z = 3.0f;
  input->points[0].intensity = 4.0f;
  input->points[1].x = -1.0f;
  input->points[2].y = -2.0f;

  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
  output->reserve(64);
  const std::size_t reserved_capacity = output->points.capacity();

  manager.TransformLidar(M3D::Identity(), V3D(10.0, 20.0, 30.0), input, output);
  ASSERT_EQ(output->size(), 3u);
  EXPECT_GE(output->points.capacity(), reserved_capacity);
  EXPECT_FLOAT_EQ(output->points[0].x, 11.0f);
  EXPECT_FLOAT_EQ(output->points[0].y, 22.0f);
  EXPECT_FLOAT_EQ(output->points[0].z, 33.0f);
  EXPECT_FLOAT_EQ(output->points[0].intensity, 4.0f);

  input->resize(1);
  input->points[0].x = 5.0f;
  input->points[0].y = 6.0f;
  input->points[0].z = 7.0f;
  manager.TransformLidar(M3D::Identity(), V3D::Zero(), input, output);

  ASSERT_EQ(output->size(), 1u);
  EXPECT_GE(output->points.capacity(), reserved_capacity);
  EXPECT_FLOAT_EQ(output->points[0].x, 5.0f);
  EXPECT_FLOAT_EQ(output->points[0].y, 6.0f);
  EXPECT_FLOAT_EQ(output->points[0].z, 7.0f);
}

int main(int argc, char **argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
