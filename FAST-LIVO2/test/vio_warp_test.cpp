#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "vio.h"

static_assert(!std::is_copy_constructible<Feature>::value, "Feature owns its patch and must not be copied");
static_assert(!std::is_copy_assignable<Feature>::value, "Feature owns its patch and must not be copy-assigned");

namespace
{
void configureManager(VIOManager &manager)
{
  manager.patch_pyrimid_level = 1;
  manager.patch_size = 4;
  manager.patch_size_half = 2;
  manager.patch_size_total = 16;
}

VOXEL_POINTS *visualVoxel(const V3D &position)
{
  VOXEL_POINTS *voxel = new VOXEL_POINTS(0);
  voxel->voxel_points.push_back(new VisualPoint(position));
  return voxel;
}
} // namespace

TEST(VioWarp, AcceptsCompleteFiniteIdentityWarp)
{
  VIOManager manager;
  configureManager(manager);
  cv::Mat image(20, 20, CV_8UC1);
  for (int row = 0; row < image.rows; ++row)
  {
    for (int col = 0; col < image.cols; ++col)
    {
      image.at<uint8_t>(row, col) = static_cast<uint8_t>(row + col);
    }
  }
  std::vector<float> patch(manager.patch_size_total, -1.0f);

  EXPECT_TRUE(manager.warpAffine(Matrix2d::Identity(), image, V2D(10.0, 10.0), 0, 0, 0,
                                 manager.patch_size_half, patch.data()));
  EXPECT_TRUE(std::all_of(patch.begin(), patch.end(), [](float value) {
    return std::isfinite(value) && value >= 0.0f;
  }));
}

TEST(VioWarp, RejectsSingularAffineMatrix)
{
  VIOManager manager;
  configureManager(manager);
  cv::Mat image(20, 20, CV_8UC1, cv::Scalar(10));
  std::vector<float> patch(manager.patch_size_total, -1.0f);

  EXPECT_FALSE(manager.warpAffine(Matrix2d::Zero(), image, V2D(10.0, 10.0), 0, 0, 0,
                                  manager.patch_size_half, patch.data()));
}

TEST(VioWarp, RejectsPartiallyOutOfFramePatch)
{
  VIOManager manager;
  configureManager(manager);
  cv::Mat image(20, 20, CV_8UC1, cv::Scalar(10));
  std::vector<float> patch(manager.patch_size_total, -1.0f);

  EXPECT_FALSE(manager.warpAffine(Matrix2d::Identity(), image, V2D(0.5, 0.5), 0, 0, 0,
                                  manager.patch_size_half, patch.data()));
}

TEST(VisualRecovery, FreezesFormalMapAndPromotesCandidateWithoutOverlappingStaleVoxels)
{
  VIOManager manager;
  manager.visual_lifecycle_en = true;
  manager.visual_recovery_ever_valid = true;
  const VOXEL_LOCATION old_only(0, 0, 0);
  const VOXEL_LOCATION overlap(1, 0, 0);
  const VOXEL_LOCATION candidate_only(2, 0, 0);
  manager.feat_map.emplace(old_only, visualVoxel(V3D(0.1, 0.1, 0.1)));
  manager.feat_map.emplace(overlap, visualVoxel(V3D(0.6, 0.1, 0.1)));

  manager.beginVisualRecovery();

  EXPECT_EQ(manager.visual_recovery_state, VIOManager::VISUAL_RECOVERY_SEEDING);
  EXPECT_TRUE(manager.feat_map.empty());
  ASSERT_EQ(manager.frozen_feat_map.size(), 2u);

  manager.feat_map.emplace(overlap, visualVoxel(V3D(0.7, 0.1, 0.1)));
  manager.feat_map.emplace(candidate_only, visualVoxel(V3D(1.1, 0.1, 0.1)));
  manager.visual_recovery_state = VIOManager::VISUAL_RECOVERY_VALIDATING;
  manager.promoteVisualRecovery();

  EXPECT_EQ(manager.visual_recovery_state, VIOManager::VISUAL_RECOVERY_NORMAL);
  EXPECT_TRUE(manager.frozen_feat_map.empty());
  EXPECT_EQ(manager.feat_map.size(), 3u);
  EXPECT_EQ(manager.visual_recovery_success_count, 1);
  EXPECT_EQ(manager.recovered_visual_voxels.size(), 2u);
  EXPECT_NE(manager.recovered_visual_voxels.find(overlap), manager.recovered_visual_voxels.end());
  EXPECT_NE(manager.recovered_visual_voxels.find(candidate_only), manager.recovered_visual_voxels.end());
  EXPECT_EQ(manager.recovered_visual_voxels.find(old_only), manager.recovered_visual_voxels.end());
}

TEST(VisualRecovery, RejectsCandidateAndRestoresFrozenFormalMap)
{
  VIOManager manager;
  manager.visual_lifecycle_en = true;
  manager.visual_recovery_ever_valid = true;
  const VOXEL_LOCATION formal_key(0, 0, 0);
  const VOXEL_LOCATION candidate_key(1, 0, 0);
  manager.feat_map.emplace(formal_key, visualVoxel(V3D(0.1, 0.1, 0.1)));
  manager.beginVisualRecovery();
  manager.feat_map.emplace(candidate_key, visualVoxel(V3D(0.6, 0.1, 0.1)));

  manager.abortVisualRecovery();

  EXPECT_EQ(manager.visual_recovery_state, VIOManager::VISUAL_RECOVERY_COOLDOWN);
  EXPECT_EQ(manager.feat_map.size(), 1u);
  EXPECT_NE(manager.feat_map.find(formal_key), manager.feat_map.end());
  EXPECT_EQ(manager.feat_map.find(candidate_key), manager.feat_map.end());
  EXPECT_TRUE(manager.frozen_feat_map.empty());
  EXPECT_EQ(manager.visual_recovery_failure_count, 1);
  EXPECT_TRUE(manager.recovered_visual_voxels.empty());
}

TEST(VisualRecovery, RequiresHistoryStarvationAndThreeValidatedCandidateFrames)
{
  VIOManager manager;
  manager.visual_lifecycle_en = true;
  manager.visual_lifecycle_max_fail = 6;
  manager.cross_modal_min_track_points = 30;
  manager.visual_recovery_ever_valid = true;
  manager.visual_quality = 1.0;
  manager.lio_health_score = 1.0;
  manager.cross_modal_map_write_blocked = false;
  manager.temporal_map_write_blocked = false;
  manager.vio_covariance_numerical_failure = false;
  manager.feat_map.emplace(VOXEL_LOCATION(0, 0, 0), visualVoxel(V3D(0.1, 0.1, 0.1)));

  for (int frame = 0; frame < 2; ++frame)
  {
    manager.vio_update_valid = false;
    manager.total_points = 0;
    manager.updateVisualRecoveryState();
    EXPECT_EQ(manager.visual_recovery_state, VIOManager::VISUAL_RECOVERY_NORMAL);
  }
  manager.updateVisualRecoveryState();
  EXPECT_EQ(manager.visual_recovery_state, VIOManager::VISUAL_RECOVERY_SEEDING);

  manager.feat_map.emplace(VOXEL_LOCATION(1, 0, 0), visualVoxel(V3D(0.6, 0.1, 0.1)));
  manager.updateVisualRecoveryState();
  EXPECT_EQ(manager.visual_recovery_state, VIOManager::VISUAL_RECOVERY_VALIDATING);
  EXPECT_EQ(manager.visual_recovery_attempts, 1);

  manager.vio_update_valid = true;
  manager.total_points = 15;
  manager.updateVisualRecoveryState();
  manager.updateVisualRecoveryState();
  EXPECT_EQ(manager.visual_recovery_state, VIOManager::VISUAL_RECOVERY_VALIDATING);
  EXPECT_EQ(manager.visual_recovery_validation_streak, 2);
  manager.updateVisualRecoveryState();

  EXPECT_EQ(manager.visual_recovery_state, VIOManager::VISUAL_RECOVERY_NORMAL);
  EXPECT_EQ(manager.visual_recovery_success_count, 1);
  EXPECT_TRUE(manager.frozen_feat_map.empty());
}

TEST(DirectionalFusion, UsesVisualInformationToRecoverOnlyLidarWeakDirections)
{
  VIOManager manager;
  manager.directional_lio_vio_fusion_en = true;
  manager.lio_state_directions.setIdentity();
  manager.lio_direction_factors.setZero();
  manager.lio_direction_factors(0) = 1.0;

  Eigen::Matrix<double, 6, 6> prior = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> visual_information = Eigen::Matrix<double, 6, 6>::Zero();
  visual_information(1, 1) = 9.0;
  Eigen::Matrix<double, 6, 6> projection;

  ASSERT_TRUE(manager.computeDirectionalFusionProjection(prior, visual_information, projection));
  EXPECT_NEAR(manager.visual_direction_factors(0), 0.0, 1e-12);
  EXPECT_NEAR(manager.visual_direction_factors(1), 0.9, 1e-12);
  EXPECT_NEAR(manager.fused_direction_factors(0), 1.0, 1e-12);
  EXPECT_NEAR(manager.fused_direction_factors(1), 0.9, 1e-12);
  EXPECT_NEAR(projection(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(projection(1, 1), 0.9, 1e-12);
  EXPECT_NEAR(projection(2, 2), 0.0, 1e-12);
  EXPECT_TRUE(manager.directional_fusion_active);
}

TEST(DirectionalFusion, PreservesCoupledLidarDirectionsWithoutAxisSplitting)
{
  VIOManager manager;
  manager.directional_lio_vio_fusion_en = true;
  manager.lio_state_directions.setIdentity();
  manager.lio_state_directions.col(1) << 1.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  manager.lio_direction_factors << 0.2, 0.7, 0.4, 0.5, 0.6, 0.8;

  Eigen::Matrix<double, 6, 6> prior = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> projection;
  ASSERT_TRUE(manager.computeDirectionalFusionProjection(
      prior, Eigen::Matrix<double, 6, 6>::Zero(), projection));

  Eigen::Matrix<double, 6, 6> directions = manager.lio_state_directions;
  for (int direction = 0; direction < 6; ++direction) directions.col(direction).normalize();
  for (int direction = 0; direction < 6; ++direction)
  {
    const Eigen::Matrix<double, 6, 1> projected = projection * directions.col(direction);
    const Eigen::Matrix<double, 6, 1> expected =
        manager.lio_direction_factors(direction) * directions.col(direction);
    EXPECT_LT((projected - expected).norm(), 1e-10);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
