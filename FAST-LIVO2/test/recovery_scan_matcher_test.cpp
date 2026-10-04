#include <gtest/gtest.h>

#include "recovery_scan_matcher.h"

#include <Eigen/Geometry>

namespace
{
PointCloudXYZI::Ptr makeAsymmetricCloud()
{
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  cloud->reserve(240);
  for (int index = 0; index < 240; ++index)
  {
    const double u = static_cast<double>(index);
    PointType point;
    point.x = static_cast<float>(0.04 * u + 0.35 * std::sin(0.31 * u));
    point.y = static_cast<float>(0.7 * std::sin(0.17 * u) + 0.01 * (index % 11));
    point.z = static_cast<float>(0.5 * std::cos(0.23 * u) + 0.015 * (index % 7));
    point.intensity = static_cast<float>(index % 255);
    cloud->push_back(point);
  }
  return cloud;
}

PointCloudXYZI::Ptr transformCloud(const PointCloudXYZI::ConstPtr &input,
                                   const Eigen::Matrix4d &transform)
{
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  output->reserve(input->size());
  for (const PointType &point : input->points)
  {
    const Eigen::Vector3d transformed =
        transform.block<3, 3>(0, 0) * Eigen::Vector3d(point.x, point.y, point.z) +
        transform.block<3, 1>(0, 3);
    PointType result = point;
    result.x = static_cast<float>(transformed.x());
    result.y = static_cast<float>(transformed.y());
    result.z = static_cast<float>(transformed.z());
    output->push_back(result);
  }
  return output;
}
} // namespace

TEST(RecoveryScanMatcher, RecoversFiniteDirectionalRelativeConstraint)
{
  const PointCloudXYZI::Ptr previous = makeAsymmetricCloud();
  Eigen::Matrix4d current_to_previous = Eigen::Matrix4d::Identity();
  current_to_previous.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(0.04, Eigen::Vector3d(0.3, 0.7, 0.2).normalized()).toRotationMatrix();
  current_to_previous.block<3, 1>(0, 3) = Eigen::Vector3d(0.18, -0.06, 0.04);
  const PointCloudXYZI::Ptr current = transformCloud(previous, current_to_previous.inverse());

  RecoveryScanMatcher matcher;
  matcher.maximum_correspondence_distance = 0.6;
  matcher.maximum_rmse = 0.2;
  const RecoveryScanMatchResult result =
      matcher.align(current, previous, current_to_previous);

  ASSERT_TRUE(result.converged);
  EXPECT_GE(result.correspondences, matcher.minimum_correspondences);
  EXPECT_LT(result.rmse, 1e-3);
  EXPECT_LT((result.current_to_previous - current_to_previous).norm(), 1e-3);
  EXPECT_TRUE(result.covariance.allFinite());
  EXPECT_TRUE(result.direction_factors.allFinite());
  EXPECT_GE(result.direction_factors.minCoeff(), 0.0);
  EXPECT_LE(result.direction_factors.maxCoeff(), 1.0);
  Eigen::LLT<Eigen::Matrix<double, 6, 6>> covariance_llt(result.covariance);
  EXPECT_EQ(covariance_llt.info(), Eigen::Success);
}

TEST(RecoveryScanMatcher, RejectsMotionOutsidePhysicalGate)
{
  const PointCloudXYZI::Ptr previous = makeAsymmetricCloud();
  Eigen::Matrix4d current_to_previous = Eigen::Matrix4d::Identity();
  current_to_previous(0, 3) = 0.4;
  const PointCloudXYZI::Ptr current = transformCloud(previous, current_to_previous.inverse());

  RecoveryScanMatcher matcher;
  matcher.maximum_correspondence_distance = 0.8;
  matcher.maximum_relative_translation = 0.2;
  const RecoveryScanMatchResult result =
      matcher.align(current, previous, current_to_previous);

  EXPECT_FALSE(result.converged);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
