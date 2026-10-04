#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cmath>

#include "utils/so3_math.h"

namespace
{
Eigen::Matrix3d rotationExp(const Eigen::Vector3d &angle)
{
  return Exp(Eigen::Vector3d(angle));
}
}

TEST(SO3CovarianceReset, RightJacobianMatchesInjectionFiniteDifference)
{
  const Eigen::Vector3d delta(0.31, -0.22, 0.14);
  const Eigen::Matrix3d expected = RightJacobianSO3(delta);
  const Eigen::Matrix3d inverse_injection = rotationExp(-delta);
  Eigen::Matrix3d numerical;
  const double epsilon = 1e-7;

  for (int axis = 0; axis < 3; ++axis)
  {
    Eigen::Vector3d perturbation = Eigen::Vector3d::Zero();
    perturbation(axis) = epsilon;
    const Eigen::Matrix3d relative = inverse_injection * rotationExp(delta + perturbation);
    numerical.col(axis) = Log(Eigen::Matrix3d(relative)) / epsilon;
  }

  EXPECT_LT((numerical - expected).norm(), 2e-7);
}

TEST(SO3CovarianceReset, SmallAngleSeriesHasRightPerturbationSign)
{
  const Eigen::Vector3d delta(2e-7, -3e-7, 4e-7);
  Eigen::Matrix3d delta_hat;
  delta_hat << SKEW_SYM_MATRX(delta);
  const Eigen::Matrix3d first_order = Eigen::Matrix3d::Identity() - 0.5 * delta_hat;

  EXPECT_TRUE(RightJacobianSO3(Eigen::Vector3d::Zero()).isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_LT((RightJacobianSO3(delta) - first_order).norm(), 1e-12);
}

TEST(SO3CovarianceReset, IterationTransportInvertsPriorResidualJacobian)
{
  const Eigen::Vector3d delta(0.27, -0.18, 0.11);
  const Eigen::Matrix3d current_from_propagated = rotationExp(delta);
  Eigen::Matrix3d prior_residual_jacobian;
  const double epsilon = 1e-7;

  for (int axis = 0; axis < 3; ++axis)
  {
    Eigen::Vector3d perturbation = Eigen::Vector3d::Zero();
    perturbation(axis) = epsilon;
    const Eigen::Vector3d residual_plus =
        Log(Eigen::Matrix3d(current_from_propagated * rotationExp(perturbation)));
    const Eigen::Vector3d residual_minus =
        Log(Eigen::Matrix3d(current_from_propagated * rotationExp(-perturbation)));
    prior_residual_jacobian.col(axis) = (residual_plus - residual_minus) / (2.0 * epsilon);
  }

  EXPECT_LT((prior_residual_jacobian * RightJacobianSO3(delta) -
             Eigen::Matrix3d::Identity()).norm(), 3e-7);
}

TEST(SO3CovarianceReset, ExponentialMapKeepsSubThresholdIncrements)
{
  const Eigen::Vector3d delta(2e-7, -3e-7, 4e-7);
  Eigen::Matrix3d delta_hat;
  delta_hat << SKEW_SYM_MATRX(delta);
  const Eigen::Matrix3d first_order = Eigen::Matrix3d::Identity() + delta_hat;

  EXPECT_LT((rotationExp(delta) - first_order).norm(), 3e-13);
  EXPECT_GT((rotationExp(delta) - Eigen::Matrix3d::Identity()).norm(), 1e-8);
}

TEST(SO3CovarianceReset, TransportsFullCrossCovarianceAndPreservesSPD)
{
  constexpr int state_dimension = 19;
  Eigen::Matrix<double, state_dimension, state_dimension> seed;
  for (int row = 0; row < state_dimension; ++row)
  {
    for (int col = 0; col < state_dimension; ++col)
    {
      seed(row, col) = std::sin(0.17 * (row + 1) * (col + 2));
    }
  }
  const auto covariance = seed * seed.transpose() +
                          0.1 * Eigen::Matrix<double, state_dimension, state_dimension>::Identity();
  const Eigen::Matrix3d rotation_reset = RightJacobianSO3(Eigen::Vector3d(0.4, -0.3, 0.2));
  Eigen::Matrix<double, state_dimension, state_dimension> reset =
      Eigen::Matrix<double, state_dimension, state_dimension>::Identity();
  reset.block<3, 3>(0, 0) = rotation_reset;
  const auto transported = (reset * covariance * reset.transpose()).eval();

  EXPECT_LT((transported - transported.transpose()).norm(), 1e-12);
  EXPECT_TRUE((transported.block<3, state_dimension - 3>(0, 3).isApprox(
      rotation_reset * covariance.block<3, state_dimension - 3>(0, 3), 1e-12)));
  Eigen::LLT<Eigen::Matrix<double, state_dimension, state_dimension>> llt(transported);
  EXPECT_EQ(llt.info(), Eigen::Success);
}

#ifdef X86_ARCH
TEST(EigenPclAbi, UsesDistributionCompatibleAlignmentContract)
{
  EXPECT_EQ(EIGEN_MAX_ALIGN_BYTES, 16);
  EXPECT_EQ(EIGEN_MAX_STATIC_ALIGN_BYTES, 16);
  EXPECT_EQ(EIGEN_MALLOC_ALREADY_ALIGNED, 1);
}
#endif

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
