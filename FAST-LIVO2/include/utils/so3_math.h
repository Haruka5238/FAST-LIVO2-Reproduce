#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <Eigen/Core>
#include <cmath>
#include <math.h>

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> Exp(const Eigen::MatrixBase<Derived> &ang)
{
  using T = typename Derived::Scalar;
  const Eigen::Matrix<T, 3, 1> angle = ang;
  const T theta_sq = angle.squaredNorm();
  const T theta_four = theta_sq * theta_sq;
  T sinc = T(0);
  T one_minus_cos_over_theta_sq = T(0);
  if (theta_sq < T(1e-8))
  {
    sinc = T(1) - theta_sq / T(6) + theta_four / T(120);
    one_minus_cos_over_theta_sq = T(0.5) - theta_sq / T(24) + theta_four / T(720);
  }
  else
  {
    const T theta = std::sqrt(theta_sq);
    sinc = std::sin(theta) / theta;
    one_minus_cos_over_theta_sq = (T(1) - std::cos(theta)) / theta_sq;
  }
  Eigen::Matrix<T, 3, 3> angle_hat;
  angle_hat << SKEW_SYM_MATRX(angle);
  return Eigen::Matrix<T, 3, 3>::Identity() + sinc * angle_hat +
         one_minus_cos_over_theta_sq * angle_hat * angle_hat;
}

template <typename T, typename Ts> Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
  const Eigen::Matrix<T, 3, 1> angle = ang_vel * static_cast<T>(dt);
  return Exp(angle);
}

template <typename T> Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
  const Eigen::Matrix<T, 3, 1> angle(v1, v2, v3);
  return Exp(angle);
}

// Right Jacobian for the local SO(3) perturbation R_new = R * Exp(delta).
// The series branch avoids cancellation in (1-cos(theta)) for small updates.
inline Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &delta)
{
  Eigen::Matrix3d delta_hat;
  delta_hat << SKEW_SYM_MATRX(delta);
  const double theta_sq = delta.squaredNorm();
  double first_coefficient = 0.0;
  double second_coefficient = 0.0;
  if (theta_sq < 1e-8)
  {
    const double theta_four = theta_sq * theta_sq;
    first_coefficient = 0.5 - theta_sq / 24.0 + theta_four / 720.0;
    second_coefficient = 1.0 / 6.0 - theta_sq / 120.0 + theta_four / 5040.0;
  }
  else
  {
    const double theta = std::sqrt(theta_sq);
    first_coefficient = (1.0 - std::cos(theta)) / theta_sq;
    second_coefficient = (theta - std::sin(theta)) / (theta_sq * theta);
  }
  return Eigen::Matrix3d::Identity() - first_coefficient * delta_hat +
         second_coefficient * delta_hat * delta_hat;
}

/* Logrithm of a Rotation Matrix */
template <typename T> Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
  T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T> Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
  T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;
  T x, y, z;
  if (!singular)
  {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  }
  else
  {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  return ang;
}

#endif
