/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedPose3.h
 * @brief   Extended pose on SE_2(3) with Barrau ordering (R, v, p).
 *
 * Unlike NavState which stores (R, t, v) for backward compatibility,
 * ExtendedPose3 follows the Brossard/Barrau convention:
 *   T = [ R  v  p ]
 *       [ 0  1  0 ]
 *       [ 0  0  1 ]
 * with tangent ordering xi = [ theta(3), nu(3), rho(3) ]
 * (rotation, velocity-tangent, position-tangent).
 *
 * Used by CombinedImuFactor2 and related SE_2(3) preintegration.
 */

#pragma once

#include <gtsam/base/MatrixLieGroup.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

// Velocity type (alias for Vector3). Matches NavState/Gal3.
using Velocity3 = Vector3;

/**
 * ExtendedPose3: element of SE_2(3) with (R, v, p) ordering.
 *
 * Group product (Barrau):
 *   (R1, v1, p1) * (R2, v2, p2) = (R1 R2, v1 + R1 v2, p1 + R1 p2)
 *
 * Tangent vector xi = [theta, nu, rho] (dim 9).
 */
class GTSAM_EXPORT ExtendedPose3 : public MatrixLieGroup<ExtendedPose3, 9, 5> {
 private:
  Rot3 R_;       ///< Rotation nRb: body-to-nav
  Velocity3 v_;  ///< Velocity in nav frame
  Point3 p_;     ///< Position in nav frame

 public:
  using LieAlgebra = Matrix5;
  using Vector25 = Eigen::Matrix<double, 25, 1>;

  /// @name Constructors
  /// @{

  /// Default constructor: identity.
  ExtendedPose3() : v_(Vector3::Zero()), p_(0, 0, 0) {}

  /// Construct from rotation, velocity, position.
  ExtendedPose3(const Rot3& R, const Velocity3& v, const Point3& p)
      : R_(R), v_(v), p_(p) {}

  /// Construct from Pose3 and velocity.
  ExtendedPose3(const Pose3& pose, const Velocity3& v)
      : R_(pose.rotation()), v_(v), p_(pose.translation()) {}

  /// Construct from a 5x5 homogeneous matrix.
  explicit ExtendedPose3(const Matrix5& T)
      : R_(T.block<3, 3>(0, 0)),
        v_(T.block<3, 1>(0, 3)),
        p_(T.block<3, 1>(0, 4)) {}

  /// Named constructor with derivatives.
  static ExtendedPose3 Create(const Rot3& R, const Velocity3& v, const Point3& p,
                              OptionalJacobian<9, 3> H1 = {},
                              OptionalJacobian<9, 3> H2 = {},
                              OptionalJacobian<9, 3> H3 = {});

  /// Named constructor from Pose3 + velocity with derivatives.
  static ExtendedPose3 FromPoseVelocity(const Pose3& pose, const Vector3& v,
                                        OptionalJacobian<9, 6> H1 = {},
                                        OptionalJacobian<9, 3> H2 = {});

  /// @}
  /// @name Component Access
  /// @{

  const Rot3& attitude(OptionalJacobian<3, 9> H = {}) const;
  const Velocity3& velocity(OptionalJacobian<3, 9> H = {}) const;
  const Point3& position(OptionalJacobian<3, 9> H = {}) const;

  const Rot3& rotation() const { return R_; }

  /// Alias for position(), matching Pose3's accessor name so that generic
  /// factor code templated on Pose works with ExtendedPose3.
  const Point3& translation() const { return p_; }

  Pose3 pose() const { return Pose3(R_, p_); }

  /// @}
  /// @name Derived quantities
  /// @{

  Matrix3 R() const { return R_.matrix(); }
  const Vector3& v() const { return v_; }
  Vector3 p() const { return p_; }

  /// Velocity in body frame
  Velocity3 bodyVelocity(OptionalJacobian<3, 9> H = {}) const;

  /// Matrix5 representation: T = [R v p; 0 1 0; 0 0 1]
  Matrix5 matrix() const;

  Vector25 vec(OptionalJacobian<25, 9> H = {}) const;

  /// @}
  /// @name Testable
  /// @{

  GTSAM_EXPORT
  friend std::ostream& operator<<(std::ostream& os, const ExtendedPose3& X);

  void print(const std::string& s = "") const;

  bool equals(const ExtendedPose3& other, double tol = 1e-8) const;

  /// @}
  /// @name Group
  /// @{

  static ExtendedPose3 Identity() { return ExtendedPose3(); }

  ExtendedPose3 inverse() const;

  using LieGroup<ExtendedPose3, 9>::inverse;

  /// SE_2(3) group product: (R1 R2, v1 + R1 v2, p1 + R1 p2)
  ExtendedPose3 operator*(const ExtendedPose3& X) const {
    return ExtendedPose3(R_ * X.R_, v_ + R_ * X.v_, p_ + R_ * X.p_);
  }

  // Tangent sugar: xi = [theta(3), nu(3), rho(3)]
  static Eigen::Block<Vector9, 3, 1> dR(Vector9& v) {
    return v.segment<3>(0);
  }
  static Eigen::Block<Vector9, 3, 1> dV(Vector9& v) {
    return v.segment<3>(3);
  }
  static Eigen::Block<Vector9, 3, 1> dP(Vector9& v) {
    return v.segment<3>(6);
  }
  static Eigen::Block<const Vector9, 3, 1> dR(const Vector9& v) {
    return v.segment<3>(0);
  }
  static Eigen::Block<const Vector9, 3, 1> dV(const Vector9& v) {
    return v.segment<3>(3);
  }
  static Eigen::Block<const Vector9, 3, 1> dP(const Vector9& v) {
    return v.segment<3>(6);
  }

  /// @}
  /// @name Lie Group
  /// @{

  /// Exponential map at identity (Barrau/Brossard, left-trivialized).
  static ExtendedPose3 Expmap(const Vector9& xi,
                              OptionalJacobian<9, 9> Hxi = {});

  /// Logarithmic map at identity.
  static Vector9 Logmap(const ExtendedPose3& X,
                        OptionalJacobian<9, 9> HX = {});

  /// AdjointMap: Ad_g for SE_2(3).
  Matrix9 AdjointMap() const;

  /// Adjoint action on a tangent vector.
  Vector9 Adjoint(const Vector9& xi_b, OptionalJacobian<9, 9> H_this = {},
                  OptionalJacobian<9, 9> H_xib = {}) const;

  /// ad(xi) matrix.
  static Matrix9 adjointMap(const Vector9& xi);

  /// Action of the adjoint map on a tangent vector.
  static Vector9 adjoint(const Vector9& xi, const Vector9& y,
                         OptionalJacobian<9, 9> Hxi = {},
                         OptionalJacobian<9, 9> Hy = {});

  /// Derivative of Expmap (right Jacobian of SE_2(3)).
  static Matrix9 ExpmapDerivative(const Vector9& xi);

  /// Derivative of Logmap.
  static Matrix9 LogmapDerivative(const Vector9& xi);
  static Matrix9 LogmapDerivative(const ExtendedPose3& X);

  /// Chart at origin.
  struct GTSAM_EXPORT ChartAtOrigin {
    static ExtendedPose3 Retract(const Vector9& xi, ChartJacobian Hxi = {});
    static Vector9 Local(const ExtendedPose3& X, ChartJacobian HX = {});
  };

  /// Hat: xi -> 5x5 Lie algebra element.
  static Matrix5 Hat(const Vector9& xi);

  /// Vee: 5x5 Lie algebra element -> xi.
  static Vector9 Vee(const Matrix5& X);

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(R_);
    ar& BOOST_SERIALIZATION_NVP(v_);
    ar& BOOST_SERIALIZATION_NVP(p_);
  }
#endif
};

// Traits
template <>
struct traits<ExtendedPose3>
    : public internal::MatrixLieGroup<ExtendedPose3, 5> {};

template <>
struct traits<const ExtendedPose3>
    : public internal::MatrixLieGroup<ExtendedPose3, 5> {};

}  // namespace gtsam
