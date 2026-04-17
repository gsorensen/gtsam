/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedPose3.cpp
 * @brief   SE_2(3) extended pose (R, v, p) — Barrau convention.
 */

#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Kernel.h>
#include <gtsam/geometry/SO3.h>

#include <string>

namespace gtsam {

//------------------------------------------------------------------------------
ExtendedPose3 ExtendedPose3::Create(const Rot3& R, const Velocity3& v,
                                    const Point3& p,
                                    OptionalJacobian<9, 3> H1,
                                    OptionalJacobian<9, 3> H2,
                                    OptionalJacobian<9, 3> H3) {
  Matrix3 Rt;
  if (H2 || H3) Rt = R.transpose();
  if (H1) *H1 << I_3x3, Z_3x3, Z_3x3;
  if (H2) *H2 << Z_3x3, Rt, Z_3x3;
  if (H3) *H3 << Z_3x3, Z_3x3, Rt;
  return ExtendedPose3(R, v, p);
}

//------------------------------------------------------------------------------
ExtendedPose3 ExtendedPose3::FromPoseVelocity(const Pose3& pose,
                                              const Vector3& vel,
                                              OptionalJacobian<9, 6> H1,
                                              OptionalJacobian<9, 3> H2) {
  // xi = [theta, nu, rho], Pose3 tangent = [theta, rho]
  // drotation/d(Pose3 tangent) = [I, 0]; dposition/d(Pose3 tangent) = [0, I]
  if (H1) *H1 << I_3x3, Z_3x3, Z_3x3, Z_3x3, Z_3x3, I_3x3;
  if (H2) *H2 << Z_3x3, pose.rotation().transpose(), Z_3x3;
  return ExtendedPose3(pose, vel);
}

//------------------------------------------------------------------------------
const Rot3& ExtendedPose3::attitude(OptionalJacobian<3, 9> H) const {
  if (H) *H << I_3x3, Z_3x3, Z_3x3;
  return R_;
}

//------------------------------------------------------------------------------
const Velocity3& ExtendedPose3::velocity(OptionalJacobian<3, 9> H) const {
  if (H) *H << Z_3x3, R(), Z_3x3;
  return v_;
}

//------------------------------------------------------------------------------
const Point3& ExtendedPose3::position(OptionalJacobian<3, 9> H) const {
  if (H) *H << Z_3x3, Z_3x3, R();
  return p_;
}

//------------------------------------------------------------------------------
Vector3 ExtendedPose3::bodyVelocity(OptionalJacobian<3, 9> H) const {
  const Rot3& nRb = R_;
  Matrix3 D_bv_nRb;
  Vector3 b_v = nRb.unrotate(v_, H ? &D_bv_nRb : 0);
  if (H) *H << D_bv_nRb, I_3x3, Z_3x3;
  return b_v;
}

//------------------------------------------------------------------------------
Matrix5 ExtendedPose3::matrix() const {
  Matrix5 T = Matrix5::Identity();
  T.block<3, 3>(0, 0) = R_.matrix();
  T.block<3, 1>(0, 3) = v_;
  T.block<3, 1>(0, 4) = p_;
  return T;
}

//------------------------------------------------------------------------------
ExtendedPose3::Vector25 ExtendedPose3::vec(OptionalJacobian<25, 9> H) const {
  const Matrix5 T = this->matrix();
  if (H) {
    H->setZero();
    auto Rm = T.block<3, 3>(0, 0);
    // Derivatives of the three rotation columns wrt theta (rows 0-2, 5-7, 10-12)
    H->block<3, 1>(0, 1) = -Rm.col(2);
    H->block<3, 1>(0, 2) = Rm.col(1);
    H->block<3, 1>(5, 0) = Rm.col(2);
    H->block<3, 1>(5, 2) = -Rm.col(0);
    H->block<3, 1>(10, 0) = -Rm.col(1);
    H->block<3, 1>(10, 1) = Rm.col(0);
    // Velocity column (col 3 of T, rows 15-17 of vec) wrt nu (tangent block 3-5)
    H->block<3, 3>(15, 3) = Rm;
    // Position column (col 4 of T, rows 20-22 of vec) wrt rho (tangent block 6-8)
    H->block<3, 3>(20, 6) = Rm;
  }
  return Eigen::Map<const Vector25>(T.data());
}

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const ExtendedPose3& X) {
  os << "R: " << X.attitude() << "\n";
  os << "v: " << X.velocity().transpose() << "\n";
  os << "p: " << X.position().transpose();
  return os;
}

//------------------------------------------------------------------------------
void ExtendedPose3::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

//------------------------------------------------------------------------------
bool ExtendedPose3::equals(const ExtendedPose3& other, double tol) const {
  return R_.equals(other.R_, tol) &&
         equal_with_abs_tol(v_, other.v_, tol) &&
         traits<Point3>::Equals(p_, other.p_, tol);
}

//------------------------------------------------------------------------------
ExtendedPose3 ExtendedPose3::inverse() const {
  const Rot3 Rt = R_.inverse();
  return ExtendedPose3(Rt, Rt * (-v_), Rt * (-p_));
}

//------------------------------------------------------------------------------
// Expmap for SE_2(3), left-trivialized (Barrau/Brossard convention).
//   Expmap([theta, nu, rho]) = ( Expmap(theta), J_l(theta)*nu, J_l(theta)*rho )
// Hxi returns the *right* Jacobian (consistent with GTSAM's retract convention
// X.retract(v) = X * Expmap(v)).
ExtendedPose3 ExtendedPose3::Expmap(const Vector9& xi,
                                    OptionalJacobian<9, 9> Hxi) {
  const Vector3 w = xi.head<3>();
  const Vector3 nu = xi.segment<3>(3);
  const Vector3 rho = xi.tail<3>();

  const so3::DexpFunctor local(w);

#ifdef GTSAM_USE_QUATERNIONS
  const Rot3 R = traits<gtsam::Quaternion>::Expmap(w);
#else
  const Rot3 R(local.expmap());
#endif

  Matrix3 H_v_w, H_p_w;
  const Vector3 v = local.Jacobian().applyLeft(nu, Hxi ? &H_v_w : nullptr);
  const Vector3 p = local.Jacobian().applyLeft(rho, Hxi ? &H_p_w : nullptr);

  if (Hxi) {
    // Right Jacobian of SO(3). In GTSAM's convention,
    //   Expmap(w + dw) ~ Expmap(w) * Expmap(Jr * dw)
    const Matrix3 Jr = local.Jacobian().right();
    const Matrix3 Rt = R.transpose();
    // Rows 0-2 = d/d theta; rows 3-5 = d/d nu; rows 6-8 = d/d rho.
    *Hxi <<
        Jr, Z_3x3, Z_3x3,
        Rt * H_v_w, Jr, Z_3x3,
        Rt * H_p_w, Z_3x3, Jr;
  }

  return ExtendedPose3(R, v, p);
}

//------------------------------------------------------------------------------
Vector9 ExtendedPose3::Logmap(const ExtendedPose3& X,
                              OptionalJacobian<9, 9> HX) {
  if (HX) *HX = LogmapDerivative(X);

  const Vector3 phi = Rot3::Logmap(X.rotation());
  const Vector3& v = X.velocity();
  const Vector3& p = X.position();
  const double t = phi.norm();

  if (t < 1e-8) {
    Vector9 log;
    log << phi, v, p;
    return log;
  }

  // Apply J_l^{-1}(phi) to both nu and rho components.
  // Using the same series as NavState::Logmap (Barfoot / Barrau convention).
  const Matrix3 W = skewSymmetric(phi / t);
  const double Tan = std::tan(0.5 * t);
  const Vector3 Wv = W * v;
  const Vector3 Wp = W * p;
  const Vector3 nu =
      v - (0.5 * t) * Wv + (1 - t / (2.0 * Tan)) * (W * Wv);
  const Vector3 rho =
      p - (0.5 * t) * Wp + (1 - t / (2.0 * Tan)) * (W * Wp);

  Vector9 log;
  log << phi, nu, rho;
  return log;
}

//------------------------------------------------------------------------------
Matrix9 ExtendedPose3::AdjointMap() const {
  // Barrau: Ad_g for SE_2(3) with (R, v, p) ordering and tangent [theta, nu, rho]
  //   Ad = [  R     0    0
  //          v^R    R    0
  //          p^R    0    R ]
  const Matrix3 Rm = R_.matrix();
  const Matrix3 A = skewSymmetric(v_) * Rm;
  const Matrix3 B = skewSymmetric(p_) * Rm;
  Matrix9 adj;
  adj << Rm, Z_3x3, Z_3x3,
         A, Rm, Z_3x3,
         B, Z_3x3, Rm;
  return adj;
}

//------------------------------------------------------------------------------
Vector9 ExtendedPose3::Adjoint(const Vector9& xi_b,
                               OptionalJacobian<9, 9> H_this,
                               OptionalJacobian<9, 9> H_xib) const {
  const Matrix9 Ad = AdjointMap();
  if (H_this) *H_this = -Ad * adjointMap(xi_b);
  if (H_xib) *H_xib = Ad;
  return Ad * xi_b;
}

//------------------------------------------------------------------------------
Matrix9 ExtendedPose3::adjointMap(const Vector9& xi) {
  const Matrix3 w_hat = skewSymmetric(xi(0), xi(1), xi(2));
  const Matrix3 nu_hat = skewSymmetric(xi(3), xi(4), xi(5));
  const Matrix3 rho_hat = skewSymmetric(xi(6), xi(7), xi(8));
  Matrix9 ad;
  ad << w_hat, Z_3x3, Z_3x3,
        nu_hat, w_hat, Z_3x3,
        rho_hat, Z_3x3, w_hat;
  return ad;
}

//------------------------------------------------------------------------------
Vector9 ExtendedPose3::adjoint(const Vector9& xi, const Vector9& y,
                               OptionalJacobian<9, 9> Hxi,
                               OptionalJacobian<9, 9> Hy) {
  if (Hxi) {
    Hxi->setZero();
    for (int i = 0; i < 9; ++i) {
      Vector9 dxi = Vector9::Zero();
      dxi(i) = 1.0;
      Hxi->col(i) = adjointMap(dxi) * y;
    }
  }
  const Matrix9 ad_xi = adjointMap(xi);
  if (Hy) *Hy = ad_xi;
  return ad_xi * y;
}

//------------------------------------------------------------------------------
Matrix9 ExtendedPose3::ExpmapDerivative(const Vector9& xi) {
  Matrix9 J;
  Expmap(xi, J);
  return J;
}

//------------------------------------------------------------------------------
Matrix9 ExtendedPose3::LogmapDerivative(const Vector9& xi) {
  const Vector3 w = xi.head<3>();
  const Vector3 nu = xi.segment<3>(3);
  const Vector3 rho = xi.tail<3>();

  const so3::DexpFunctor local(w);
  Matrix3 H_v_w, H_p_w;
  local.Jacobian().applyLeft(nu, H_v_w);
  local.Jacobian().applyLeft(rho, H_p_w);

  const Matrix3 Rt = local.expmap().transpose();
  const Matrix3 Qv = Rt * H_v_w;
  const Matrix3 Qp = Rt * H_p_w;

  const Matrix3 Jw = Rot3::LogmapDerivative(w);
  const Matrix3 Qv2 = -Jw * Qv * Jw;
  const Matrix3 Qp2 = -Jw * Qp * Jw;

  Matrix9 J;
  J << Jw, Z_3x3, Z_3x3,
       Qv2, Jw, Z_3x3,
       Qp2, Z_3x3, Jw;
  return J;
}

//------------------------------------------------------------------------------
Matrix9 ExtendedPose3::LogmapDerivative(const ExtendedPose3& X) {
  return LogmapDerivative(Logmap(X));
}

//------------------------------------------------------------------------------
Matrix5 ExtendedPose3::Hat(const Vector9& xi) {
  const double wx = xi(0), wy = xi(1), wz = xi(2);
  const double vx = xi(3), vy = xi(4), vz = xi(5);
  const double px = xi(6), py = xi(7), pz = xi(8);
  Matrix5 X;
  X << 0., -wz, wy, vx, px,
       wz, 0., -wx, vy, py,
       -wy, wx, 0., vz, pz,
       0., 0., 0., 0., 0.,
       0., 0., 0., 0., 0.;
  return X;
}

//------------------------------------------------------------------------------
Vector9 ExtendedPose3::Vee(const Matrix5& X) {
  Vector9 xi;
  xi << X(2, 1), X(0, 2), X(1, 0),
        X(0, 3), X(1, 3), X(2, 3),
        X(0, 4), X(1, 4), X(2, 4);
  return xi;
}

//------------------------------------------------------------------------------
ExtendedPose3 ExtendedPose3::ChartAtOrigin::Retract(const Vector9& xi,
                                                   ChartJacobian Hxi) {
  return Expmap(xi, Hxi);
}

//------------------------------------------------------------------------------
Vector9 ExtendedPose3::ChartAtOrigin::Local(const ExtendedPose3& X,
                                            ChartJacobian HX) {
  return Logmap(X, HX);
}

}  // namespace gtsam
