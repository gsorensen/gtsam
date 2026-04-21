/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file  ManifoldPreintegrationSE23.cpp
 *  @brief Implementation of SE_2(3) manifold preintegration.
 */

#include "ManifoldPreintegrationSE23.h"

#include <gtsam/geometry/Rot3.h>

#include <cmath>
#include <limits>
#include <type_traits>

using namespace std;

namespace gtsam {

namespace {

// ---------------------------------------------------------------------------
// Small SE_2(3) helpers, inlined here to keep ManifoldPreintegrationSE23
// self-contained inside gtsam (the canonical copies live in
// extensions/PreintegrationHelpers.{hpp,cpp} where they are unit-tested).
// ---------------------------------------------------------------------------

/// Inverse of the left Jacobian of SO(3).
inline Matrix3 so3_J_l_inv(const Vector3& theta) {
  const double theta_ = theta.norm();
  const double theta_sq = theta_ * theta_;
  const double twothetasintheta = 2.0 * theta_ * std::sin(theta_);
  const Matrix3 theta_x = skewSymmetric(theta);
  Matrix3 J_inv = I_3x3 - 0.5 * theta_x;
  if (std::numeric_limits<double>::epsilon() <= theta_sq &&
      std::numeric_limits<double>::epsilon() <= twothetasintheta) {
    const double a =
        (1.0 / theta_sq) - ((1.0 + std::cos(theta_)) / twothetasintheta);
    J_inv += a * theta_x * theta_x;
  }
  return J_inv;
}

/// SE_2(3) Logmap for a (R, v, p) 5x5 homogeneous matrix.
inline Vector9 se23_Logmap(const Matrix5& T) {
  const Rot3 R(Matrix3(T.block<3, 3>(0, 0)));
  const Vector3 v = T.block<3, 1>(0, 3);
  const Vector3 p = T.block<3, 1>(0, 4);
  const Vector3 logR = Rot3::Logmap(R);
  const Matrix3 J_inv = so3_J_l_inv(logR);
  Vector9 xi;
  xi << logR, J_inv * v, J_inv * p;
  return xi;
}

/// SE_2(3) Ypsilon_hat(w_hat, f_hat, dt) — Eq. 36 Brossard et al.
/// No longer used for state propagation (we compose intrinsically instead to
/// keep R on SO(3)); retained for reference / potential covariance checks.
[[maybe_unused]] inline Matrix5 se23_Ypsilon_hat(const Vector3& w_hat,
                                                 const Vector3& f_hat,
                                                 double dt) {
  const double dt22 = 0.5 * dt * dt;
  Matrix5 T = Matrix5::Identity();
  T.block<3, 3>(0, 0) = Rot3::Expmap(w_hat * dt).matrix();
  T.block<3, 1>(0, 3) = f_hat * dt;
  T.block<3, 1>(0, 4) = f_hat * dt22;
  return T;
}

/// Phi_t(T, dt) = (R, v, p + dt * v) — Eq. 25 Brossard et al. Applied to a
/// 5x5 homogeneous SE_2(3) matrix.
[[maybe_unused]] inline Matrix5 se23_Phi_t(const Matrix5& T, double dt) {
  Matrix5 Tp = T;
  Tp.block<3, 1>(0, 4) = T.block<3, 1>(0, 4) + dt * T.block<3, 1>(0, 3);
  return Tp;
}

/// SE_2(3) state-transition matrix F_dt — Eq. 33 Brossard et al.
inline Matrix9 se23_F_dt(double dt) {
  Matrix9 F = Matrix9::Identity();
  F.block<3, 3>(6, 3) = dt * I_3x3;
  return F;
}

/// SE_2(3) input Jacobian G_j(w_hat, f_hat, dt) — Eq. 38 Brossard.
inline Matrix96 se23_G_j(const Vector3& w_hat, const Vector3& /*f_hat*/,
                         double dt) {
  const double dt22 = 0.5 * dt * dt;
  const Matrix3 expminwhatdt = Rot3::Expmap(-w_hat * dt).matrix();
  Matrix96 G = Matrix96::Zero();
  G.block<3, 3>(0, 3) = -so3_J_l_inv(w_hat * dt) * dt;  // theta / gyro
  G.block<3, 3>(3, 0) = -expminwhatdt * dt;             // nu    / acc
  G.block<3, 3>(6, 0) = -expminwhatdt * dt22;           // rho   / acc
  return G;
}

}  // namespace

//------------------------------------------------------------------------------
template <typename Bias>
void ManifoldPreintegrationSE23<Bias>::resetIntegration() {
  deltaTij_ = 0.0;
  deltaXij_ = ExtendedPose3();
  delRdelBiasOmega_.setZero();
  delNUdelBiasAcc_.setZero();
  delNUdelBiasOmega_.setZero();
  delRHOdelBiasAcc_.setZero();
  delRHOdelBiasOmega_.setZero();
}

//------------------------------------------------------------------------------
template <typename Bias>
bool ManifoldPreintegrationSE23<Bias>::equals(
    const ManifoldPreintegrationSE23& other, double tol) const {
  return p_->equals(*other.p_, tol) &&
         std::abs(deltaTij_ - other.deltaTij_) < tol &&
         biasHat_.equals(other.biasHat_, tol) &&
         deltaXij_.equals(other.deltaXij_, tol) &&
         equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol) &&
         equal_with_abs_tol(delNUdelBiasAcc_, other.delNUdelBiasAcc_, tol) &&
         equal_with_abs_tol(delNUdelBiasOmega_, other.delNUdelBiasOmega_,
                            tol) &&
         equal_with_abs_tol(delRHOdelBiasAcc_, other.delRHOdelBiasAcc_, tol) &&
         equal_with_abs_tol(delRHOdelBiasOmega_, other.delRHOdelBiasOmega_,
                            tol);
}

//------------------------------------------------------------------------------
// update: advance the preintegrated state on SE_2(3) using the Brossard
// left-trivialized propagation
//
//   deltaXij_{k+1}  = deltaXij_k * Ypsilon_hat(w_hat, f_hat, dt)
//   A_k = F_dt(dt)                 (9x9 state-transition, linearized)
//   [B_k | C_k] = G_j(w_hat, f_hat, dt)   (9x3 | 9x3, acc then gyro cols)
//
// The bias Jacobians J_k (9x6, cols [acc, gyro], rows [theta, nu, rho])
// propagate as J_{k+1} = F_dt * J_k + G_j * diag(beta_acc, beta_gyro),
// where beta_* = exp(-t_k / tau_*) for the GaussMarkovBias model and 1 for
// ConstantBias.
//------------------------------------------------------------------------------
template <typename Bias>
void ManifoldPreintegrationSE23<Bias>::update(const Vector3& measuredAcc,
                                              const Vector3& measuredOmega,
                                              const double dt, Matrix9* A,
                                              Matrix93* B, Matrix93* C) {
  // 1. Correct bias to obtain (f_hat, w_hat).
  Vector3 acc, omega;
  if constexpr (std::is_same_v<Bias, imuBias::GaussMarkovBias>) {
    acc = biasHat_.correctAccelerometer(measuredAcc, deltaTij_);
    omega = biasHat_.correctGyroscope(measuredOmega, deltaTij_);
  } else {
    acc = biasHat_.correctAccelerometer(measuredAcc);
    omega = biasHat_.correctGyroscope(measuredOmega);
  }

  // 2. Possibly correct for sensor pose.
  Matrix3 D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega;
  if (p().body_P_sensor) {
    std::tie(acc, omega) = this->correctMeasurementsBySensorPose(
        acc, omega, D_correctedAcc_acc, D_correctedAcc_omega,
        D_correctedOmega_omega);
  }

  // 3. Advance group state on SE_2(3).
  //    Recursion Z_{k+1} = Phi_dt(Z_k) * Ypsilon_hat(w_hat, f_hat, dt) so that
  //    the full predict factors cleanly as
  //      X_j = Gamma_T(g) * Phi_T(X_i) * Z_j.
  //    F_dt (which has dt*I at the rho/nu block) is precisely the tangent-space
  //    counterpart of this recursion.
  //
  //    NOTE: we do NOT compose via a generic 5x5 matrix product + reslice —
  //    doing so lets the rotation block drift off SO(3) under accumulation,
  //    and ExtendedPose3(Matrix5) slices the 3x3 without re-orthogonalizing.
  //    After ~tens of seconds the non-orthogonality is large enough to flip
  //    the quaternion conversion branch and cause a visible attitude jump.
  //    Instead compose intrinsically: Rot3::Expmap keeps R on SO(3) exactly,
  //    and v/p get the algebraically equivalent updates that Phi_t * Ypsilon
  //    yields for an exact SO(3) rotation block.
  const Rot3 R_i = deltaXij_.rotation();
  const Vector3 v_i = deltaXij_.velocity();
  const Vector3 p_i = deltaXij_.position();
  const Vector3 Ra = R_i * acc;
  const Rot3 R_new = R_i * Rot3::Expmap(omega * dt);
  const Vector3 v_new = v_i + Ra * dt;
  const Vector3 p_new = p_i + v_i * dt + Ra * (0.5 * dt * dt);
  deltaXij_ = ExtendedPose3(R_new, v_new, p_new);

  // 4. Linearized transition + measurement Jacobians.
  const Matrix9 F = se23_F_dt(dt);
  const Matrix96 G = se23_G_j(omega, acc, dt);  // cols 0-2 acc, 3-5 gyro

  *A = F;
  *B = G.block<9, 3>(0, 0);  // d(xi_inc)/d(acc)
  *C = G.block<9, 3>(0, 3);  // d(xi_inc)/d(omega)

  if (p().body_P_sensor) {
    *C *= D_correctedOmega_omega;
    if (!p().body_P_sensor->translation().isZero()) {
      *C += *B * D_correctedAcc_omega;
    }
    *B *= D_correctedAcc_acc;  // must be last
  }

  // 5. Propagate bias Jacobians. For GaussMarkovBias, scale the per-step
  //    contribution by the effective decay factors evaluated at t_k.
  double beta_acc = 1.0, beta_omega = 1.0;
  if constexpr (std::is_same_v<Bias, imuBias::GaussMarkovBias>) {
    const double t_k = deltaTij_;  // pre-increment time
    beta_acc = std::exp(-t_k / biasHat_.tauAcc());
    beta_omega = std::exp(-t_k / biasHat_.tauGyro());
  }

  // Non-zero G_j blocks.
  const Matrix3 G_theta_gyro = G.block<3, 3>(0, 3);  // theta row, gyro col
  const Matrix3 G_nu_acc = G.block<3, 3>(3, 0);      // nu row,    acc col
  const Matrix3 G_rho_acc = G.block<3, 3>(6, 0);     // rho row,   acc col

  // F_dt couples only nu -> rho (block<3,3>(6,3) = dt*I). Apply that before
  // adding the G increment for the rho block.
  const Matrix3 delRHOdelBiasAcc_new =
      delRHOdelBiasAcc_ + dt * delNUdelBiasAcc_ + beta_acc * G_rho_acc;
  const Matrix3 delRHOdelBiasOmega_new =
      delRHOdelBiasOmega_ + dt * delNUdelBiasOmega_;  // no gyro->rho G block

  delRdelBiasOmega_ += beta_omega * G_theta_gyro;
  delNUdelBiasAcc_ += beta_acc * G_nu_acc;
  // delNUdelBiasOmega_ unchanged (no gyro->nu G block, no F coupling into nu)
  delRHOdelBiasAcc_ = delRHOdelBiasAcc_new;
  delRHOdelBiasOmega_ = delRHOdelBiasOmega_new;

  // 6. Bookkeeping.
  deltaTij_ += dt;
}

//------------------------------------------------------------------------------
// biasCorrectedDelta
//   xi = Log(deltaXij_) + J_bias * (bias_i - biasHat_)
// where J_bias is assembled from the tracked 3x3 blocks with row order
// [theta, nu, rho] and col order [acc, gyro].
//------------------------------------------------------------------------------
template <typename Bias>
Vector9 ManifoldPreintegrationSE23<Bias>::biasCorrectedDelta(
    const Bias& bias_i, OptionalJacobian<9, 6> H) const {
  const Bias biasIncr = bias_i - biasHat_;
  const Vector3 dba = biasIncr.accelerometer();
  const Vector3 dbg = biasIncr.gyroscope();

  // Base tangent from the preintegrated group element.
  const Vector9 xi0 = se23_Logmap(deltaXij_.matrix());

  Vector9 xi = xi0;
  xi.segment<3>(0) += delRdelBiasOmega_ * dbg;
  xi.segment<3>(3) += delNUdelBiasAcc_ * dba + delNUdelBiasOmega_ * dbg;
  xi.segment<3>(6) += delRHOdelBiasAcc_ * dba + delRHOdelBiasOmega_ * dbg;

  if (H) {
    H->setZero();
    // cols 0-2: d(xi)/d(bias_acc)
    H->block<3, 3>(3, 0) = delNUdelBiasAcc_;
    H->block<3, 3>(6, 0) = delRHOdelBiasAcc_;
    // cols 3-5: d(xi)/d(bias_gyro)
    H->block<3, 3>(0, 3) = delRdelBiasOmega_;
    H->block<3, 3>(3, 3) = delNUdelBiasOmega_;
    H->block<3, 3>(6, 3) = delRHOdelBiasOmega_;
  }
  return xi;
}

//------------------------------------------------------------------------------

}  // namespace gtsam

// Explicit instantiations.
template class gtsam::ManifoldPreintegrationSE23<gtsam::imuBias::ConstantBias>;
template class gtsam::ManifoldPreintegrationSE23<
    gtsam::imuBias::GaussMarkovBias>;
