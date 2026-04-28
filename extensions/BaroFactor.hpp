#pragma once

#include "JacobianTraits.hpp"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace parnav {

/// Barometric height factor with an additive bias state.
///
/// Measurement model converts a pressure sample (kPa) to height via the ICAO
/// standard atmosphere, then subtracts a fixed base height to obtain a
/// relative-height observation `z_rel`. The predicted height is
/// `-p_D + baro_bias` (NED convention), so the residual is
/// `(-p_D + baro_bias) - z_rel`.
///
/// Jacobian layout:
///   Pose3          (tangent order [theta, rho]):     H1 = [ 0_{1x3} | zvec' * R ]
///   ExtendedPose3  (tangent order [theta, nu, rho]): H1 = [ 0_{1x3} | 0_{1x3} | zvec' * R ]
/// where `zvec = (0, 0, -1)` and `R = pose.rotation().matrix()`.
/// H2 = 1 (scalar bias).
template <typename Pose>
class BaroFactor
    : public gtsam::NoiseModelFactorN<Pose, double> {
 private:
  using Base = gtsam::NoiseModelFactorN<Pose, double>;
  using Jacobian = Eigen::Matrix<double, 1, JacobianTraits<Pose>::Cols>;

  double base_height_;
  double height_rel_;  // z_rel = heightOut(pressure) - base_height_

 public:
  using Base::evaluateError;

  BaroFactor() = default;
  ~BaroFactor() override = default;

  /// @param pressure_kPa raw pressure measurement (kilopascal).
  /// @param p0_kPa local sea-level reference pressure (kPa); the standard
  ///        101.29 is only correct on a standard day -- on any other day this
  ///        is the local QFF/QNH and must be calibrated.
  BaroFactor(gtsam::Key pose_key, gtsam::Key baro_bias_key, double pressure_kPa,
             const gtsam::SharedNoiseModel& model, double base_height = 0.0,
             double p0_kPa = 101.29)
      : Base(model, pose_key, baro_bias_key),
        base_height_(base_height),
        height_rel_(height_from_pressure(pressure_kPa, p0_kPa) - base_height) {}

  auto evaluateError(const Pose& pose, const double& bias,
                     gtsam::OptionalMatrixType H1 = OptionalNone,
                     gtsam::OptionalMatrixType H2 = OptionalNone) const
      -> gtsam::Vector override;

  /// NASA-GRC simple troposphere model: pressure (kPa) -> height (m).
  /// p0_kPa is the *local* sea-level reference; the canonical 101.29 only
  /// holds on a standard day.
  static inline double height_from_pressure(double n_kPa,
                                            double p0_kPa = 101.29) {
    return (std::pow(n_kPa / p0_kPa, 1.0 / 5.256) * 288.08 - 273.1 - 15.04) /
           -0.00649;
  }

  /// Solve for p0 such that height_from_pressure(p_meas, p0) == h_target.
  /// Inverse of the forward formula above; T0 constants kept identical.
  static inline double p0_from_known_altitude(double p_meas_kPa,
                                              double h_target_m) {
    const double T_K = (273.1 + 15.04) - 0.00649 * h_target_m;
    return p_meas_kPa / std::pow(T_K / 288.08, 5.256);
  }
};

}  // namespace parnav
