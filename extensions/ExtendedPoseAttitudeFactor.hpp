#pragma once

#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace parnav {

/// Attitude factor for ExtendedPose3: 2D Unit3 residual on rotation.
/// Mirrors gtsam::Pose3AttitudeFactor. Tangent order for ExtendedPose3 is
/// (theta, nu, rho), so the 2x3 rotation Jacobian goes in columns 0..2.
class ExtendedPoseAttitudeFactor
    : public gtsam::NoiseModelFactorN<gtsam::ExtendedPose3>,
      public gtsam::AttitudeFactor {
  using Base = gtsam::NoiseModelFactorN<gtsam::ExtendedPose3>;

 public:
  using Base::evaluateError;
  using shared_ptr = std::shared_ptr<ExtendedPoseAttitudeFactor>;
  using This = ExtendedPoseAttitudeFactor;

  ExtendedPoseAttitudeFactor() = default;
  ~ExtendedPoseAttitudeFactor() override = default;

  /// @param key      ExtendedPose3 variable key
  /// @param nRef     reference direction in navigation frame
  /// @param model    Gaussian noise model (2D)
  /// @param bMeasured measured direction in body frame (default Z-axis)
  ExtendedPoseAttitudeFactor(gtsam::Key key, const gtsam::Unit3& nRef,
                             const gtsam::SharedNoiseModel& model,
                             const gtsam::Unit3& bMeasured = gtsam::Unit3(0, 0,
                                                                          1))
      : Base(model, key), gtsam::AttitudeFactor(nRef, bMeasured) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  gtsam::Vector evaluateError(
      const gtsam::ExtendedPose3& nTb,
      gtsam::OptionalMatrixType H = OptionalNone) const override {
    gtsam::Vector e = attitudeError(nTb.rotation(), H);
    if (H) {
      gtsam::Matrix H23 = *H;
      *H = gtsam::Matrix::Zero(2, 9);
      H->block<2, 3>(0, 0) = H23;
    }
    return e;
  }
};

}  // namespace parnav
