#pragma once

#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace parnav {

/// Position-only GPS measurement on ExtendedPose3.
/// Residual = position(X) - measured.
class GPSFactorSE23
    : public gtsam::NoiseModelFactorN<gtsam::ExtendedPose3> {
  gtsam::Point3 measured_;

 public:
  GPSFactorSE23(gtsam::Key key, const gtsam::Point3& measured,
                const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactorN<gtsam::ExtendedPose3>(model, key),
        measured_(measured) {}

  gtsam::Vector evaluateError(
      const gtsam::ExtendedPose3& X,
      gtsam::OptionalMatrixType H = OptionalNone) const override {
    if (H) {
      gtsam::Matrix H_p(3, 9);
      gtsam::Point3 p = X.position(H_p);
      *H = H_p;
      return p - measured_;
    }
    return X.position() - measured_;
  }
};

}  // namespace parnav
