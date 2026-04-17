#pragma once

#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Pose3.h>

namespace parnav {

template <typename LieGroup> struct JacobianTraits;

template <> struct JacobianTraits<gtsam::Pose3> {
  static constexpr int Cols = 6;
  static constexpr int PosIdx = 3;
};

// ExtendedPose3 tangent order is (theta, nu, rho) = (att, vel, pos), so the
// position block lives at tangent index 6.
template <> struct JacobianTraits<gtsam::ExtendedPose3> {
  static constexpr int Cols = 9;
  static constexpr int PosIdx = 6;
};

}  // namespace parnav
