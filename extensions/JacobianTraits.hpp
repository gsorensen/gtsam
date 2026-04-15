#pragma once

#include <gtsam/geometry/Pose3.h>

namespace parnav {

template <typename LieGroup> struct JacobianTraits;

template <> struct JacobianTraits<gtsam::Pose3> {
  static constexpr int Cols = 6;
  static constexpr int PosIdx = 3;
};

}  // namespace parnav
