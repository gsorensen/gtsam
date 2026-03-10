/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ImuBias.h
 * @date  Feb 2, 2012
 * @author Vadim Indelman, Stephen Williams
 */

#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/VectorSpace.h>

#include <iosfwd>
#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif

namespace gtsam {

/// All bias models live in the imuBias namespace
namespace imuBias {

class GTSAM_EXPORT GaussMarkovBias {
 private:
  double tauAcc_;     ///< Correlation time for accelerometer bias
  double tauGyro_;    ///< Correlation time for gyroscope bias
  Vector3 biasAcc_;   ///< The units for stddev are σ = m/s² or m √Hz/s²
  Vector3 biasGyro_;  ///< The units for stddev are σ = rad/s or rad √Hz/s

 public:
  static const size_t dimension = 6;

  /// @name Standard Constructors
  /// @{

  GaussMarkovBias()
      : tauAcc_(1),
        tauGyro_(1),
        biasAcc_(0.0, 0.0, 0.0),
        biasGyro_(0.0, 0.0, 0.0) {}

  GaussMarkovBias(const Vector3& biasAcc, const Vector3& biasGyro,
                  const double& tauAcc, const double& tauGyro)
      : tauAcc_(tauAcc),
        tauGyro_(tauGyro),
        biasAcc_(biasAcc),
        biasGyro_(biasGyro) {}

  explicit GaussMarkovBias(const Vector6& v, const double& tauAcc,
                           const double& tauGyro)
      : tauAcc_(tauAcc),
        tauGyro_(tauGyro),
        biasAcc_(v.head<3>()),
        biasGyro_(v.tail<3>()) {}
  /// @}

  Vector6 vector() const {
    Vector6 v;
    v << biasAcc_, biasGyro_;
    return v;
  }

  const Vector3& accelerometer() const { return biasAcc_; }
  const Vector3& gyroscope() const { return biasGyro_; }

  Vector3 correctAccelerometer(const Vector3& measurement, double dt,
                               OptionalJacobian<3, 6> H1 = {},
                               OptionalJacobian<3, 3> H2 = {}) const {
    // Compute Jacobians and take the time constant into account
    const double beta = exp(-dt / tauAcc_);
    if (H1) (*H1) << -beta * I_3x3, Z_3x3;
    if (H2) (*H2) << I_3x3;
    return measurement - beta * biasAcc_;
  }

  Vector3 correctGyroscope(const Vector3& measurement, const double dt,
                           OptionalJacobian<3, 6> H1 = {},
                           OptionalJacobian<3, 3> H2 = {}) const {
    const double beta = exp(-dt / tauGyro_);
    if (H1) (*H1) << Z_3x3, -beta * I_3x3;
    if (H2) (*H2) << I_3x3;
    return measurement - beta * biasGyro_;
  }

  /// @name Testable
  /// @{

  /// ostream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const GaussMarkovBias& bias);

  /// print with optional string
  void print(const std::string& s = "") const;

  /** equality up to tolerance */
  inline bool equals(const GaussMarkovBias& expected, double tol = 1e-5) const {
    return equal_with_abs_tol(biasAcc_, expected.biasAcc_, tol) &&
           equal_with_abs_tol(biasGyro_, expected.biasGyro_, tol);
  }

  /// @}

  /// @name Group
  /// @{
  static GaussMarkovBias Identity() { return GaussMarkovBias(); }

  inline GaussMarkovBias operator-() const {
    return GaussMarkovBias(-biasAcc_, -biasGyro_, tauAcc_, tauGyro_);
  }

  GaussMarkovBias operator+(const Vector6& v) const {
    return GaussMarkovBias(biasAcc_ + v.head<3>(), biasGyro_ + v.tail<3>(),
                           tauAcc_, tauGyro_);
  }

  GaussMarkovBias operator+(const GaussMarkovBias& b) const {
    return GaussMarkovBias(biasAcc_ + b.biasAcc_, biasGyro_ + b.biasGyro_,
                           tauAcc_, tauGyro_);
  }

  GaussMarkovBias operator-(const GaussMarkovBias& b) const {
    return GaussMarkovBias(biasAcc_ - b.biasAcc_, biasGyro_ - b.biasGyro_,
                           tauAcc_, tauGyro_);
  }

  /// @}
  ///
  /// @name Manifold
  /// @{
  GaussMarkovBias retract(const Vector6& v) const {
    return GaussMarkovBias(biasAcc_ + v.head<3>(), biasGyro_ + v.tail<3>(),
                           tauAcc_, tauGyro_);
  }

  Vector6 localCoordinates(const GaussMarkovBias& other) const {
    return other.vector() - vector();
  }

  /// @}
};

class GTSAM_EXPORT ConstantBias {
 private:
  Vector3 biasAcc_;   ///< The units for stddev are σ = m/s² or m √Hz/s²
  Vector3 biasGyro_;  ///< The units for stddev are σ = rad/s or rad √Hz/s

 public:
  /// dimension of the variable - used to autodetect sizes
  static const size_t dimension = 6;

  /// @name Standard Constructors
  /// @{

  ConstantBias() : biasAcc_(0.0, 0.0, 0.0), biasGyro_(0.0, 0.0, 0.0) {}

  ConstantBias(const Vector3& biasAcc, const Vector3& biasGyro)
      : biasAcc_(biasAcc), biasGyro_(biasGyro) {}

  explicit ConstantBias(const Vector6& v)
      : biasAcc_(v.head<3>()), biasGyro_(v.tail<3>()) {}

  /// @}

  /** return the accelerometer and gyro biases in a single vector */
  Vector6 vector() const {
    Vector6 v;
    v << biasAcc_, biasGyro_;
    return v;
  }

  /** get accelerometer bias */
  const Vector3& accelerometer() const { return biasAcc_; }

  /** get gyroscope bias */
  const Vector3& gyroscope() const { return biasGyro_; }

  /** Correct an accelerometer measurement using this bias model, and optionally
   * compute Jacobians */
  Vector3 correctAccelerometer(const Vector3& measurement,
                               OptionalJacobian<3, 6> H1 = {},
                               OptionalJacobian<3, 3> H2 = {}) const {
    if (H1) (*H1) << -I_3x3, Z_3x3;
    if (H2) (*H2) << I_3x3;
    return measurement - biasAcc_;
  }

  /** Correct a gyroscope measurement using this bias model, and optionally
   * compute Jacobians */
  Vector3 correctGyroscope(const Vector3& measurement,
                           OptionalJacobian<3, 6> H1 = {},
                           OptionalJacobian<3, 3> H2 = {}) const {
    if (H1) (*H1) << Z_3x3, -I_3x3;
    if (H2) (*H2) << I_3x3;
    return measurement - biasGyro_;
  }

  /// @name Testable
  /// @{

  /// ostream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const ConstantBias& bias);

  /// print with optional string
  void print(const std::string& s = "") const;

  /** equality up to tolerance */
  inline bool equals(const ConstantBias& expected, double tol = 1e-5) const {
    return equal_with_abs_tol(biasAcc_, expected.biasAcc_, tol) &&
           equal_with_abs_tol(biasGyro_, expected.biasGyro_, tol);
  }

  /// @}
  /// @name Group
  /// @{

  /** identity for group operation */
  static ConstantBias Identity() { return ConstantBias(); }

  /** inverse */
  inline ConstantBias operator-() const {
    return ConstantBias(-biasAcc_, -biasGyro_);
  }

  /** addition of vector on right */
  ConstantBias operator+(const Vector6& v) const {
    return ConstantBias(biasAcc_ + v.head<3>(), biasGyro_ + v.tail<3>());
  }

  /** addition */
  ConstantBias operator+(const ConstantBias& b) const {
    return ConstantBias(biasAcc_ + b.biasAcc_, biasGyro_ + b.biasGyro_);
  }

  /** subtraction */
  ConstantBias operator-(const ConstantBias& b) const {
    return ConstantBias(biasAcc_ - b.biasAcc_, biasGyro_ - b.biasGyro_);
  }

  /// @}
  /// @name Manifold
  /// @{

  /// The retract function
  ConstantBias retract(const Vector6& v) const {
    return ConstantBias(biasAcc_ + v.head<3>(), biasGyro_ + v.tail<3>());
  }

  /// The local coordinates function
  Vector6 localCoordinates(const ConstantBias& other) const {
    return other.vector() - vector();
  }

  /// @}

 private:
  /// @name Advanced Interface
  /// @{

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(biasAcc_);
    ar& BOOST_SERIALIZATION_NVP(biasGyro_);
  }
#endif

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
  /// @}

};  // ConstantBias class
}  // namespace imuBias

template <>
struct traits<imuBias::ConstantBias>
    : public internal::VectorSpace<imuBias::ConstantBias> {};

template <>
struct traits<const imuBias::GaussMarkovBias>
    : public internal::VectorSpace<imuBias::GaussMarkovBias> {};

}  // namespace gtsam
