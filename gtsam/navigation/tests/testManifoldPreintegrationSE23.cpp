/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testManifoldPreintegrationSE23.cpp
 * @brief   Unit tests for ManifoldPreintegrationSE23 (Barrau SE_2(3) form).
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/ManifoldPreintegrationSE23.h>

#include <CppUnitLite/TestHarness.h>

#include "imuFactorTesting.h"

using namespace std::placeholders;

/* ************************************************************************* */
// The class's bias-Jacobian blocks live in the SE_2(3) tangent of the
// *biasCorrectedDelta*, not on the (R, v, p) fields of deltaXij_. Verify by
// comparing H = d(biasCorrectedDelta)/d(bias_i) against a numerical derivative
// around a small nonzero biasHat (so the linearization matches a finite-
// difference probe).
TEST(ManifoldPreintegrationSE23, BiasCorrectedDeltaJacobian) {
  testing::SomeMeasurements measurements;

  const Bias biasHat(Vector3(0.01, -0.02, 0.005),
                     Vector3(-0.003, 0.004, 0.002));
  ManifoldPreintegrationSE23<> pim(testing::Params(), biasHat);
  testing::integrateMeasurements(measurements, &pim);

  std::function<Vector9(const Bias&)> f = [&](const Bias& b) {
    return pim.biasCorrectedDelta(b, {});
  };

  Matrix96 aH;
  pim.biasCorrectedDelta(biasHat, aH);

  EXPECT(assert_equal(numericalDerivative11<Vector9, Bias>(f, biasHat),
                      Matrix(aH), 1e-6));
}

/* ************************************************************************* */
// Check predict() Jacobians with numerical derivatives.
TEST(ManifoldPreintegrationSE23, PredictJacobians) {
  testing::SomeMeasurements measurements;
  ManifoldPreintegrationSE23<> pim(testing::Params());
  testing::integrateMeasurements(measurements, &pim);

  const ExtendedPose3 x1(Rot3::Yaw(0.2), Vector3(0.4, -0.1, 0.05),
                         Point3(0.3, -0.2, 0.1));
  const Bias bias(Vector3(0.01, -0.02, 0.005), Vector3(-0.001, 0.002, 0.003));

  Matrix9 aH1;
  Matrix96 aH2;
  pim.predict(x1, bias, aH1, aH2);

  std::function<ExtendedPose3(const ExtendedPose3&, const Bias&)> f =
      [&](const ExtendedPose3& s, const Bias& b) {
        return pim.predict(s, b);
      };

  EXPECT(assert_equal(numericalDerivative21(f, x1, bias), aH1, 1e-5));
  EXPECT(assert_equal(numericalDerivative22(f, x1, bias), aH2, 1e-5));
}

/* ************************************************************************* */
// Check computeError() Jacobians with numerical derivatives.
TEST(ManifoldPreintegrationSE23, ComputeErrorJacobians) {
  testing::SomeMeasurements measurements;
  ManifoldPreintegrationSE23<> pim(testing::Params());
  testing::integrateMeasurements(measurements, &pim);

  const ExtendedPose3 x1(Rot3::Yaw(0.2), Vector3(0.4, -0.1, 0.05),
                         Point3(0.3, -0.2, 0.1));
  const ExtendedPose3 x2 = pim.predict(x1, Bias());
  const Bias bias(Vector3(0.01, -0.02, 0.005), Vector3(-0.001, 0.002, 0.003));

  Matrix9 aH1, aH2;
  Matrix96 aH3;
  pim.computeError(x1, x2, bias, aH1, aH2, aH3);

  std::function<Vector9(const ExtendedPose3&, const ExtendedPose3&,
                        const Bias&)>
      f = [&](const ExtendedPose3& a, const ExtendedPose3& b, const Bias& c) {
        return pim.computeError(a, b, c, nullptr, nullptr, nullptr);
      };

  EXPECT(assert_equal(numericalDerivative31(f, x1, x2, bias), aH1, 1e-5));
  EXPECT(assert_equal(numericalDerivative32(f, x1, x2, bias), aH2, 1e-5));
  EXPECT(assert_equal(numericalDerivative33(f, x1, x2, bias), aH3, 1e-5));
}

/* ************************************************************************* */
// Sanity: zero gyro, constant acc along +x with zero gravity should give a
// kinematic straight-line trajectory.
TEST(ManifoldPreintegrationSE23, StraightLineZeroGravity) {
  auto p = std::make_shared<PreintegrationParams>(Vector3::Zero());
  p->gyroscopeCovariance = 1e-8 * I_3x3;
  p->accelerometerCovariance = 1e-8 * I_3x3;
  p->integrationCovariance = 1e-8 * I_3x3;

  ManifoldPreintegrationSE23<> pim(p);
  const Vector3 acc(1.0, 0.0, 0.0);
  const Vector3 omega = Vector3::Zero();
  const double dt = 0.01;
  const int N = 100;
  for (int i = 0; i < N; ++i) pim.integrateMeasurement(acc, omega, dt);

  const double T = N * dt;  // 1.0 s
  const ExtendedPose3 x1;   // identity
  const ExtendedPose3 x2 = pim.predict(x1, Bias());

  EXPECT(assert_equal(Rot3(), x2.rotation(), 1e-9));
  // v = a * T = 1.0
  EXPECT(assert_equal(Vector3(1.0, 0, 0), x2.velocity(), 1e-6));
  // p = 0.5 * a * T^2 = 0.5
  EXPECT(assert_equal(Point3(0.5, 0, 0), x2.position(), 1e-6));
  EXPECT_DOUBLES_EQUAL(T, pim.deltaTij(), 1e-9);
}

/* ************************************************************************* */
// Sanity: zero bias -> computeError on predicted state is zero.
TEST(ManifoldPreintegrationSE23, ZeroResidualAtPrediction) {
  testing::SomeMeasurements measurements;
  ManifoldPreintegrationSE23<> pim(testing::Params());
  testing::integrateMeasurements(measurements, &pim);

  const ExtendedPose3 x1(Rot3::Ypr(0.1, -0.2, 0.3), Vector3(0.5, 0.1, -0.2),
                         Point3(1.0, 2.0, -0.5));
  const Bias bias;
  const ExtendedPose3 x2 = pim.predict(x1, bias);

  const Vector9 e =
      pim.computeError(x1, x2, bias, nullptr, nullptr, nullptr);
  EXPECT(assert_equal(Vector(Vector9::Zero()), Vector(e), 1e-8));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
