/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testExtendedPose3.cpp
 * @brief   Unit tests for ExtendedPose3 (SE_2(3), Barrau ordering).
 */

#include <gtsam/geometry/ExtendedPose3.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <CppUnitLite/TestHarness.h>

#include <functional>

using namespace std;
using namespace gtsam;

// Concept instantiations.
GTSAM_CONCEPT_TESTABLE_INST(ExtendedPose3)
GTSAM_CONCEPT_MATRIX_LIE_GROUP_INST(ExtendedPose3)

static const double kTol = 1e-8;

// Common test fixtures.
static const Rot3 kRot = Rot3::RzRyRx(0.1, -0.2, 0.3);
static const Velocity3 kVel(0.5, 0.6, -0.7);
static const Point3 kPos(1.0, -2.0, 3.0);
static const Pose3 kPose(kRot, kPos);
static const ExtendedPose3 kX(kRot, kVel, kPos);
static const ExtendedPose3 kIdentity = ExtendedPose3::Identity();
static const ExtendedPose3 kX2(Rot3::RzRyRx(-0.2, 0.3, 0.1), Velocity3(0.6, -0.7, 0.5),
                               Point3(-2.0, 3.0, 1.0));

/* ************************************************************************* */
TEST(ExtendedPose3, Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<ExtendedPose3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<ExtendedPose3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<ExtendedPose3>);
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<ExtendedPose3>);
}

/* ************************************************************************* */
TEST(ExtendedPose3, LieGroupDerivatives) {
  CHECK_LIE_GROUP_DERIVATIVES(kIdentity, kIdentity);
  CHECK_LIE_GROUP_DERIVATIVES(kIdentity, kX);
  CHECK_LIE_GROUP_DERIVATIVES(kX, kIdentity);
  CHECK_LIE_GROUP_DERIVATIVES(kX, kX2);
}

/* ************************************************************************* */
TEST(ExtendedPose3, ChartDerivatives) {
  CHECK_CHART_DERIVATIVES(kIdentity, kIdentity);
  CHECK_CHART_DERIVATIVES(kIdentity, kX);
  CHECK_CHART_DERIVATIVES(kX, kIdentity);
  CHECK_CHART_DERIVATIVES(kX, kX2);
}

/* ************************************************************************* */
TEST(ExtendedPose3, Constructors) {
  // From (R, v, p) with Jacobians.
  Matrix93 H1, H2, H3;
  ExtendedPose3 g1 = ExtendedPose3::Create(kRot, kVel, kPos, H1, H2, H3);
  EXPECT(assert_equal(kX, g1, kTol));

  std::function<ExtendedPose3(const Rot3&, const Velocity3&, const Point3&)> create =
      [](const Rot3& R, const Velocity3& v, const Point3& p) {
        return ExtendedPose3::Create(R, v, p);
      };
  EXPECT(assert_equal(numericalDerivative31(create, kRot, kVel, kPos), (Matrix)H1, 1e-6));
  EXPECT(assert_equal(numericalDerivative32(create, kRot, kVel, kPos), (Matrix)H2, 1e-6));
  EXPECT(assert_equal(numericalDerivative33(create, kRot, kVel, kPos), (Matrix)H3, 1e-6));

  // From (Pose3, velocity) with Jacobians.
  Matrix Hp1, Hp2;
  ExtendedPose3 g2 = ExtendedPose3::FromPoseVelocity(kPose, kVel, Hp1, Hp2);
  EXPECT(assert_equal(kX, g2, kTol));

  std::function<ExtendedPose3(const Pose3&, const Velocity3&)> fromPV =
      [](const Pose3& pose, const Velocity3& v) {
        return ExtendedPose3::FromPoseVelocity(pose, v);
      };
  EXPECT(assert_equal(numericalDerivative21(fromPV, kPose, kVel), Hp1, 1e-6));
  EXPECT(assert_equal(numericalDerivative22(fromPV, kPose, kVel), Hp2, 1e-6));

  // Default constructor is identity.
  EXPECT(assert_equal(kIdentity, ExtendedPose3(), kTol));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Accessors) {
  // Values.
  EXPECT(assert_equal(kRot, kX.attitude(), kTol));
  EXPECT(assert_equal(kRot, kX.rotation(), kTol));
  EXPECT(assert_equal(kVel, kX.velocity(), kTol));
  EXPECT(assert_equal(kPos, kX.position(), kTol));
  EXPECT(assert_equal(kPose, kX.pose(), kTol));

  // Attitude Jacobian.
  Matrix39 Ha;
  kX.attitude(Ha);
  std::function<Rot3(const ExtendedPose3&)> att =
      [](const ExtendedPose3& X) { return X.attitude(); };
  EXPECT(assert_equal(numericalDerivative11<Rot3, ExtendedPose3>(att, kX), (Matrix)Ha, 1e-7));

  // Velocity Jacobian.
  Matrix39 Hv;
  kX.velocity(Hv);
  std::function<Velocity3(const ExtendedPose3&)> vel =
      [](const ExtendedPose3& X) { return X.velocity(); };
  EXPECT(assert_equal(numericalDerivative11<Velocity3, ExtendedPose3>(vel, kX), (Matrix)Hv, 1e-7));

  // Position Jacobian.
  Matrix39 Hp;
  kX.position(Hp);
  std::function<Point3(const ExtendedPose3&)> pos =
      [](const ExtendedPose3& X) { return X.position(); };
  EXPECT(assert_equal(numericalDerivative11<Point3, ExtendedPose3>(pos, kX), (Matrix)Hp, 1e-7));

  // bodyVelocity Jacobian and value.
  Matrix39 Hbv;
  Velocity3 bv = kX.bodyVelocity(Hbv);
  EXPECT(assert_equal<Velocity3>(kRot.unrotate(kVel), bv, kTol));
  std::function<Velocity3(const ExtendedPose3&)> bvf =
      [](const ExtendedPose3& X) { return X.bodyVelocity(); };
  EXPECT(assert_equal(numericalDerivative11<Velocity3, ExtendedPose3>(bvf, kX), (Matrix)Hbv, 1e-7));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Matrix) {
  // Layout: [R v p; 0 1 0; 0 0 1]
  Matrix5 M = kX.matrix();
  EXPECT(assert_equal(kRot.matrix(), Matrix3(M.block<3, 3>(0, 0)), kTol));
  EXPECT(assert_equal(kVel, Vector3(M.block<3, 1>(0, 3)), kTol));
  EXPECT(assert_equal(kPos, Point3(M.block<3, 1>(0, 4)), kTol));
  EXPECT_DOUBLES_EQUAL(1.0, M(3, 3), kTol);
  EXPECT_DOUBLES_EQUAL(0.0, M(3, 4), kTol);
  EXPECT_DOUBLES_EQUAL(0.0, M(4, 3), kTol);
  EXPECT_DOUBLES_EQUAL(1.0, M(4, 4), kTol);

  // Round-trip through Matrix5 constructor.
  ExtendedPose3 g(M);
  EXPECT(assert_equal(kX, g, kTol));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Identity) {
  EXPECT(assert_equal(ExtendedPose3(), kIdentity, kTol));
  EXPECT(assert_equal(kX, kIdentity * kX, kTol));
  EXPECT(assert_equal(kX, kX * kIdentity, kTol));
  EXPECT(assert_equal(kIdentity, kX * kX.inverse(), kTol));
  EXPECT(assert_equal(kIdentity, kX.inverse() * kX, kTol));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Compose) {
  // Direct check: (R1 R2, v1 + R1 v2, p1 + R1 p2)
  ExtendedPose3 prod = kX * kX2;
  EXPECT(assert_equal(kX.rotation() * kX2.rotation(), prod.rotation(), kTol));
  EXPECT(assert_equal<Velocity3>(kX.velocity() + kX.rotation() * kX2.velocity(),
                                 prod.velocity(), kTol));
  EXPECT(assert_equal<Point3>(kX.position() + kX.rotation() * kX2.position(),
                              prod.position(), kTol));

  // Associativity.
  EXPECT(assert_equal((kX * kX2) * kIdentity, kX * (kX2 * kIdentity), kTol));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Inverse) {
  ExtendedPose3 inv = kX.inverse();
  // Inverse satisfies X * X^-1 = Identity.
  EXPECT(assert_equal(kIdentity, kX * inv, 1e-8));
  EXPECT(assert_equal(kIdentity, inv * kX, 1e-8));
}

/* ************************************************************************* */
TEST(ExtendedPose3, HatVee) {
  Vector9 xi = (Vector9() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.7, 0.8, -0.9).finished();
  Matrix5 X = ExtendedPose3::Hat(xi);

  // Layout: [w^ nu rho; 0 0 0 0 0; 0 0 0 0 0]
  EXPECT(assert_equal(skewSymmetric(xi.head<3>()), Matrix3(X.block<3, 3>(0, 0)), kTol));
  EXPECT(assert_equal(Vector3(xi.segment<3>(3)), Vector3(X.block<3, 1>(0, 3)), kTol));
  EXPECT(assert_equal(Vector3(xi.tail<3>()), Vector3(X.block<3, 1>(0, 4)), kTol));
  EXPECT(assert_equal(Vector2::Zero().eval(), Vector2(X.block<2, 1>(3, 3)), kTol));

  // Vee round trip.
  EXPECT(assert_equal(xi, ExtendedPose3::Vee(X), kTol));
}

/* ************************************************************************* */
TEST(ExtendedPose3, ExpmapZero) {
  Matrix9 H;
  ExtendedPose3 g = ExtendedPose3::Expmap(Vector9::Zero(), H);
  EXPECT(assert_equal(kIdentity, g, kTol));

  std::function<ExtendedPose3(const Vector9&)> f =
      [](const Vector9& xi) { return ExtendedPose3::Expmap(xi); };
  EXPECT(assert_equal(numericalDerivative11(f, Vector9(Vector9::Zero())), H, 1e-6));
}

/* ************************************************************************* */
TEST(ExtendedPose3, ExpmapJacobian) {
  Vector9 xi = (Vector9() << 0.05, -0.1, 0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8).finished();
  Matrix9 H;
  ExtendedPose3::Expmap(xi, H);
  std::function<ExtendedPose3(const Vector9&)> f =
      [](const Vector9& v) { return ExtendedPose3::Expmap(v); };
  EXPECT(assert_equal(numericalDerivative11(f, xi), H, 1e-6));
}

/* ************************************************************************* */
TEST(ExtendedPose3, LogmapJacobian) {
  const double jac_tol = 1e-6;
  Matrix9 H;
  ExtendedPose3::Logmap(kX, H);
  std::function<Vector9(const ExtendedPose3&)> f =
      [](const ExtendedPose3& X) { return ExtendedPose3::Logmap(X); };
  EXPECT(assert_equal(numericalDerivative11<Vector9, ExtendedPose3>(f, kX, jac_tol), H, jac_tol));

  Matrix9 H2;
  ExtendedPose3::Logmap(kX2, H2);
  EXPECT(assert_equal(numericalDerivative11<Vector9, ExtendedPose3>(f, kX2, jac_tol), H2, jac_tol));
}

/* ************************************************************************* */
TEST(ExtendedPose3, ExpLogRoundtrip) {
  // General tangent vector.
  Vector9 xi = (Vector9() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.7, 0.8, -0.9).finished();
  ExtendedPose3 g = ExtendedPose3::Expmap(xi);
  EXPECT(assert_equal(xi, ExtendedPose3::Logmap(g), 1e-7));

  // Round trip group -> tangent -> group.
  Vector9 xi_back = ExtendedPose3::Logmap(kX);
  ExtendedPose3 gx = ExtendedPose3::Expmap(xi_back);
  EXPECT(assert_equal(kX, gx, 1e-7));
}

/* ************************************************************************* */
TEST(ExtendedPose3, ExpLogNearZero) {
  Vector9 xi;
  xi << 1e-8, -2e-8, 1.5e-8, 3e-6, -4e-6, 2.5e-6, -1e-5, 2e-5, -3e-5;
  ExtendedPose3 g = ExtendedPose3::Expmap(xi);
  EXPECT(assert_equal(xi, ExtendedPose3::Logmap(g), 1e-8));

  Vector9 xi_zero = Vector9::Zero();
  EXPECT(assert_equal(xi_zero, ExtendedPose3::Logmap(ExtendedPose3::Expmap(xi_zero)), 1e-10));
}

/* ************************************************************************* */
TEST(ExtendedPose3, ExpmapDerivativeConsistency) {
  // ExpmapDerivative(xi) must equal the Jacobian returned by Expmap(xi, H).
  Vector9 xi = (Vector9() << 0.1, 0.2, -0.3, 0.4, -0.5, 0.6, -0.7, 0.8, -0.9).finished();
  Matrix9 H;
  ExtendedPose3::Expmap(xi, H);
  EXPECT(assert_equal(H, ExtendedPose3::ExpmapDerivative(xi), kTol));
}

/* ************************************************************************* */
TEST(ExtendedPose3, LogmapDerivativeConsistency) {
  // LogmapDerivative(X) must equal the Jacobian returned by Logmap(X, H).
  Matrix9 H;
  ExtendedPose3::Logmap(kX, H);
  EXPECT(assert_equal(H, ExtendedPose3::LogmapDerivative(kX), 1e-8));
}

/* ************************************************************************* */
TEST(ExtendedPose3, ExpmapLogmapInverseDerivative) {
  // ExpmapDerivative(xi) and LogmapDerivative(Expmap(xi)) should be mutual inverses.
  Vector9 xi = (Vector9() << 0.1, 0.2, -0.3, 0.4, -0.5, 0.6, -0.7, 0.8, -0.9).finished();
  Matrix9 E = ExtendedPose3::ExpmapDerivative(xi);
  Matrix9 L = ExtendedPose3::LogmapDerivative(ExtendedPose3::Expmap(xi));
  EXPECT(assert_equal(Matrix9::Identity().eval(), Matrix9(L * E), 1e-6));
}

/* ************************************************************************* */
TEST(ExtendedPose3, AdjointMap) {
  Matrix9 Ad = kX.AdjointMap();

  // Generic (compose-derived) AdjointMap should match specialised implementation.
  Matrix9 Ad_generic =
      static_cast<const MatrixLieGroup<ExtendedPose3, 9, 5>*>(&kX)->AdjointMap();
  EXPECT(assert_equal(Ad_generic, Ad, 1e-7));

  // Adjoint property: Log(g * Exp(xi) * g^-1) = Ad(g) * xi.
  Vector9 xi = (Vector9() << 0.05, -0.1, 0.15, 0.2, -0.25, 0.3, -0.35, 0.4, -0.45).finished();
  ExtendedPose3 conj = kX * ExtendedPose3::Expmap(xi) * kX.inverse();
  EXPECT(assert_equal(Vector9(Ad * xi), ExtendedPose3::Logmap(conj), 1e-7));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Adjoint) {
  Vector9 xi = (Vector9() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.7, 0.8, -0.9).finished();
  Matrix9 H_this, H_xi;
  Vector9 result = kX.Adjoint(xi, H_this, H_xi);
  EXPECT(assert_equal(Vector9(kX.AdjointMap() * xi), result, kTol));

  std::function<Vector9(const ExtendedPose3&, const Vector9&)> adjf =
      [](const ExtendedPose3& X, const Vector9& v) { return X.Adjoint(v); };
  EXPECT(assert_equal(numericalDerivative21(adjf, kX, xi), H_this, 1e-6));
  EXPECT(assert_equal(numericalDerivative22(adjf, kX, xi), H_xi, 1e-6));
}

/* ************************************************************************* */
TEST(ExtendedPose3, StaticAdjoint) {
  Vector9 xi = (Vector9() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.7, 0.8, -0.9).finished();
  Vector9 y = (Vector9() << -0.5, 0.4, -0.3, 0.2, -0.1, 0.0, 0.3, -0.35, 0.45).finished();

  // ad(xi) * y == adjoint(xi, y).
  Matrix9 ad = ExtendedPose3::adjointMap(xi);
  EXPECT(assert_equal(Vector9(ad * y), ExtendedPose3::adjoint(xi, y), kTol));

  // Derivative wrt y equals ad(xi).
  Matrix Hxi, Hy;
  ExtendedPose3::adjoint(xi, y, Hxi, Hy);
  EXPECT(assert_equal((Matrix)ad, Hy, kTol));

  std::function<Vector9(const Vector9&, const Vector9&)> adf =
      [](const Vector9& a, const Vector9& b) { return ExtendedPose3::adjoint(a, b); };
  EXPECT(assert_equal(numericalDerivative21(adf, xi, y), Hxi, 1e-6));
  EXPECT(assert_equal(numericalDerivative22(adf, xi, y), Hy, 1e-6));

  // Jacobi identity: [x, [y, z]] + [y, [z, x]] + [z, [x, y]] = 0.
  Vector9 z = (Vector9() << 0.7, 0.8, 0.9, -0.1, -0.2, -0.3, 0.5, 0.6, 0.7).finished();
  Vector9 sum =
      ExtendedPose3::adjoint(xi, ExtendedPose3::adjoint(y, z)) +
      ExtendedPose3::adjoint(y, ExtendedPose3::adjoint(z, xi)) +
      ExtendedPose3::adjoint(z, ExtendedPose3::adjoint(xi, y));
  EXPECT(assert_equal(Vector9(Vector9::Zero()), sum, 1e-9));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Vec) {
  ExtendedPose3::Vector25 v = kX.vec();
  Matrix5 T = kX.matrix();
  EXPECT(assert_equal(ExtendedPose3::Vector25(Eigen::Map<const ExtendedPose3::Vector25>(T.data())),
                      v, kTol));

  Eigen::Matrix<double, 25, 9> H;
  kX.vec(H);
  auto f = [](const ExtendedPose3& X) -> ExtendedPose3::Vector25 { return X.vec(); };
  Matrix H_num = numericalDerivative11<ExtendedPose3::Vector25, ExtendedPose3, 9>(f, kX);
  EXPECT(assert_equal(H_num, H, 1e-7));
}

/* ************************************************************************* */
TEST(ExtendedPose3, Retract) {
  // retract(zero) == self.
  EXPECT(assert_equal(kX, kX.retract(Vector9::Zero()), kTol));
  EXPECT(assert_equal(Vector9::Zero().eval(), kX.localCoordinates(kX), kTol));

  // retract/local round-trip.
  Vector9 xi = (Vector9() << 0.1, -0.1, 0.2, 0.3, -0.2, 0.1, -0.3, 0.2, 0.1).finished();
  ExtendedPose3 Y = kX.retract(xi);
  EXPECT(assert_equal(xi, kX.localCoordinates(Y), 1e-7));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
