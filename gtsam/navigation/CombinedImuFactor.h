/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactor.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 *  @author Varun Agrawal
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

/* standard includes */
#include <ostream>

namespace gtsam {

#ifdef GTSAM_TANGENT_PREINTEGRATION
typedef TangentPreintegration DefaultPreintegrationType;
#else
typedef ManifoldPreintegration DefaultPreintegrationType;
#endif

/*
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating
 * conditionally independent sets in factor graphs: a unifying perspective based
 * on smart factors, Int. Conf. on Robotics and Automation (ICRA), 2014.
 *
 * REFERENCES:
 * [1] G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups",
 *     Volume 2, 2008.
 * [2] T. Lupton and S.Sukkarieh, "Visual-Inertial-Aided Navigation for
 *     High-Dynamic Motion in Built Environments Without Initial Conditions",
 *     TRO, 28(1):61-76, 2012.
 * [3] L. Carlone, S. Williams, R. Roberts, "Preintegrated IMU factor:
 *     Computation of the Jacobian Matrices", Tech. Report, 2013.
 *     Available in this repo as "PreintegratedIMUJacobians.pdf".
 * [4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, IMU Preintegration on
 *     Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation,
 *     Robotics: Science and Systems (RSS), 2015.
 */

/**
 * PreintegratedCombinedMeasurements integrates the IMU measurements
 * (rotation rates and accelerations) and the corresponding covariance matrix.
 * The measurements are then used to build the CombinedImuFactor. Integration
 * is done incrementally (ideally, one integrates the measurement as soon as
 * it is received from the IMU) so as to avoid costly integration at time of
 * factor construction.
 *
 * @ingroup navigation
 */
template <class PreintegrationType>
class GTSAM_EXPORT PreintegratedCombinedMeasurementsT : public PreintegrationType {
 public:
  typedef PreintegrationCombinedParams Params;

 protected:
  /* Covariance matrix of the preintegrated measurements
   * COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY BiasAcc
   * BiasOmega] (first-order propagation from *measurementCovariance*).
   * PreintegratedCombinedMeasurements also include the biases and keep the
   * correlation between the preintegrated measurements and the biases
   */
  Eigen::Matrix<double, 15, 15> preintMeasCov_;

  template <class PIM> friend class CombinedImuFactorT;

 public:
  /// @name Constructors
  /// @{

  /// Default constructor only for serialization and wrappers
  PreintegratedCombinedMeasurementsT() { this->resetIntegration(); }

  /**
   *  Default constructor, initializes the class with no measurements
   *  @param p Parameters, typically fixed in a single application
   *  @param biasHat Current estimate of acceleration and rotation rate biases
   *  @param preintMeasCov Covariance matrix used in noise model.
   */
  PreintegratedCombinedMeasurementsT(
      const std::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias(),
      const Eigen::Matrix<double, 15, 15>& preintMeasCov =
          Eigen::Matrix<double, 15, 15>::Zero())
      : PreintegrationType(p, biasHat), preintMeasCov_(preintMeasCov) {
    this->PreintegrationType::resetIntegration();
  }

  /**
   *  Construct preintegrated directly from members: base class and
   * preintMeasCov
   *  @param base               PreintegrationType instance
   *  @param preintMeasCov      Covariance matrix used in noise model.
   */
  PreintegratedCombinedMeasurementsT(
      const PreintegrationType& base,
      const Eigen::Matrix<double, 15, 15>& preintMeasCov)
      : PreintegrationType(base), preintMeasCov_(preintMeasCov) {
    this->PreintegrationType::resetIntegration();
  }

  /// Virtual destructor
  ~PreintegratedCombinedMeasurementsT() override {}

  /// @}

  /// @name Basic utilities
  /// @{

  /// Re-initialize PreintegratedCombinedMeasurements
  void resetIntegration() override;

  /// const reference to params, shadows definition in base class
  Params& p() const { return *std::static_pointer_cast<Params>(this->p_); }
  /// @}

  /// @name Access instance variables
  /// @{
  /// Return pre-integrated measurement covariance
  Matrix preintMeasCov() const { return preintMeasCov_; }
  /// @}

  /// @name Testable
  /// @{
  /// print
  void print(
      const std::string& s = "Preintegrated Measurements:") const override;
  /// equals
  bool equals(const PreintegratedCombinedMeasurementsT<PreintegrationType>& expected,
              double tol = 1e-9) const;
  /// @}

  /// @name Main functionality
  /// @{

  /**
   * Add a single IMU measurement to the preintegration.
   * Both accelerometer and gyroscope measurements are taken to be in the sensor
   * frame and conversion to the body frame is handled by `body_P_sensor` in
   * `PreintegrationParams`.
   *
   * @param measuredAcc Measured acceleration (as given by the sensor)
   * @param measuredOmega Measured angular velocity (as given by the sensor)
   * @param dt Time interval between two consecutive IMU measurements
   */
  void integrateMeasurement(const Vector3& measuredAcc,
                            const Vector3& measuredOmega,
                            const double dt) override;

  /// @}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
/// @deprecated: biasAccOmegaInt is no longer used. Use a prior on first bias instead.
  void resetIntegration(const gtsam::Matrix6& Q_init) {
    std::cerr << "Warning: setBiasAccOmegaInit() is deprecated and no longer used." << std::endl;
    PreintegrationType::resetIntegration();
    preintMeasCov_.setZero();
  }
#endif

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationType);
    ar& BOOST_SERIALIZATION_NVP(preintMeasCov_);
  }
#endif

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

// For backward compatibility:
using PreintegratedCombinedMeasurements = PreintegratedCombinedMeasurementsT<DefaultPreintegrationType>;

/**
 * CombinedImuFactor is a 6-ways factor involving previous state (pose and
 * velocity of the vehicle, as well as bias at previous time step), and current
 * state (pose, velocity, bias at current time step). Following the pre-
 * integration scheme proposed in [2], the CombinedImuFactor includes many IMU
 * measurements, which are "summarized" using the
 * PreintegratedCombinedMeasurements class. There are 3 main differences wrpt
 * the ImuFactor class: 1) The factor is 6-ways, meaning that it also involves
 * both biases (previous and current time step).Therefore, the factor internally
 * imposes the biases to be slowly varying; in particular, the matrices
 * "biasAccCovariance" and "biasOmegaCovariance" described the random walk that
 * models bias evolution. 2) The preintegration covariance takes into account
 * the noise in the bias estimate used for integration. 3) The covariance matrix
 * of the PreintegratedCombinedMeasurements preserves the correlation between
 * the bias uncertainty and the preintegrated measurements uncertainty.
 *
 * @ingroup navigation
 */
template <class PIM = PreintegratedCombinedMeasurements>
class GTSAM_EXPORT CombinedImuFactorT
    : public NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
                               imuBias::ConstantBias, imuBias::ConstantBias> {
 public:
 private:
  typedef CombinedImuFactorT<PIM> This;
  typedef NoiseModelFactorN<Pose3, Vector3, Pose3, Vector3,
                            imuBias::ConstantBias, imuBias::ConstantBias>
      Base;

  PIM pim_;

 public:
  // Provide access to Matrix& version of evaluateError:
  using Base::evaluateError;

  /** Shorthand for a smart pointer to a factor */
  typedef std::shared_ptr<This> shared_ptr;

  /** Default constructor - only use for serialization */
  CombinedImuFactorT() {}

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias_i Previous bias key
   * @param bias_j Current bias key
   * @param PreintegratedCombinedMeasurements Combined IMU measurements
   */
  CombinedImuFactorT(
      Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias_i, Key bias_j,
      const PIM& preintegratedMeasurements)
      : Base(noiseModel::Gaussian::Covariance(preintegratedMeasurements.preintMeasCov()),
             pose_i, vel_i, pose_j, vel_j, bias_i, bias_j),
        pim_(preintegratedMeasurements) {}

  ~CombinedImuFactorT() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  /** implement functions needed for Testable */

  /// @name Testable
  /// @{
  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;
  /// @}

  /** Access the preintegrated measurements. */

  const PIM& preintegratedMeasurements() const {
    return pim_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i,
                       const Pose3& pose_j, const Vector3& vel_j,
                       const imuBias::ConstantBias& bias_i,
                       const imuBias::ConstantBias& bias_j,
                       OptionalMatrixType H1, OptionalMatrixType H2,
                       OptionalMatrixType H3, OptionalMatrixType H4,
                       OptionalMatrixType H5,
                       OptionalMatrixType H6) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // NoiseModelFactor6 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor6", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(pim_);
  }
#endif

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
// class CombinedImuFactorT

// For backward compatibility:
using CombinedImuFactor = CombinedImuFactorT<>;

template <>
struct traits<PreintegrationCombinedParams>
    : public Testable<PreintegrationCombinedParams> {};

template <class PreintegrationType>
struct traits<PreintegratedCombinedMeasurementsT<PreintegrationType>>
    : public Testable<PreintegratedCombinedMeasurementsT<PreintegrationType>> {};
 
template <class PIM>
struct traits<CombinedImuFactorT<PIM>> : public Testable<CombinedImuFactorT<PIM>> {};

//------------------------------------------------------------------------------
// PreintegratedCombinedMeasurementsT template method implementations
//------------------------------------------------------------------------------
template <class PreintegrationType>
void PreintegratedCombinedMeasurementsT<PreintegrationType>::print(
    const std::string& s) const {
  PreintegrationType::print(s);
  std::cout << "  preintMeasCov [ " << preintMeasCov_ << " ]" << std::endl;
}

template <class PreintegrationType>
bool PreintegratedCombinedMeasurementsT<PreintegrationType>::equals(
    const PreintegratedCombinedMeasurementsT<PreintegrationType>& other,
    double tol) const {
  return PreintegrationType::equals(other, tol) &&
         equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
}

template <class PreintegrationType>
void PreintegratedCombinedMeasurementsT<
    PreintegrationType>::resetIntegration() {
  PreintegrationType::resetIntegration();
  preintMeasCov_.setZero();
}

// sugar for derivative blocks used in integrateMeasurement
#define GTSAM_D_R_R(H) (H)->block<3, 3>(0, 0)
#define GTSAM_D_R_t(H) (H)->block<3, 3>(0, 3)
#define GTSAM_D_R_v(H) (H)->block<3, 3>(0, 6)
#define GTSAM_D_t_R(H) (H)->block<3, 3>(3, 0)
#define GTSAM_D_t_t(H) (H)->block<3, 3>(3, 3)
#define GTSAM_D_t_v(H) (H)->block<3, 3>(3, 6)
#define GTSAM_D_v_R(H) (H)->block<3, 3>(6, 0)
#define GTSAM_D_v_t(H) (H)->block<3, 3>(6, 3)
#define GTSAM_D_v_v(H) (H)->block<3, 3>(6, 6)
#define GTSAM_D_a_a(H) (H)->block<3, 3>(9, 9)
#define GTSAM_D_g_g(H) (H)->block<3, 3>(12, 12)

template <class PreintegrationType>
void PreintegratedCombinedMeasurementsT<PreintegrationType>::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  if (dt <= 0) {
    throw std::runtime_error(
        "PreintegratedCombinedMeasurements::integrateMeasurement: dt <=0");
  }

  // Update preintegrated measurements.
  Matrix9 A;       // Jacobian wrt preintegrated measurements without bias (df/dx)
  Matrix93 B, C;  // Jacobian of state wrt accel bias and omega bias.
  PreintegrationType::update(measuredAcc, measuredOmega, dt, &A, &B, &C);

  // Update preintegrated measurements covariance: as in [2] we consider a first
  // order propagation that can be seen as a prediction phase in an EKF
  // framework. In this implementation, in contrast to [2], we consider the
  // uncertainty of the bias selection and we keep correlation between biases
  // and preintegrated measurements

  // Single Jacobians to propagate covariance
  Matrix3 theta_H_omega = C.topRows<3>();
  Matrix3 pos_H_acc = B.middleRows<3>(3);
  Matrix3 vel_H_acc = B.bottomRows<3>();

  // overall Jacobian wrt preintegrated measurements (df/dx)
  Eigen::Matrix<double, 15, 15> F;
  F.setZero();
  F.block<9, 9>(0, 0) = A;
  F.block<3, 3>(0, 12) = theta_H_omega;
  F.block<3, 3>(3, 9) = pos_H_acc;
  F.block<3, 3>(6, 9) = vel_H_acc;
  F.block<6, 6>(9, 9) = I_6x6;

  // Update the uncertainty on the state (matrix F in [4]).
  preintMeasCov_ = F * preintMeasCov_ * F.transpose();

  // propagate uncertainty
  // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
  const Matrix3& aCov = this->p().accelerometerCovariance;
  const Matrix3& wCov = this->p().gyroscopeCovariance;
  const Matrix3& iCov = this->p().integrationCovariance;

  // first order uncertainty propagation
  // Optimized matrix mult: (1/dt) * G * measurementCovariance * G.transpose()
  Eigen::Matrix<double, 15, 15> G_measCov_Gt;
  G_measCov_Gt.setZero(15, 15);

  // BLOCK DIAGONAL TERMS
  GTSAM_D_R_R(&G_measCov_Gt) =
      (theta_H_omega * (wCov / dt) * theta_H_omega.transpose());
  GTSAM_D_t_t(&G_measCov_Gt) =
      (pos_H_acc * (aCov / dt) * pos_H_acc.transpose()) + (dt * iCov);
  GTSAM_D_v_v(&G_measCov_Gt) = (vel_H_acc * (aCov / dt) * vel_H_acc.transpose());
  GTSAM_D_a_a(&G_measCov_Gt) = dt * this->p().biasAccCovariance;
  GTSAM_D_g_g(&G_measCov_Gt) = dt * this->p().biasOmegaCovariance;

  // OFF BLOCK DIAGONAL TERMS
  GTSAM_D_t_v(&G_measCov_Gt) = (pos_H_acc * (aCov / dt) * vel_H_acc.transpose());
  GTSAM_D_v_t(&G_measCov_Gt) = (vel_H_acc * (aCov / dt) * pos_H_acc.transpose());

  preintMeasCov_.noalias() += G_measCov_Gt;
}

#undef GTSAM_D_R_R
#undef GTSAM_D_R_t
#undef GTSAM_D_R_v
#undef GTSAM_D_t_R
#undef GTSAM_D_t_t
#undef GTSAM_D_t_v
#undef GTSAM_D_v_R
#undef GTSAM_D_v_t
#undef GTSAM_D_v_v
#undef GTSAM_D_a_a
#undef GTSAM_D_g_g

//------------------------------------------------------------------------------
// CombinedImuFactorT template method implementations
//------------------------------------------------------------------------------
template <class PIM>
void CombinedImuFactorT<PIM>::print(const std::string& s,
                                    const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? s : s + "\n")
            << "CombinedImuFactor("
            << keyFormatter(this->template key<1>()) << ","
            << keyFormatter(this->template key<2>()) << ","
            << keyFormatter(this->template key<3>()) << ","
            << keyFormatter(this->template key<4>()) << ","
            << keyFormatter(this->template key<5>()) << ","
            << keyFormatter(this->template key<6>()) << ")\n";
  pim_.print("  preintegrated measurements:");
  this->noiseModel_->print("  noise model: ");
}

template <class PIM>
bool CombinedImuFactorT<PIM>::equals(const NonlinearFactor& other,
                                     double tol) const {
  const This* e = dynamic_cast<const This*>(&other);
  return e != nullptr && Base::equals(*e, tol) && pim_.equals(e->pim_, tol);
}

template <class PIM>
Vector CombinedImuFactorT<PIM>::evaluateError(
    const Pose3& pose_i, const Vector3& vel_i, const Pose3& pose_j,
    const Vector3& vel_j, const imuBias::ConstantBias& bias_i,
    const imuBias::ConstantBias& bias_j, OptionalMatrixType H1,
    OptionalMatrixType H2, OptionalMatrixType H3, OptionalMatrixType H4,
    OptionalMatrixType H5, OptionalMatrixType H6) const {
  // error wrt bias evolution model (random walk)
  Matrix6 Hbias_i, Hbias_j;
  Vector6 fbias =
      traits<imuBias::ConstantBias>::Between(bias_j, bias_i,
                                             H6 ? &Hbias_j : nullptr,
                                             H5 ? &Hbias_i : nullptr)
          .vector();

  Matrix96 D_r_pose_i, D_r_pose_j, D_r_bias_i;
  Matrix93 D_r_vel_i, D_r_vel_j;

  // error wrt preintegrated measurements
  Vector9 r_Rpv = pim_.computeErrorAndJacobians(
      pose_i, vel_i, pose_j, vel_j, bias_i,
      H1 ? &D_r_pose_i : nullptr, H2 ? &D_r_vel_i : nullptr,
      H3 ? &D_r_pose_j : nullptr, H4 ? &D_r_vel_j : nullptr,
      H5 ? &D_r_bias_i : nullptr);

  // if we need the jacobians
  if (H1) {
    H1->resize(15, 6);
    H1->block<9, 6>(0, 0) = D_r_pose_i;
    // adding: [dBiasAcc/dPi ; dBiasOmega/dPi]
    H1->block<6, 6>(9, 0).setZero();
  }
  if (H2) {
    H2->resize(15, 3);
    H2->block<9, 3>(0, 0) = D_r_vel_i;
    // adding: [dBiasAcc/dVi ; dBiasOmega/dVi]
    H2->block<6, 3>(9, 0).setZero();
  }
  if (H3) {
    H3->resize(15, 6);
    H3->block<9, 6>(0, 0) = D_r_pose_j;
    // adding: [dBiasAcc/dPj ; dBiasOmega/dPj]
    H3->block<6, 6>(9, 0).setZero();
  }
  if (H4) {
    H4->resize(15, 3);
    H4->block<9, 3>(0, 0) = D_r_vel_j;
    // adding: [dBiasAcc/dVi ; dBiasOmega/dVi]
    H4->block<6, 3>(9, 0).setZero();
  }
  if (H5) {
    H5->resize(15, 6);
    H5->block<9, 6>(0, 0) = D_r_bias_i;
    // adding: [dBiasAcc/dBias_i ; dBiasOmega/dBias_i]
    H5->block<6, 6>(9, 0) = Hbias_i;
  }
  if (H6) {
    H6->resize(15, 6);
    H6->block<9, 6>(0, 0).setZero();
    // adding: [dBiasAcc/dBias_j ; dBiasOmega/dBias_j]
    H6->block<6, 6>(9, 0) = Hbias_j;
  }

  // overall error
  Vector r(15);
  r << r_Rpv, fbias;  // vector of size 15
  return r;
}

template <class PIM>
std::ostream& operator<<(std::ostream& os, const CombinedImuFactorT<PIM>& f) {
  f.preintegratedMeasurements().print("combined preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel()->sigmas().transpose();
  return os;
}

}  // namespace gtsam
