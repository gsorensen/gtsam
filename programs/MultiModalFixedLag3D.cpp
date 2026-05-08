/**
 * @file MultiModalFixedLag3D.cpp
 * @brief 3D fixed-lag smoother for the multi-modale-simulator export.
 *
 * Consumes `simulation_results_3d.npz` + `simulation_results_3d.meta.json`
 * produced by `f_export_3d.py`. The simulator is planar; we lift it into
 * SO(3)/R^3 and pin (z, roll, pitch) via heterogeneous Pose3 priors so the
 * standard GTSAM CombinedImuFactor + GPSFactor pipeline can run unchanged.
 *
 * Out of scope: data association for polar/camera detections, robust kernels,
 * trust model. This is a port-validation harness, not a port of the Python
 * factor-graph frontend.
 */

#include "CompassFactor.hpp"
#include "MultiModalSimLoader.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace {

struct Args {
  std::string sim_data;
  std::string meta;
  int ship_index = 0;
  double lag = 5.0;
  std::string out_csv = "multimodal_estimates.csv";
  std::string robust_pos = "none";   // none | huber | tukey | gmc
  double robust_pos_k = 1.345;        // Huber default; Tukey ~ 4.685; GMC ~ 1.0
  std::string robust_yaw = "none";
  double robust_yaw_k = 1.345;
};

Args parseArgs(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc)
        throw std::runtime_error("missing value for " + k);
      return argv[++i];
    };
    if (k == "--sim-data") a.sim_data = next();
    else if (k == "--meta") a.meta = next();
    else if (k == "--ship-index") a.ship_index = std::stoi(next());
    else if (k == "--lag") a.lag = std::stod(next());
    else if (k == "--out") a.out_csv = next();
    else if (k == "--robust-pos") a.robust_pos = next();
    else if (k == "--robust-pos-k") a.robust_pos_k = std::stod(next());
    else if (k == "--robust-yaw") a.robust_yaw = next();
    else if (k == "--robust-yaw-k") a.robust_yaw_k = std::stod(next());
    else if (k == "-h" || k == "--help") {
      std::cout
          << "MultiModalFixedLag3D --sim-data PATH --meta PATH "
          << "[--ship-index 0] [--lag 5.0] [--out estimates.csv]\n"
          << "  [--robust-pos none|huber|tukey|gmc] [--robust-pos-k K]\n"
          << "  [--robust-yaw none|huber|tukey|gmc] [--robust-yaw-k K]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown flag: " + k);
    }
  }
  if (a.sim_data.empty() || a.meta.empty())
    throw std::runtime_error("--sim-data and --meta are required");
  return a;
}

gtsam::SharedNoiseModel wrapRobust(const std::string& kind, double k,
                                   const gtsam::SharedNoiseModel& base) {
  if (kind == "none" || kind.empty()) return base;
  using gtsam::noiseModel::mEstimator::GemanMcClure;
  using gtsam::noiseModel::mEstimator::Huber;
  using gtsam::noiseModel::mEstimator::Tukey;
  using gtsam::noiseModel::Robust;
  if (kind == "huber") return Robust::Create(Huber::Create(k), base);
  if (kind == "tukey") return Robust::Create(Tukey::Create(k), base);
  if (kind == "gmc")   return Robust::Create(GemanMcClure::Create(k), base);
  throw std::runtime_error("unknown robust kernel: " + kind);
}

gtsam::Pose3 poseFromGt(const parnav::SimData3D& d, int ship, int t) {
  auto p = d.gtPose(ship, t);
  gtsam::Rot3 R = gtsam::Rot3::Quaternion(p[3], p[4], p[5], p[6]);
  return gtsam::Pose3(R, gtsam::Point3(p[0], p[1], p[2]));
}

}  // namespace

int main(int argc, char** argv) {
  Args args = parseArgs(argc, argv);

  parnav::SimMeta meta = parnav::loadSimMeta(args.meta);
  parnav::SimData3D d = parnav::loadSimData(args.sim_data, meta);

  if (args.ship_index < 0 || args.ship_index >= d.n_ships)
    throw std::runtime_error("ship-index out of range");
  const int ship = args.ship_index;
  const int T = d.T;
  const double g0 = meta.gravity;

  // ---- Noise models ----
  // GNSS xy/yaw pulled from sensor metadata; z held tight (z=0 is exact in the
  // lifted scenario). Position and yaw are fully decoupled — separate base
  // models, separate (optional) robust wrappers, separate validity gates.
  double sigma_xy = 0.5;
  double sigma_yaw = 0.05;
  for (const auto& g : meta.gnss) {
    if (g.ship == ship || g.ship == -1) {
      if (g.noise_xy > 0.0) sigma_xy = g.noise_xy;
      if (g.noise_heading > 0.0) sigma_yaw = g.noise_heading;
      break;
    }
  }
  auto gnss_pos_base = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << sigma_xy, sigma_xy, 0.05).finished());
  auto gnss_yaw_base = gtsam::noiseModel::Isotropic::Sigma(1, sigma_yaw);
  auto gnss_pos_noise = wrapRobust(args.robust_pos, args.robust_pos_k, gnss_pos_base);
  auto gnss_yaw_noise = wrapRobust(args.robust_yaw, args.robust_yaw_k, gnss_yaw_base);

  // Heterogeneous Pose3 prior pinning (z, roll, pitch) tightly while leaving
  // (x, y, yaw) loose. Order is (rx, ry, rz, tx, ty, tz) per GTSAM's Pose3
  // tangent convention.
  const double sig_loose = 1e6;
  const double sig_planar = 1e-3;
  auto planar_pin = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << sig_planar, sig_planar, sig_loose,
       sig_loose, sig_loose, sig_planar).finished());

  // Initial-state prior (tight on truth at k=0).
  auto pose_prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
  auto vel_prior_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
  auto bias_prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

  // ---- IMU preintegration params ----
  double sigma_acc = 0.05;       // m/s^2/sqrt(Hz) -- tunable; sim is noiseless
  double sigma_gyro = 0.005;     // rad/s/sqrt(Hz)
  double sigma_b_acc = 1e-4;
  double sigma_b_gyro = 1e-5;

  auto p = gtsam::PreintegrationCombinedParams::MakeSharedU(g0);
  p->accelerometerCovariance = gtsam::I_3x3 * sigma_acc * sigma_acc;
  p->gyroscopeCovariance = gtsam::I_3x3 * sigma_gyro * sigma_gyro;
  p->integrationCovariance = gtsam::I_3x3 * 1e-10;
  p->biasAccCovariance = gtsam::I_3x3 * sigma_b_acc * sigma_b_acc;
  p->biasOmegaCovariance = gtsam::I_3x3 * sigma_b_gyro * sigma_b_gyro;

  // ---- Fixed-lag smoother ----
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeSkip = 1;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.findUnusedFactorSlots = true;
  gtsam::IncrementalFixedLagSmoother smoother(args.lag, isam_params);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;

  // ---- t=0: priors + ground-truth initialization ----
  gtsam::Pose3 pose0 = poseFromGt(d, ship, 0);
  auto v0_eig = d.gtVel(ship, 0);
  gtsam::Vector3 v0(v0_eig[0], v0_eig[1], v0_eig[2]);
  gtsam::imuBias::ConstantBias bias0;

  graph.addPrior<gtsam::Pose3>(X(0), pose0, pose_prior_noise);
  graph.addPrior<gtsam::Vector3>(V(0), v0, vel_prior_noise);
  graph.addPrior<gtsam::imuBias::ConstantBias>(B(0), bias0, bias_prior_noise);
  values.insert(X(0), pose0);
  values.insert(V(0), v0);
  values.insert(B(0), bias0);
  timestamps[X(0)] = d.time[0];
  timestamps[V(0)] = d.time[0];
  timestamps[B(0)] = d.time[0];

  smoother.update(graph, values, timestamps);
  graph.resize(0);
  values.clear();
  timestamps.clear();

  auto pim = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(p, bias0);

  gtsam::NavState prev_state(pose0, v0);
  gtsam::imuBias::ConstantBias prev_bias = bias0;

  // ---- Output CSV ----
  std::ofstream out(args.out_csv);
  out << "t,x,y,z,qw,qx,qy,qz,vx,vy,vz,gt_x,gt_y,gt_yaw\n";

  auto write_row = [&](double t, const gtsam::Pose3& P, const gtsam::Vector3& v,
                       int gt_t) {
    auto q = P.rotation().toQuaternion();
    auto pt = P.translation();
    auto gp = d.gtPose(ship, gt_t);
    double gt_yaw = 2.0 * std::atan2(gp[6], gp[3]);
    out << t << ',' << pt.x() << ',' << pt.y() << ',' << pt.z() << ','
        << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
        << v.x() << ',' << v.y() << ',' << v.z() << ','
        << gp[0] << ',' << gp[1] << ',' << gt_yaw << '\n';
  };
  write_row(d.time[0], pose0, v0, 0);

  // ---- Main loop ----
  for (int t = 1; t < T; ++t) {
    auto imu_t = d.imuSample(ship, t);
    gtsam::Vector3 omega(imu_t[0], imu_t[1], imu_t[2]);
    gtsam::Vector3 accel(imu_t[3], imu_t[4], imu_t[5]);
    pim->integrateMeasurement(accel, omega, d.imu_dt[t]);

    // CombinedImuFactor between (X,V,B)_{t-1} and (X,V,B)_t
    graph.add(gtsam::CombinedImuFactor(X(t - 1), V(t - 1), X(t), V(t),
                                       B(t - 1), B(t), *pim));

    // Predict initial estimate.
    gtsam::NavState pred = pim->predict(prev_state, prev_bias);
    values.insert(X(t), pred.pose());
    values.insert(V(t), pred.velocity());
    values.insert(B(t), prev_bias);

    // Planar pin: heterogeneous Pose3 prior anchored at predicted (x, y, yaw)
    // with z=0, roll=pitch=0. Rebuild a planar pose from the prediction.
    {
      gtsam::Vector3 rpy = pred.pose().rotation().rpy();
      gtsam::Rot3 R_planar = gtsam::Rot3::Yaw(rpy.z());
      gtsam::Pose3 planar_target(
          R_planar,
          gtsam::Point3(pred.pose().x(), pred.pose().y(), 0.0));
      graph.addPrior<gtsam::Pose3>(X(t), planar_target, planar_pin);
    }

    // GNSS aiding — position and yaw are independent factors with independent
    // validity masks and independent (robust) noise models. Spoofed position
    // therefore cannot drag yaw with it; the M-estimator on the position side
    // is what's expected to absorb the spoofing.
    for (int s = 0; s < d.n_gnss; ++s) {
      const auto& gm = meta.gnss[s];
      if (gm.ship != ship && gm.ship != -1) continue;
      if (d.gnssPosValid(s, t)) {
        auto r = d.gnssPos(s, t);
        graph.add(gtsam::GPSFactor(X(t),
                                   gtsam::Point3(r[0], r[1], r[2]),
                                   gnss_pos_noise));
      }
      if (d.gnssYawValid(s, t)) {
        graph.add(parnav::CompassFactor<gtsam::Pose3>(
            X(t), d.gnssYaw(s, t), gnss_yaw_noise));
      }
    }

    timestamps[X(t)] = d.time[t];
    timestamps[V(t)] = d.time[t];
    timestamps[B(t)] = d.time[t];

    try {
      smoother.update(graph, values, timestamps);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "smoother.update failed at t=%d: %s\n", t, e.what());
      throw;
    }

    auto est_pose = smoother.calculateEstimate<gtsam::Pose3>(X(t));
    auto est_vel = smoother.calculateEstimate<gtsam::Vector3>(V(t));
    auto est_bias =
        smoother.calculateEstimate<gtsam::imuBias::ConstantBias>(B(t));

    write_row(d.time[t], est_pose, est_vel, t);

    prev_state = gtsam::NavState(est_pose, est_vel);
    prev_bias = est_bias;
    pim->resetIntegrationAndSetBias(prev_bias);

    graph.resize(0);
    values.clear();
    timestamps.clear();
  }

  out.close();
  std::printf("Wrote %s (T=%d, ship=%d, lag=%.2fs)\n",
              args.out_csv.c_str(), T, ship, args.lag);
  return 0;
}
