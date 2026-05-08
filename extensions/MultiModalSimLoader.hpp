#pragma once

// Loader for the multi-modale-simulator 3D export
// (`simulation_results_3d.npz` + `simulation_results_3d.meta.json`).
//
// The schema is fixed by `f_export_3d.py` in the simulator repo. Every array
// is plain float64; cnpy reads it directly. Sensor identities live in the
// JSON sidecar so cnpy never has to handle strings or object arrays.

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

namespace parnav {

struct GnssSensorMeta {
  std::string name;
  int ship{-1};
  std::array<double, 3> relative_pose{{0.0, 0.0, 0.0}};
  double noise_xy{0.0};
  double noise_heading{0.0};
};

struct PolarSensorMeta {
  std::string name;
  int ship{-1};
  std::array<double, 3> relative_pose{{0.0, 0.0, 0.0}};
  double range_noise{0.0};
  double angle_noise_deg{0.0};
};

struct CameraSensorMeta {
  std::string name;
  int ship{-1};
  std::array<double, 3> relative_pose{{0.0, 0.0, 0.0}};
  double angle_noise_deg{0.0};
};

struct SimMeta {
  std::string scenario;
  int n_ships{0};
  int T{0};
  double dt_nominal{1.0};
  double gravity{9.81};
  std::vector<GnssSensorMeta> gnss;
  std::vector<PolarSensorMeta> polar;
  std::vector<CameraSensorMeta> camera;
};

// Numeric payload. Indices match SimMeta::{gnss,polar,camera} ordering.
struct SimData3D {
  // (T,)
  std::vector<double> time;
  std::vector<double> imu_dt;

  int n_ships{0};
  int T{0};

  // (n_ships * T * 7)  layout [x,y,z,qw,qx,qy,qz]
  std::vector<double> gt_pose;
  // (n_ships * T * 3)
  std::vector<double> gt_vel;
  // (n_ships * T * 6)  body-frame [wx,wy,wz, ax,ay,az]
  std::vector<double> imu;

  // GNSS position and heading are independent streams.
  int n_gnss{0};
  std::vector<double> gnss_pos;              // (Ng * T * 3)
  std::vector<std::uint8_t> gnss_pos_valid;  // (Ng * T)
  std::vector<double> gnss_yaw;              // (Ng * T)
  std::vector<std::uint8_t> gnss_yaw_valid;  // (Ng * T)

  // Polar: (Np * T * Kp * 3) [range, az, el]; valid (Np * T * Kp)
  int n_polar{0};
  int kmax_polar{0};
  std::vector<double> polar;
  std::vector<std::uint8_t> polar_valid;

  // Camera: (Nc * T * Kc * 2) [az, el]; valid (Nc * T * Kc)
  int n_camera{0};
  int kmax_camera{0};
  std::vector<double> camera;
  std::vector<std::uint8_t> camera_valid;

  // Static
  std::vector<double> markers;            // (Nm * 3)
  int n_markers{0};
  std::vector<double> shoreline;          // (Ns * 2 * 3)
  int n_shoreline{0};

  // Accessors -------------------------------------------------------------
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gtPose(int ship, int t) const {
    return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
        gt_pose.data() + (ship * T + t) * 7);
  }
  Eigen::Map<const Eigen::Matrix<double, 3, 1>> gtVel(int ship, int t) const {
    return Eigen::Map<const Eigen::Matrix<double, 3, 1>>(
        gt_vel.data() + (ship * T + t) * 3);
  }
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> imuSample(int ship, int t) const {
    return Eigen::Map<const Eigen::Matrix<double, 6, 1>>(
        imu.data() + (ship * T + t) * 6);
  }
  bool gnssPosValid(int s, int t) const {
    return gnss_pos_valid[s * T + t] != 0;
  }
  Eigen::Map<const Eigen::Matrix<double, 3, 1>> gnssPos(int s, int t) const {
    return Eigen::Map<const Eigen::Matrix<double, 3, 1>>(
        gnss_pos.data() + (s * T + t) * 3);
  }
  bool gnssYawValid(int s, int t) const {
    return gnss_yaw_valid[s * T + t] != 0;
  }
  double gnssYaw(int s, int t) const {
    return gnss_yaw[s * T + t];
  }
  bool polarValid(int s, int t, int k) const {
    return polar_valid[(s * T + t) * kmax_polar + k] != 0;
  }
  Eigen::Map<const Eigen::Matrix<double, 3, 1>> polarReading(int s, int t, int k) const {
    return Eigen::Map<const Eigen::Matrix<double, 3, 1>>(
        polar.data() + ((s * T + t) * kmax_polar + k) * 3);
  }
  bool cameraValid(int s, int t, int k) const {
    return camera_valid[(s * T + t) * kmax_camera + k] != 0;
  }
  Eigen::Map<const Eigen::Matrix<double, 2, 1>> cameraReading(int s, int t, int k) const {
    return Eigen::Map<const Eigen::Matrix<double, 2, 1>>(
        camera.data() + ((s * T + t) * kmax_camera + k) * 2);
  }
};

// Throws std::runtime_error on schema mismatch or IO errors.
SimMeta loadSimMeta(const std::string& json_path);
SimData3D loadSimData(const std::string& npz_path, const SimMeta& meta);

}  // namespace parnav
