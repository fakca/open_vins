/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_UPDATER_NONHOLONOMIC_H
#define OV_MSCKF_UPDATER_NONHOLONOMIC_H

#include <memory>

#include "utils/sensor_data.h"

#include "UpdaterOptions.h"
#include "utils/NoiseManager.h"

namespace ov_core {
class Feature;
class FeatureDatabase;
} // namespace ov_core
namespace ov_type {
class Landmark;
} // namespace ov_type

namespace ov_msckf {

class State;
class Propagator;

/**
 * @brief Will try to update using non-holonomic constraint assumption.
 *
 * Non-holonomic constraints are applicable to wheeled vehicles that cannot move sideways.
 * This constraint enforces that the lateral (sideways) and vertical velocities in the vehicle frame are zero.
 * The vehicle can only move in its forward direction (along its longitudinal axis).
 * This is particularly useful for ground vehicles such as cars, differential drive robots, and other wheeled platforms.
 * 
 * The constraint is: v_body_y = 0 and v_body_z = 0
 * where v_body is the velocity expressed in the IMU body frame.
 */
class UpdaterNonHolonomic {

public:
  /**
   * @brief Default constructor for our non-holonomic constraint updater.
   * @param options Updater options (chi2 multiplier)
   * @param noises imu noise characteristics (continuous time)
   * @param db Feature tracker database with all features in it
   * @param prop Propagator class object which can predict the state forward in time
   * @param gravity_mag Global gravity magnitude of the system (normally 9.81)
   * @param nhc_max_velocity Max velocity we should consider to do a update with
   * @param nhc_noise_multiplier Multiplier of our constraint noise (default should be 1.0)
   */
  UpdaterNonHolonomic(UpdaterOptions &options, NoiseManager &noises, std::shared_ptr<ov_core::FeatureDatabase> db,
                      std::shared_ptr<Propagator> prop, double gravity_mag, double nhc_max_velocity, double nhc_noise_multiplier);

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   * @param oldest_time Time that we can discard measurements before
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1) {

    // Append it to our vector
    imu_data.emplace_back(message);

    // Clean old measurements
    clean_old_imu_measurements(oldest_time - 0.10);
  }

  /**
   * @brief This will remove any IMU measurements that are older then the given measurement time
   * @param oldest_time Time that we can discard measurements before (in IMU clock)
   */
  void clean_old_imu_measurements(double oldest_time) {
    if (oldest_time < 0)
      return;
    auto it0 = imu_data.begin();
    while (it0 != imu_data.end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  /**
   * @brief Try to apply non-holonomic constraint update to the state.
   * @param state State of the filter
   * @param timestamp Next camera timestamp we want to see if we should propagate to.
   * @return True if the non-holonomic constraint was successfully applied
   */
  bool try_update(std::shared_ptr<State> state, double timestamp);

protected:
  /// Options used during update (chi2 multiplier)
  UpdaterOptions _options;

  /// Container for the imu noise values
  NoiseManager _noises;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our propagator!
  std::shared_ptr<Propagator> _prop;

  /// Gravity vector
  Eigen::Vector3d _gravity;

  /// Max velocity (m/s) that we should consider a non-holonomic constraint with
  double _nhc_max_velocity = 5.0;

  /// Multiplier of our constraint noise (default should be 1.0)
  double _nhc_noise_multiplier = 1.0;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data;

  /// Estimate for time offset at last propagation time
  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  /// Last timestamp we did non-holonomic constraint update with
  double last_nhc_state_timestamp = 0.0;

  /// Number of times we have called update
  int last_nhc_count = 0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_NONHOLONOMIC_H
