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

#include "UpdaterNonHolonomic.h"

#include "UpdaterHelper.h"

#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterNonHolonomic::UpdaterNonHolonomic(UpdaterOptions &options, NoiseManager &noises, std::shared_ptr<ov_core::FeatureDatabase> db,
                                         std::shared_ptr<Propagator> prop, double gravity_mag, double nhc_max_velocity,
                                         double nhc_noise_multiplier)
    : _options(options), _noises(noises), _db(db), _prop(prop), _nhc_max_velocity(nhc_max_velocity),
      _nhc_noise_multiplier(nhc_noise_multiplier) {

  // Gravity
  _gravity << 0.0, 0.0, gravity_mag;

  // Save our raw pixel noise squared
  _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
  _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
  _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
  _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 1000; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

bool UpdaterNonHolonomic::try_update(std::shared_ptr<State> state, double timestamp) {

  // Return if the state is already at the desired time
  if (state->_timestamp == timestamp) {
    last_nhc_state_timestamp = 0.0;
    return false;
  }

  // Set the last time offset value if we have just started the system up
  if (!have_last_prop_time_offset) {
    last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
    have_last_prop_time_offset = true;
  }

  // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
  double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

  // Move forward in time
  last_prop_time_offset = t_off_new;

  // Check if velocity is within limits for non-holonomic constraint
  // If moving too fast, we might not want to apply this constraint
  if (state->_imu->vel().norm() > _nhc_max_velocity) {
    last_nhc_state_timestamp = 0.0;
    last_nhc_count = 0;
    PRINT_DEBUG(YELLOW "[NHC]: rejected - velocity too high |v_IinG| = %.3f (max %.3f)\n" RESET, state->_imu->vel().norm(),
                _nhc_max_velocity);
    return false;
  }

  // Order of our Jacobian (we only need velocity as state variable for this constraint)
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->v());

  // The non-holonomic constraint: lateral (y) and vertical (z) velocities in body frame should be zero
  // v_body = R_ItoG^T * v_IinG
  // We constrain: v_body_y = 0 and v_body_z = 0
  // Measurement dimension is 2 (y and z components)
  int h_size = 5; // 3 (orientation) + 2 (constrained velocity components)
  int m_size = 2; // 2 constraints (y and z velocity in body frame)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);

  // Get the rotation matrix from global to IMU frame
  Eigen::Matrix3d R_GtoI = state->_imu->Rot();
  Eigen::Vector3d v_IinG = state->_imu->vel();

  // Transform velocity to body frame
  Eigen::Vector3d v_IinI = R_GtoI * v_IinG;

  // Residual: The y and z components of velocity in body frame should be zero
  res(0) = v_IinI(1); // lateral velocity (y component)
  res(1) = v_IinI(2); // vertical velocity (z component)

  // Jacobian of v_IinI = R_GtoI * v_IinG with respect to orientation and velocity
  // d(v_IinI)/d(theta) = d(R_GtoI)/d(theta) * v_IinG = -[R_GtoI * v_IinG]_x = -[v_IinI]_x
  // d(v_IinI)/d(v_IinG) = R_GtoI

  // Get FEJ rotation if needed
  Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
  Eigen::Vector3d v_IinI_jacob = R_GtoI_jacob * v_IinG;

  // Jacobian w.r.t orientation (only y and z rows of the skew matrix)
  // H_theta = -[v_IinI]_x, we take rows 1 and 2 (y and z)
  Eigen::Matrix3d v_IinI_skew = skew_x(v_IinI_jacob);
  H.block(0, 0, 2, 3) = -v_IinI_skew.block(1, 0, 2, 3);

  // Jacobian w.r.t velocity (only y and z rows of R_GtoI)
  H.block(0, 3, 2, 3) = R_GtoI_jacob.block(1, 0, 2, 3);

  // Compress the system
  UpdaterHelper::measurement_compress_inplace(H, res);
  if (H.rows() < 1) {
    return false;
  }

  // Measurement noise covariance
  // We use a fixed noise for the non-holonomic constraint
  // This represents uncertainty in the constraint (e.g., small lateral slip)
  Eigen::MatrixXd R = _nhc_noise_multiplier * Eigen::MatrixXd::Identity(res.rows(), res.rows());
  // Scale by a reasonable velocity noise (e.g., 0.01 m/s for lateral slip)
  double sigma_nhc = 0.01; // 1 cm/s lateral velocity uncertainty
  R *= std::pow(sigma_nhc, 2);

  // Chi2 distance check
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  double chi2 = res.dot(S.llt().solve(res));

  // Get our threshold (we precompute up to 1000 but handle the case that it is more)
  double chi2_check;
  if (res.rows() < 1000) {
    chi2_check = chi_squared_table[res.rows()];
  } else {
    boost::math::chi_squared chi_squared_dist(res.rows());
    chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    PRINT_WARNING(YELLOW "[NHC]: chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
  }

  // Check if we should apply the constraint
  if (chi2 > _options.chi2_multipler * chi2_check) {
    last_nhc_state_timestamp = 0.0;
    last_nhc_count = 0;
    PRINT_DEBUG(YELLOW "[NHC]: rejected - chi2 %.3f > %.3f (residual: y=%.4f, z=%.4f)\n" RESET, chi2,
                _options.chi2_multipler * chi2_check, res(0), res(1));
    return false;
  }

  PRINT_INFO(CYAN "[NHC]: accepted - chi2 %.3f < %.3f (residual: y=%.4f, z=%.4f, |v|=%.3f)\n" RESET, chi2,
             _options.chi2_multipler * chi2_check, res(0), res(1), state->_imu->vel().norm());

  // Apply the update
  StateHelper::EKFUpdate(state, Hx_order, H, res, R);

  // Update state timestamp
  state->_timestamp = timestamp;

  // Finally return
  last_nhc_state_timestamp = timestamp;
  last_nhc_count++;
  return true;
}
