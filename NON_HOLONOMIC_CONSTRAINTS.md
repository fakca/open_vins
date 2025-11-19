# Non-Holonomic Constraint Support in OpenVINS

This document describes the non-holonomic constraint feature added to OpenVINS for improved state estimation in wheeled ground vehicles.

## Overview

Non-holonomic constraints are kinematic constraints that apply to wheeled vehicles that cannot move sideways. This includes:
- Cars and trucks
- Differential drive robots
- Ackermann-steered vehicles
- Most wheeled ground robots

The constraint enforces that lateral (y-axis) and vertical (z-axis) velocities in the vehicle body frame are zero, allowing only forward/backward motion along the longitudinal axis.

## When to Use

**Use non-holonomic constraints when:**
- Your platform is a wheeled ground vehicle
- You're experiencing drift in lateral motion estimates
- Visual features are scarce or unreliable
- You want to improve long-term accuracy on ground vehicles

**Do NOT use when:**
- Platform is an aerial vehicle (drone, aircraft)
- Platform has omnidirectional capability (mecanum wheels, etc.)
- Vehicle is experiencing significant wheel slip
- Platform is not constrained to ground motion (e.g., marine vehicles)

## Configuration

Add the following parameters to your `estimator_config.yaml`:

```yaml
# non-holonomic constraint update parameters
# useful for wheeled vehicles that cannot move sideways
try_nhc: true                  # Enable/disable NHC updates
nhc_chi2_multipler: 1.0        # Chi-squared test multiplier
nhc_max_velocity: 5.0          # Max velocity (m/s) to apply constraint
nhc_noise_multiplier: 1.0      # Noise multiplier for uncertainty
```

### Parameter Tuning Guide

**`try_nhc`** (boolean, default: false)
- Set to `true` to enable non-holonomic constraint updates
- Set to `false` to disable (default for safety)

**`nhc_chi2_multipler`** (float, default: 1.0)
- Controls how strict the chi-squared validation is
- Higher values (e.g., 5.0) = more permissive, accepts more updates
- Lower values (e.g., 0.5) = more strict, rejects dubious updates
- Start with 1.0 and increase if updates are rejected too often

**`nhc_max_velocity`** (float, default: 5.0 m/s)
- Maximum velocity magnitude to apply constraint
- Prevents constraint application during very high-speed motion
- For slow robots: use 2.0 m/s
- For normal driving: use 5.0-10.0 m/s
- For high-speed applications: use 15.0+ m/s

**`nhc_noise_multiplier`** (float, default: 1.0)
- Scales the measurement noise of the constraint
- Higher values (e.g., 10.0) = weaker constraint, more uncertainty
- Lower values (e.g., 0.1) = stronger constraint, less uncertainty
- Increase if filter diverges with NHC enabled
- Decrease for tighter constraint in well-controlled environments

## Example Configurations

### Conservative (Recommended for Testing)
```yaml
try_nhc: true
nhc_chi2_multipler: 5.0
nhc_max_velocity: 3.0
nhc_noise_multiplier: 10.0
```

### Balanced (Recommended for Most Applications)
```yaml
try_nhc: true
nhc_chi2_multipler: 1.0
nhc_max_velocity: 5.0
nhc_noise_multiplier: 1.0
```

### Aggressive (For Controlled Environments)
```yaml
try_nhc: true
nhc_chi2_multipler: 1.0
nhc_max_velocity: 10.0
nhc_noise_multiplier: 0.5
```

## Integration with Other Features

### Using with Zero Velocity Update (ZUPT)
Both ZUPT and NHC can be enabled simultaneously:
```yaml
# Zero velocity for stationary detection
try_zupt: true
zupt_max_velocity: 0.5
zupt_noise_multiplier: 10

# Non-holonomic for motion constraints
try_nhc: true
nhc_max_velocity: 5.0
nhc_noise_multiplier: 1.0
```

ZUPT handles stationary cases while NHC handles moving cases with kinematic constraints.

## Monitoring and Debugging

### Log Messages
The implementation outputs informative log messages:

**Accepted updates:**
```
[NHC]: accepted - chi2 0.523 < 5.991 (residual: y=0.0012, z=0.0034, |v|=2.341)
```

**Rejected updates:**
```
[NHC]: rejected - chi2 12.453 > 5.991 (residual: y=0.1234, z=0.0567)
[NHC]: rejected - velocity too high |v_IinG| = 7.234 (max 5.000)
```

### Troubleshooting

**Problem: NHC updates always rejected**
- Solution: Increase `nhc_chi2_multipler` to 5.0 or higher
- Check if vehicle is actually following non-holonomic constraints

**Problem: Filter diverges with NHC enabled**
- Solution: Increase `nhc_noise_multiplier` to 5.0 or 10.0
- Reduce `nhc_max_velocity` to apply only at lower speeds
- Check if wheel slip or sliding is occurring

**Problem: Not enough NHC updates**
- Solution: Increase `nhc_max_velocity`
- Check log messages to see why updates are rejected

## Technical Implementation

### Files Modified/Added
- `ov_msckf/src/update/UpdaterNonHolonomic.h` - Header file
- `ov_msckf/src/update/UpdaterNonHolonomic.cpp` - Implementation
- `ov_msckf/src/core/VioManager.h` - Integration
- `ov_msckf/src/core/VioManager.cpp` - Integration
- `ov_msckf/src/core/VioManagerOptions.h` - Configuration options
- `ov_msckf/cmake/ROS1.cmake` - Build configuration
- `docs/update-nonholonomic.dox` - Technical documentation

### Key Equations

**Constraint:**
```
v_body_y = 0  (lateral velocity = 0)
v_body_z = 0  (vertical velocity = 0)
```

**Transformation:**
```
v_body = R_GtoI * v_global
```

Where `R_GtoI` is the rotation from global to IMU frame.

## Performance Expectations

With properly tuned parameters, you should expect:
- Reduced lateral drift in straight-line motion
- Improved orientation estimates during motion
- Better long-term trajectory accuracy
- Minimal computational overhead (~0.1-0.5 ms per update)

## References

For detailed mathematical derivations and theory, see:
- `docs/update-nonholonomic.dox` - Full mathematical documentation
- OpenVINS documentation: https://docs.openvins.com/

## Support

For issues, questions, or contributions related to this feature:
- Open an issue on the OpenVINS GitHub repository
- Include your configuration parameters and log output
- Describe your vehicle platform and use case
