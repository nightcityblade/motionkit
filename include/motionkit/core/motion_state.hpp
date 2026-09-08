// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "motionkit/core/types.hpp"

namespace motionkit {

/// Where an axis is and what it is doing, at one instant.
///
/// Distinct from MotionSample, which carries jerk as well. Jerk is the control
/// input rather than part of the state: it can be chosen freely at any instant,
/// while position, velocity and acceleration cannot be. A planner that took
/// jerk as an initial condition would be accepting something it is about to
/// overwrite.
///
/// Units are the axis's own and are never converted by this library: metres
/// for a prismatic axis, radians for a revolute one, each derivative per
/// second. Mixing them is the caller's error to avoid, and the reason limits
/// and states should be sourced from the same axis description.
struct MotionState {
  /// Position along the axis.
  Scalar position{0.0};
  /// First derivative of position with respect to time.
  Scalar velocity{0.0};
  /// Second derivative of position with respect to time.
  Scalar acceleration{0.0};
};

/// The state of one axis at one instant, together with the jerk being applied.
struct MotionSample {
  /// Position along the axis.
  Scalar position{0.0};
  /// First derivative of position with respect to time.
  Scalar velocity{0.0};
  /// Second derivative of position with respect to time.
  Scalar acceleration{0.0};
  /// Third derivative of position with respect to time -- the control input
  /// being applied over the segment this sample falls in, not a measurement.
  Scalar jerk{0.0};

  /// The state alone, with the jerk dropped. Useful for handing the current
  /// point to a planner, which takes a state and chooses its own jerk.
  [[nodiscard]] constexpr MotionState state() const noexcept {
    return MotionState{position, velocity, acceleration};
  }
};

}  // namespace motionkit
