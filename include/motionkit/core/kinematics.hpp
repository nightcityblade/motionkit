// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// Joints a chain can carry. Six for an arm, a couple more for a positioner.
inline constexpr std::size_t kMaxJoints = 8;

/// Rows of a spatial Jacobian: three linear, three angular.
inline constexpr std::size_t kTwistSize = 6;

/// Why a kinematics call could not answer.
enum class KinematicsError : std::uint8_t {
  None = 0,
  /// A joint count that does not match the chain, or a buffer too small.
  SizeMismatch,
  /// More joints than kMaxJoints, or none at all.
  JointCountUnsupported,
  /// A joint axis of zero length. An axis is a direction; there is no such
  /// thing as a joint that rotates about nothing.
  DegenerateAxis,
  /// A NaN or infinite input.
  NonFiniteInput,
  /// The solver reached its iteration budget without meeting the tolerance.
  DidNotConverge,
  /// The solver converged to a pose outside the joint limits and could not
  /// find a way back inside.
  OutsideJointLimits,
};

std::string_view toString(KinematicsError error) noexcept;

/// One revolute joint, described where it can be measured.
///
/// `axis` and `point` are given **in the base frame with every joint at zero**.
/// That is the whole reason this library does not use Denavit-Hartenberg
/// parameters: a DH table requires a particular frame attached to every link,
/// there are two incompatible conventions in circulation that produce different
/// arms from identical numbers, and neither is anything you can measure. An
/// axis direction and a point on that axis are both readable off a CAD model
/// with a ruler, and there is only one way to interpret them.
struct RevoluteJoint {
  /// Direction of positive rotation, right-handed. Normalised on construction
  /// of the chain; need not be unit here.
  Vec3 axis{0.0, 0.0, 1.0};
  /// Any point lying on the axis.
  Vec3 point;
  /// Travel limits in radians. The defaults are deliberately wider than any
  /// real joint so that an unconfigured chain is unconstrained rather than
  /// silently locked -- unlike MotionLimits, where silence must mean "may not
  /// move", a joint limit of zero here would mean a chain that cannot be posed
  /// at all, and that failure is not safer than the alternative.
  Scalar lower{-6.2831853071795864769};
  Scalar upper{6.2831853071795864769};
};

/// How an inverse-kinematics solve ended, and what it cost.
struct IkReport {
  /// Iterations actually taken.
  std::size_t iterations{0};
  /// Remaining position error in metres, and orientation error in radians.
  Scalar position_error{0.0};
  Scalar orientation_error{0.0};
  /// Smallest manipulability seen along the way. Near zero means the solve
  /// passed close to a singularity, which is worth knowing even when it
  /// converged.
  Scalar least_manipulability{0.0};
  /// Largest single joint step the solver commanded, radians. This is the
  /// number damping exists to bound.
  Scalar largest_step{0.0};
};

/// Settings for the iterative solve.
struct IkOptions {
  Scalar position_tolerance{1e-6};     ///< metres
  Scalar orientation_tolerance{1e-6};  ///< radians
  std::size_t max_iterations{100};

  /// Damping factor, in the same units as the Jacobian.
  ///
  /// **Not optional, and zero is not a neutral choice.** With no damping the
  /// step is the Moore-Penrose pseudo-inverse, which near a singularity asks
  /// for joint rates that go to infinity as the manipulability goes to zero --
  /// a tool motion of a millimetre commanding radians of joint travel. Damping
  /// trades a little tracking error for a bounded step, and the trade is
  /// measured in KinematicsSingularity.DampingBoundsTheStepThatOtherwiseDiverges.
  Scalar damping{1e-3};

  /// Clamp on any single joint step, radians. A second bound, because damping
  /// is a soft limit and a solver handed an unreachable target will otherwise
  /// take a large first step in a plausible direction.
  Scalar max_step{0.5};

  /// Respect the joints' own limits, clamping each iterate into range.
  bool enforce_joint_limits{true};
};

/// An open serial chain of revolute joints ending in a tool.
///
/// Forward kinematics is a product of exponentials:
///
///     base_T_tool(q) = T1(q1) * T2(q2) * ... * Tn(qn) * base_T_tool(0)
///
/// where each Ti is the screw motion of joint i about its own axis, expressed
/// in the base frame at the zero configuration. For a revolute joint that
/// screw is just "translate to a point on the axis, rotate, translate back",
/// which is why this needs no convention beyond the one already used for SE3.
///
/// **Threading**: const methods are pure and reentrant. Building a chain is
/// not concurrent with anything.
class SerialChain {
 public:
  constexpr SerialChain() noexcept = default;

  /// Builds a chain from its joints and the tool pose at zero.
  ///
  /// Axes are normalised here, once, so no later call has to wonder.
  static Expected<SerialChain, KinematicsError> build(
      std::span<const RevoluteJoint> joints, const SE3& base_T_tool_at_zero);

  [[nodiscard]] constexpr std::size_t jointCount() const noexcept { return count_; }

  /// Pose of the tool in the base frame at configuration `q`.
  [[nodiscard]] Expected<SE3, KinematicsError> forward(
      std::span<const Scalar> q) const noexcept;

  /// Geometric Jacobian in the base frame, row-major, 6 x jointCount().
  ///
  /// Rows 0..2 map joint rates to the linear velocity of the tool *point*;
  /// rows 3..5 map them to angular velocity. Both are expressed in the base
  /// frame, which is the convention the error twist in inverse() uses too --
  /// mixing the two frames is the classic way to get a solver that converges
  /// for some targets and spirals for others.
  [[nodiscard]] KinematicsError jacobian(std::span<const Scalar> q,
                                         std::span<Scalar> out) const noexcept;

  /// Yoshikawa's measure, sqrt(det(J * J^T)).
  ///
  /// Zero exactly at a singularity, and small near one. It has units and is not
  /// comparable between arms of different size, which is why it is reported
  /// rather than compared against a threshold anyone would have to invent.
  [[nodiscard]] Expected<Scalar, KinematicsError> manipulability(
      std::span<const Scalar> q) const noexcept;

  /// Solves for joint angles reaching `base_T_target`, starting from `q`.
  ///
  /// `q` is both the seed and the output. A seed matters: a 6R arm has up to
  /// eight configurations reaching the same pose, and an iterative solver
  /// returns whichever one it falls into. Seeding from the current
  /// configuration is what keeps the arm from turning itself inside out
  /// between two nearby waypoints.
  [[nodiscard]] Expected<IkReport, KinematicsError> inverse(
      const SE3& base_T_target, std::span<Scalar> q,
      const IkOptions& options = {}) const noexcept;

  [[nodiscard]] const RevoluteJoint& joint(std::size_t index) const noexcept {
    return joints_[index];
  }

  /// A six-axis arm with a spherical wrist, roughly the proportions of a small
  /// industrial robot. Provided so tests and examples argue about one concrete
  /// machine rather than each inventing their own.
  static SerialChain sixAxisExample();

 private:
  std::array<RevoluteJoint, kMaxJoints> joints_{};
  SE3 base_T_tool_at_zero_;
  std::size_t count_{0};
};

}  // namespace motionkit
