// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// Standard gravity, m/s^2. The magnitude, not a direction.
inline constexpr Scalar kStandardGravity = 9.80665;

/// Why a dynamics call could not answer.
enum class DynamicsError : std::uint8_t {
  None = 0,
  /// A joint count that does not match the chain, or a buffer too small.
  SizeMismatch,
  /// A NaN or infinite input.
  NonFiniteInput,
  /// A body with non-positive mass. A link that weighs nothing is not a
  /// simplification; it makes the mass matrix singular and the arm
  /// unsimulatable.
  NonPositiveMass,
  /// An inertia tensor that is not symmetric, or not positive definite, or
  /// that violates the triangle inequality on its principal moments. All three
  /// describe a body that cannot exist.
  ImplausibleInertia,
};

/// A human-readable name for a DynamicsError.
std::string_view toString(DynamicsError error) noexcept;

/// The mass properties of one link.
///
/// Described the same way joints are: **in the base frame, with every joint at
/// zero**. The centre of mass is a point in that frame, and the inertia tensor
/// is taken about the centre of mass with axes parallel to that frame. Both are
/// what a CAD package reports when asked for mass properties of an assembly in
/// its own global frame, which is the point -- the alternative convention, a
/// per-link body frame, needs the link frames to be agreed first and those are
/// exactly what ADR-0008 declined to require.
///
/// Body `i` is the link that joint `i` moves. The fixed base is not a body
/// here: it never accelerates, so it contributes no inertial force and no
/// torque any joint can feel.
struct RigidBody {
  /// Mass in kilograms. Must be strictly positive.
  Scalar mass{1.0};
  /// Centre of mass, in the base frame at the zero configuration.
  Vec3 center_of_mass;
  /// Inertia tensor about the centre of mass, in base-frame axes at the zero
  /// configuration, in kg m^2. Must be symmetric and positive definite.
  Mat3 inertia{Mat3::identity()};
};

/// A serial chain with mass, and the equations of motion that follow.
///
/// Solves the rigid-body equation
///
///     tau = M(q) qdd + C(q, qd) qd + g(q)
///
/// without ever forming `C`. inverseDynamics() evaluates the whole right-hand
/// side at once by recursive Newton-Euler, which is O(n) and needs no matrix;
/// the Coriolis matrix exists mainly in textbooks, is not unique, and costs
/// more to build than the answer it helps compute.
///
/// **Frames.** Everything is in the base frame. The recursion carries each
/// link's angular velocity, angular acceleration and centre-of-mass
/// acceleration forward from the base, then the forces backward from the tip,
/// all expressed in one frame throughout. The textbook formulation instead
/// rotates every quantity into each link's own frame, which saves arithmetic
/// and costs the reader the ability to check any intermediate value against a
/// drawing.
///
/// **Gravity is not a special case.** It enters by initialising the base
/// acceleration to `-gravity` rather than by adding a weight term to each
/// link. A base accelerating upward at 9.81 m/s^2 is indistinguishable, from
/// inside, from a base at rest in a gravitational field -- so one line at the
/// top of the recursion replaces a term in every link's force balance.
///
/// **Threading**: const methods are pure and reentrant.
class DynamicChain {
 public:
  /// An empty chain with no links.
  constexpr DynamicChain() noexcept = default;

  /// Builds from a kinematic chain and one body per joint.
  ///
  /// Rejects masses that are not positive and inertia tensors that are not
  /// symmetric positive definite or that break the triangle inequality on
  /// their principal moments. Those checks are here rather than at the call
  /// site because an implausible inertia produces plausible-looking torques,
  /// and a number that is merely wrong is worse than one that is refused.
  static Expected<DynamicChain, DynamicsError> build(const SerialChain& chain,
                                                     std::span<const RigidBody> bodies);

  /// The kinematic chain these bodies hang on.
  [[nodiscard]] const SerialChain& chain() const noexcept { return chain_; }

  /// The number of joints, and so of bodies.
  [[nodiscard]] constexpr std::size_t jointCount() const noexcept { return count_; }

  /// The body at `index`, as supplied to build(). Unchecked.
  [[nodiscard]] const RigidBody& body(std::size_t index) const noexcept {
    return bodies_[index];
  }

  /// The gravitational field in the base frame, m/s^2.
  ///
  /// Defaults to standard gravity along -Z, the usual convention for a robot
  /// bolted to a floor. Set it to zero to compute dynamics without weight, or
  /// rotate it for an arm mounted on a wall or a ceiling -- a case worth
  /// making representable, because it is normally discovered on site.
  [[nodiscard]] Vec3 gravity() const noexcept { return gravity_; }

  /// Sets the gravitational field in the base frame.
  void setGravity(const Vec3& gravity) noexcept { gravity_ = gravity; }

  /// Joint torques that produce the motion `(q, qd, qdd)`, including gravity.
  ///
  /// Recursive Newton-Euler, O(n), allocation-free and non-throwing, so it is
  /// callable from the cyclic task. `tau` must hold jointCount() entries and
  /// is written in newton-metres.
  ///
  /// This is the *inverse* problem -- given a motion, what torque causes it --
  /// which is the one a controller actually asks. The forward problem, given a
  /// torque what motion follows, needs the mass matrix factorised and is what
  /// a simulator asks; massMatrix() is the half of it this library provides.
  [[nodiscard]] DynamicsError inverseDynamics(std::span<const Scalar> q,
                                              std::span<const Scalar> qd,
                                              std::span<const Scalar> qdd,
                                              std::span<Scalar> tau) const noexcept;

  /// Joint torques that merely hold the arm still at `q`.
  ///
  /// Exactly `inverseDynamics(q, 0, 0)`, named because it is the term a
  /// position controller needs as a feed-forward and the one that decides
  /// whether an arm sags when its brakes release.
  [[nodiscard]] DynamicsError gravityTorque(std::span<const Scalar> q,
                                            std::span<Scalar> tau) const noexcept;

  /// The joint-space mass matrix at `q`, row-major, jointCount() squared.
  ///
  /// Computed by the composite-rigid-body algorithm: for each joint, the
  /// momentum of everything distal to it moving as one frozen body. O(n^2),
  /// allocation-free, and independent of the recursion inverseDynamics() uses
  /// -- which is what makes the two a genuine cross-check of each other rather
  /// than two spellings of one derivation.
  ///
  /// The result is symmetric and positive definite for any physical chain, at
  /// every configuration including singular ones. Unlike the Jacobian, it does
  /// not degenerate at a singularity: an arm at full stretch is still hard to
  /// accelerate in every joint.
  [[nodiscard]] DynamicsError massMatrix(std::span<const Scalar> q,
                                         std::span<Scalar> out) const noexcept;

  /// Kinetic energy at `(q, qd)`, in joules.
  ///
  /// Half of `qd' M(q) qd`. Positive for any non-zero motion, which is the
  /// physical statement of the mass matrix being positive definite.
  [[nodiscard]] Expected<Scalar, DynamicsError> kineticEnergy(
      std::span<const Scalar> q, std::span<const Scalar> qd) const noexcept;

  /// Potential energy at `q`, in joules, relative to the base origin.
  ///
  /// Only differences are meaningful, since the datum is arbitrary. Its
  /// gradient with respect to `q` is the gravity torque, which is how
  /// gravityTorque() is checked against a derivation that shares no code
  /// with it.
  [[nodiscard]] Expected<Scalar, DynamicsError> potentialEnergy(
      std::span<const Scalar> q) const noexcept;

  /// The six-axis example arm from SerialChain, given plausible link masses.
  ///
  /// Roughly a 20 kg machine: a heavy shoulder, a lighter forearm, a small
  /// wrist. Provided so tests and examples argue about one concrete robot
  /// rather than each inventing their own.
  static DynamicChain sixAxisExample();

 private:
  SerialChain chain_;
  std::array<RigidBody, kMaxJoints> bodies_{};
  Vec3 gravity_{0.0, 0.0, -kStandardGravity};
  std::size_t count_{0};
};

}  // namespace motionkit
