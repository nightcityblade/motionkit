// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/dynamics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

bool allFinite(std::span<const Scalar> values) noexcept {
  return std::ranges::all_of(values, [](Scalar v) { return std::isfinite(v); });
}

/// The three eigenvalues of a symmetric 3x3 matrix, ascending.
///
/// Closed form rather than iterative: a symmetric 3x3 characteristic polynomial
/// is a cubic whose roots are all real, so the trigonometric solution applies
/// and terminates. Used to validate an inertia tensor, where the quantities
/// that matter -- the principal moments -- are exactly these.
std::array<Scalar, 3> symmetricEigenvalues(const Mat3& m) noexcept {
  const Scalar off = m(0, 1) * m(0, 1) + m(0, 2) * m(0, 2) + m(1, 2) * m(1, 2);
  if (off <= 0.0) {
    std::array<Scalar, 3> diagonal{m(0, 0), m(1, 1), m(2, 2)};
    std::ranges::sort(diagonal);
    return diagonal;
  }
  const Scalar mean = m.trace() / 3.0;
  const Scalar spread =
      ((m(0, 0) - mean) * (m(0, 0) - mean) + (m(1, 1) - mean) * (m(1, 1) - mean) +
       (m(2, 2) - mean) * (m(2, 2) - mean) + 2.0 * off) /
      6.0;
  const Scalar scale = std::sqrt(spread);
  Mat3 shifted = m;
  for (std::size_t d = 0; d < 3; ++d) {
    shifted(d, d) -= mean;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      shifted(i, j) /= scale;
    }
  }
  // Clamped because rounding can push the cosine a hair outside [-1, 1], and
  // acos of 1.0000000000000002 is NaN -- which would turn a valid inertia into
  // a rejected one at random.
  const Scalar half_det = std::clamp(shifted.determinant() / 2.0, -1.0, 1.0);
  const Scalar phi = std::acos(half_det) / 3.0;
  const Scalar largest = mean + 2.0 * scale * std::cos(phi);
  const Scalar smallest =
      mean + 2.0 * scale * std::cos(phi + 2.0 * std::numbers::pi_v<Scalar> / 3.0);
  return {smallest, 3.0 * mean - largest - smallest, largest};
}

/// Whether an inertia tensor could belong to a real body.
///
/// Three conditions, and all three catch a different mistake. Symmetry catches
/// a transposed or half-filled tensor. Positive principal moments catch a sign
/// error or a tensor built about the wrong point. The triangle inequality --
/// no principal moment exceeding the sum of the other two -- catches the ones
/// that pass both other tests and still describe nothing: mass cannot be
/// distributed so as to make one axis harder to spin than the other two
/// together.
bool inertiaIsPlausible(const Mat3& inertia) noexcept {
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      if (!std::isfinite(inertia(i, j))) {
        return false;
      }
    }
  }
  const Scalar scale = std::max(std::abs(inertia.trace()), Scalar{1.0});
  const Scalar symmetry_tolerance = 1e-9 * scale;
  if (std::abs(inertia(0, 1) - inertia(1, 0)) > symmetry_tolerance ||
      std::abs(inertia(0, 2) - inertia(2, 0)) > symmetry_tolerance ||
      std::abs(inertia(1, 2) - inertia(2, 1)) > symmetry_tolerance) {
    return false;
  }
  const std::array<Scalar, 3> moments = symmetricEigenvalues(inertia);
  if (moments[0] <= 0.0) {
    return false;
  }
  // Scaled, because the inequality is exactly tight for a planar body and a
  // strict test would reject a thin plate for rounding.
  return moments[2] <= moments[0] + moments[1] + 1e-9 * scale;
}

/// The inertia of a body about `about`, given its inertia about its own centre
/// of mass. The parallel-axis theorem, in tensor form.
Mat3 shiftInertia(const Mat3& about_com, Scalar mass, const Vec3& com,
                  const Vec3& about) noexcept {
  const Vec3 offset = com - about;
  // Indexed through a local array rather than Vec3::operator[], which throws
  // out of range and so has no business inside a noexcept recursion.
  const std::array<Scalar, 3> d{offset.x, offset.y, offset.z};
  const Scalar dd = offset.squaredNorm();
  Mat3 shifted = about_com;
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      const Scalar kronecker = (i == j) ? 1.0 : 0.0;
      shifted(i, j) += mass * ((dd * kronecker) - (d[i] * d[j]));
    }
  }
  return shifted;
}

/// Element-wise sum. Mat3 has no operator+, and adding one would widen the
/// public surface for a single internal use.
Mat3 sum(const Mat3& a, const Mat3& b) noexcept {
  Mat3 total;
  for (std::size_t i = 0; i < 9; ++i) {
    total.d[i] = a.d[i] + b.d[i];
  }
  return total;
}

/// `rotation * inertia * rotation^T` -- the same body, seen from a frame the
/// rotation carries it into.
Mat3 rotateInertia(const Mat3& rotation, const Mat3& inertia) noexcept {
  return rotation * inertia * rotation.transpose();
}

}  // namespace

std::string_view toString(DynamicsError error) noexcept {
  switch (error) {
    case DynamicsError::None:
      return "None";
    case DynamicsError::SizeMismatch:
      return "SizeMismatch";
    case DynamicsError::NonFiniteInput:
      return "NonFiniteInput";
    case DynamicsError::NonPositiveMass:
      return "NonPositiveMass";
    case DynamicsError::ImplausibleInertia:
      return "ImplausibleInertia";
  }
  return "Unknown";
}

namespace {

/// Everything about the links that depends on the configuration, in the base
/// frame. Computed once per call and shared by both recursions, because the
/// two would otherwise derive the same joint axes by different routes.
struct LinkFrames {
  /// Joint axis, unit, at this configuration.
  std::array<Vec3, kMaxJoints> axis{};
  /// A point on the joint axis, at this configuration.
  std::array<Vec3, kMaxJoints> point{};
  /// Centre of mass of the link the joint moves.
  std::array<Vec3, kMaxJoints> com{};
  /// Inertia about that centre of mass, in base-frame axes.
  std::array<Mat3, kMaxJoints> inertia{};
};

DynamicsError computeLinkFrames(const SerialChain& chain,
                                std::span<const RigidBody> bodies, std::size_t count,
                                std::span<const Scalar> q, LinkFrames& out) noexcept {
  std::array<SE3, kMaxJoints> carried{};
  const KinematicsError error =
      chain.linkTransforms(q, std::span<SE3>{carried.data(), count});
  if (error == KinematicsError::NonFiniteInput) {
    return DynamicsError::NonFiniteInput;
  }
  if (error != KinematicsError::None) {
    return DynamicsError::SizeMismatch;
  }
  for (std::size_t i = 0; i < count; ++i) {
    // A joint is carried by everything upstream of it but not by itself, so
    // its axis comes from the previous link's transform and its body's from
    // its own.
    const SE3 upstream = (i == 0) ? SE3{} : carried[i - 1];
    out.axis[i] = upstream.rotateVector(chain.joint(i).axis);
    out.point[i] = upstream * chain.joint(i).point;
    out.com[i] = carried[i] * bodies[i].center_of_mass;
    out.inertia[i] = rotateInertia(carried[i].rotation().matrix(), bodies[i].inertia);
  }
  return DynamicsError::None;
}

}  // namespace

Expected<DynamicChain, DynamicsError> DynamicChain::build(
    const SerialChain& chain, std::span<const RigidBody> bodies) {
  if (bodies.size() != chain.jointCount() || chain.jointCount() == 0) {
    return {DynamicChain{}, DynamicsError::SizeMismatch};
  }
  DynamicChain built;
  built.chain_ = chain;
  built.count_ = chain.jointCount();
  for (std::size_t i = 0; i < built.count_; ++i) {
    if (!std::isfinite(bodies[i].mass) || bodies[i].mass <= 0.0) {
      return {DynamicChain{}, DynamicsError::NonPositiveMass};
    }
    if (!std::isfinite(bodies[i].center_of_mass.squaredNorm())) {
      return {DynamicChain{}, DynamicsError::NonFiniteInput};
    }
    if (!inertiaIsPlausible(bodies[i].inertia)) {
      return {DynamicChain{}, DynamicsError::ImplausibleInertia};
    }
    built.bodies_[i] = bodies[i];
  }
  return {built, DynamicsError::None};
}

DynamicsError DynamicChain::inverseDynamics(std::span<const Scalar> q,
                                            std::span<const Scalar> qd,
                                            std::span<const Scalar> qdd,
                                            std::span<Scalar> tau) const noexcept {
  if (q.size() != count_ || qd.size() != count_ || qdd.size() != count_ ||
      tau.size() < count_) {
    return DynamicsError::SizeMismatch;
  }
  if (!allFinite(q) || !allFinite(qd) || !allFinite(qdd)) {
    return DynamicsError::NonFiniteInput;
  }

  LinkFrames frames;
  if (const DynamicsError error = computeLinkFrames(
          chain_, std::span<const RigidBody>{bodies_.data(), count_}, count_, q, frames);
      error != DynamicsError::None) {
    return error;
  }

  std::array<Vec3, kMaxJoints> omega{};
  std::array<Vec3, kMaxJoints> alpha{};
  std::array<Vec3, kMaxJoints> com_acceleration{};

  // Forward: velocity and acceleration outward from the base. The base is
  // given an acceleration of -gravity, which is the whole of how weight enters
  // this routine -- see the class note.
  Vec3 previous_omega;
  Vec3 previous_alpha;
  Vec3 previous_point;
  Vec3 previous_point_acceleration = -gravity_;
  for (std::size_t i = 0; i < count_; ++i) {
    const Vec3 arm = frames.point[i] - previous_point;
    const Vec3 point_acceleration = previous_point_acceleration +
                                    previous_alpha.cross(arm) +
                                    previous_omega.cross(previous_omega.cross(arm));

    const Vec3 joint_rate = frames.axis[i] * qd[i];
    omega[i] = previous_omega + joint_rate;
    // The Coriolis term. It is the only place the product of two joint rates
    // appears, and dropping it is the classic way to get dynamics that are
    // right at low speed and wrong exactly when they matter.
    alpha[i] =
        previous_alpha + (frames.axis[i] * qdd[i]) + previous_omega.cross(joint_rate);

    const Vec3 to_com = frames.com[i] - frames.point[i];
    com_acceleration[i] = point_acceleration + alpha[i].cross(to_com) +
                          omega[i].cross(omega[i].cross(to_com));

    previous_omega = omega[i];
    previous_alpha = alpha[i];
    previous_point = frames.point[i];
    previous_point_acceleration = point_acceleration;
  }

  // Backward: forces inward from the tip. `force` and `moment` carry what the
  // link beyond this one exerts, so they hold zero at the tool and the whole
  // chain's reaction by the time the loop reaches the base.
  Vec3 force;
  Vec3 moment;
  for (std::size_t i = count_; i-- > 0;) {
    const Vec3 inertial_force = bodies_[i].mass * com_acceleration[i];
    const Vec3 inertial_moment =
        (frames.inertia[i] * alpha[i]) + omega[i].cross(frames.inertia[i] * omega[i]);

    Vec3 next_moment = moment;
    if (i + 1 < count_) {
      next_moment += (frames.point[i + 1] - frames.point[i]).cross(force);
    }
    next_moment += inertial_moment;
    next_moment += (frames.com[i] - frames.point[i]).cross(inertial_force);

    force += inertial_force;
    moment = next_moment;
    // Only the component along the axis is felt by the joint. The rest is
    // carried by the bearing, which is a mechanical engineer's problem.
    tau[i] = moment.dot(frames.axis[i]);
  }
  return DynamicsError::None;
}

DynamicsError DynamicChain::gravityTorque(std::span<const Scalar> q,
                                          std::span<Scalar> tau) const noexcept {
  // Checked here rather than left to inverseDynamics, because the zero span
  // below is sized from q and would run past the array before the callee ever
  // saw it.
  if (q.size() != count_) {
    return DynamicsError::SizeMismatch;
  }
  const std::array<Scalar, kMaxJoints> still{};
  const std::span<const Scalar> zero{still.data(), count_};
  return inverseDynamics(q, zero, zero, tau);
}

DynamicsError DynamicChain::massMatrix(std::span<const Scalar> q,
                                       std::span<Scalar> out) const noexcept {
  if (q.size() != count_ || out.size() < count_ * count_) {
    return DynamicsError::SizeMismatch;
  }
  if (!allFinite(q)) {
    return DynamicsError::NonFiniteInput;
  }

  LinkFrames frames;
  if (const DynamicsError error = computeLinkFrames(
          chain_, std::span<const RigidBody>{bodies_.data(), count_}, count_, q, frames);
      error != DynamicsError::None) {
    return error;
  }

  // Composite of everything from the current joint outward, accumulated as the
  // loop walks inward. Starts empty at the tip and ends as the whole arm.
  Scalar composite_mass = 0.0;
  Vec3 composite_com;
  Mat3 composite_inertia = Mat3::zero();

  for (std::size_t i = count_; i-- > 0;) {
    const Scalar mass = bodies_[i].mass;
    const Scalar total = composite_mass + mass;
    const Vec3 merged_com =
        ((composite_com * composite_mass) + (frames.com[i] * mass)) / total;
    // Both parts move to the merged centre of mass before they can be added:
    // inertia tensors are only additive about a common point.
    composite_inertia =
        sum(shiftInertia(composite_inertia, composite_mass, composite_com, merged_com),
            shiftInertia(frames.inertia[i], mass, frames.com[i], merged_com));
    composite_mass = total;
    composite_com = merged_com;

    // Momentum of that composite body when joint i turns at unit rate and
    // every other joint is held. Its projection onto joint j's axis is the
    // torque joint j feels, which is the definition of M(j, i).
    const Vec3 lever = composite_com - frames.point[i];
    const Vec3 linear = composite_mass * frames.axis[i].cross(lever);
    const Vec3 angular = (composite_inertia * frames.axis[i]) + lever.cross(linear);

    out[(i * count_) + i] = frames.axis[i].dot(angular);
    for (std::size_t j = 0; j < i; ++j) {
      const Vec3 about_j = angular + (frames.point[i] - frames.point[j]).cross(linear);
      const Scalar coupling = frames.axis[j].dot(about_j);
      // Written to both halves from one computation. Symmetry is a theorem
      // here, not something to be re-derived and then asserted.
      out[(j * count_) + i] = coupling;
      out[(i * count_) + j] = coupling;
    }
  }
  return DynamicsError::None;
}

Expected<Scalar, DynamicsError> DynamicChain::kineticEnergy(
    std::span<const Scalar> q, std::span<const Scalar> qd) const noexcept {
  if (qd.size() != count_) {
    return {0.0, DynamicsError::SizeMismatch};
  }
  std::array<Scalar, kMaxJoints * kMaxJoints> mass{};
  if (const DynamicsError error =
          massMatrix(q, std::span<Scalar>{mass.data(), count_ * count_});
      error != DynamicsError::None) {
    return {0.0, error};
  }
  if (!allFinite(qd)) {
    return {0.0, DynamicsError::NonFiniteInput};
  }
  Scalar energy = 0.0;
  for (std::size_t i = 0; i < count_; ++i) {
    for (std::size_t j = 0; j < count_; ++j) {
      energy += qd[i] * mass[(i * count_) + j] * qd[j];
    }
  }
  return {0.5 * energy, DynamicsError::None};
}

Expected<Scalar, DynamicsError> DynamicChain::potentialEnergy(
    std::span<const Scalar> q) const noexcept {
  if (q.size() != count_) {
    return {0.0, DynamicsError::SizeMismatch};
  }
  if (!allFinite(q)) {
    return {0.0, DynamicsError::NonFiniteInput};
  }
  LinkFrames frames;
  if (const DynamicsError error = computeLinkFrames(
          chain_, std::span<const RigidBody>{bodies_.data(), count_}, count_, q, frames);
      error != DynamicsError::None) {
    return {0.0, error};
  }
  Scalar energy = 0.0;
  for (std::size_t i = 0; i < count_; ++i) {
    energy -= bodies_[i].mass * gravity_.dot(frames.com[i]);
  }
  return {energy, DynamicsError::None};
}

DynamicChain DynamicChain::sixAxisExample() {
  // Each link is treated as a uniform cylinder along the arm's z axis at the
  // zero configuration: transverse moment m(3r^2 + L^2)/12, axial moment
  // m r^2 / 2. Crude, but crude in a way that is stated -- and unlike an
  // invented tensor it cannot violate the triangle inequality, because it
  // came from a body that exists.
  struct Link {
    Scalar mass;
    Scalar height;  // centre of mass along z, metres
    Scalar length;
    Scalar radius;
  };
  // Masses fall off toward the wrist, as they do on a real machine: the motors
  // that move the arm sit as close to the base as the designer could manage.
  constexpr std::array<Link, 6> kLinks{{
      {8.0, 0.075, 0.15, 0.090},
      {6.0, 0.350, 0.43, 0.070},
      {3.5, 0.750, 0.39, 0.060},
      {1.2, 0.950, 0.10, 0.050},
      {0.8, 0.990, 0.08, 0.045},
      {0.5, 1.030, 0.08, 0.040},
  }};

  std::array<RigidBody, 6> bodies{};
  for (std::size_t i = 0; i < kLinks.size(); ++i) {
    const Link& link = kLinks[i];
    const Scalar transverse =
        link.mass * ((3.0 * link.radius * link.radius) + (link.length * link.length)) /
        12.0;
    const Scalar axial = 0.5 * link.mass * link.radius * link.radius;
    Mat3 inertia = Mat3::zero();
    inertia(0, 0) = transverse;
    inertia(1, 1) = transverse;
    inertia(2, 2) = axial;
    bodies[i] = RigidBody{link.mass, Vec3{0.0, 0.0, link.height}, inertia};
  }
  // build() fails only on masses and inertias, and these are literals.
  return build(SerialChain::sixAxisExample(), bodies).value;
}

}  // namespace motionkit
