// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/cartesian.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/so3.hpp"
#include "motionkit/core/trajectory.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

std::string_view toString(CartesianError error) noexcept {
  switch (error) {
    case CartesianError::None:
      return "None";
    case CartesianError::SizeMismatch:
      return "SizeMismatch";
    case CartesianError::NonFiniteInput:
      return "NonFiniteInput";
    case CartesianError::KnotCountUnsupported:
      return "KnotCountUnsupported";
    case CartesianError::WaypointCountUnsupported:
      return "WaypointCountUnsupported";
    case CartesianError::UnreachableAlongPath:
      return "UnreachableAlongPath";
    case CartesianError::JointLimitAlongPath:
      return "JointLimitAlongPath";
    case CartesianError::LimitsNotUsable:
      return "LimitsNotUsable";
  }
  return "Unknown";
}

CartesianPath CartesianPath::between(const SE3& start, const SE3& goal) noexcept {
  CartesianPath path;
  path.start_ = start;
  path.travel_ = goal.translation() - start.translation();
  path.goal_rotation_ = goal.rotation();
  path.length_ = path.travel_.norm();
  path.angle_ = start.rotation().angleTo(goal.rotation());
  return path;
}

SE3 CartesianPath::poseAt(Scalar s) const noexcept {
  const Scalar u = std::clamp(s, Scalar{0.0}, Scalar{1.0});
  return SE3{start_.rotation().slerp(goal_rotation_, u),
             start_.translation() + (travel_ * u)};
}

namespace {

bool poseIsFinite(const SE3& pose) noexcept {
  const SO3& r = pose.rotation();
  return std::isfinite(pose.translation().squaredNorm()) && std::isfinite(r.w()) &&
         std::isfinite(r.x()) && std::isfinite(r.y()) && std::isfinite(r.z());
}

}  // namespace

Expected<BlendedPath, CartesianError> BlendedPath::through(std::span<const SE3> waypoints,
                                                           Scalar radius) {
  const std::size_t count = waypoints.size();
  if (count < 2 || count > kMaxWaypoints) {
    return {BlendedPath{}, CartesianError::WaypointCountUnsupported};
  }
  for (const SE3& pose : waypoints) {
    if (!poseIsFinite(pose)) {
      return {BlendedPath{}, CartesianError::NonFiniteInput};
    }
  }

  std::array<Vec3, kMaxWaypoints> heading{};
  std::array<Scalar, kMaxWaypoints> reach{};
  Scalar total = 0.0;
  for (std::size_t i = 0; i + 1 < count; ++i) {
    const Vec3 leg = waypoints[i + 1].translation() - waypoints[i].translation();
    reach[i] = leg.norm();
    if (!(reach[i] > 0.0)) {
      // Two waypoints in the same place are only a problem when there is a
      // corner between them to round, because then there is no direction to
      // leave by. Between just two, it is a pure reorientation or a move to
      // where the arm already stands, and both are ordinary.
      if (count > 2) {
        return {BlendedPath{}, CartesianError::WaypointCountUnsupported};
      }
      heading[i] = Vec3{};
    } else {
      heading[i] = leg / reach[i];
    }
    total += reach[i];
  }

  // The path parameter runs on distance, which has nothing to measure when the
  // tool does not move. A pure reorientation then gets an even share per
  // segment instead -- not a length in disguise, which is why it is kept apart
  // from length_ and never mixed with it. Adding some multiple of an angle to
  // a distance would need a scale nobody can justify; see SE3::isApprox.
  const bool by_distance = total > 0.0;

  // A corner may take no more than half of either segment it touches, so two
  // adjacent blends can never reach past each other however large a radius the
  // caller asks for.
  std::array<Scalar, kMaxWaypoints> cut{};
  for (std::size_t k = 1; k + 1 < count; ++k) {
    cut[k] =
        std::min({std::max(radius, Scalar{0.0}), reach[k - 1] * 0.5, reach[k] * 0.5});
  }

  BlendedPath path;
  path.turns_ = count;
  for (std::size_t i = 0; i < count; ++i) {
    path.orientations_[i] = waypoints[i].rotation();
  }
  path.orientation_at_[0] = 0.0;

  Scalar measured = 0.0;
  for (std::size_t i = 0; i + 1 < count; ++i) {
    const Vec3 from = waypoints[i].translation() + (heading[i] * cut[i]);
    const Vec3 to = waypoints[i + 1].translation() - (heading[i] * cut[i + 1]);
    Piece straight;
    straight.from = from;
    straight.to = to;
    straight.begins_at = measured;
    straight.length = by_distance ? reach[i] - cut[i] - cut[i + 1] : 1.0;
    path.pieces_[path.pieces_used_++] = straight;
    path.length_ += reach[i] - cut[i] - cut[i + 1];
    measured += straight.length;

    if (i + 2 >= count) {
      break;
    }
    const Vec3 corner = waypoints[i + 1].translation();
    Piece bend;
    bend.curved = true;
    bend.from = to;
    bend.via = corner;
    bend.to = corner + (heading[i + 1] * cut[i + 1]);
    bend.begins_at = measured;
    // The control polygon bounds the arc and the chord is bounded by it, so
    // their average is within a fraction of a percent for the shallow turns a
    // blend actually makes. Exact Bezier arc length has no closed form, and s
    // only has to be monotone and smooth.
    bend.length = ((bend.from - corner).norm() + (bend.to - corner).norm() +
                   (bend.to - bend.from).norm()) *
                  0.5;
    path.pieces_[path.pieces_used_++] = bend;
    // The waypoint's orientation belongs where the path passes closest to it,
    // which is the middle of the corner that replaced it.
    path.orientation_at_[i + 1] = measured + (bend.length * 0.5);
    path.length_ += bend.length;
    measured += bend.length;

    const Vec3 midpoint = (bend.from + (corner * 2.0) + bend.to) * 0.25;
    path.corner_deviation_ = std::max(path.corner_deviation_, (midpoint - corner).norm());
  }
  path.orientation_at_[count - 1] = measured;
  path.measure_ = measured;
  return {path, CartesianError::None};
}

SE3 BlendedPath::poseAt(Scalar s) const noexcept {
  if (pieces_used_ == 0) {
    return SE3{};
  }
  const Scalar along = std::clamp(s, Scalar{0.0}, Scalar{1.0}) * measure_;

  std::size_t index = 0;
  for (std::size_t i = 0; i < pieces_used_; ++i) {
    if (pieces_[i].length > 0.0 && along >= pieces_[i].begins_at) {
      index = i;
    }
  }
  const Piece& piece = pieces_[index];
  const Scalar u =
      piece.length > 0.0
          ? std::clamp((along - piece.begins_at) / piece.length, Scalar{0.0}, Scalar{1.0})
          : 0.0;
  const Vec3 position = piece.curved ? (piece.from * ((1.0 - u) * (1.0 - u))) +
                                           (piece.via * (2.0 * (1.0 - u) * u)) +
                                           (piece.to * (u * u))
                                     : piece.from + ((piece.to - piece.from) * u);

  std::size_t leg = 0;
  for (std::size_t i = 0; i + 1 < turns_; ++i) {
    if (along >= orientation_at_[i]) {
      leg = i;
    }
  }
  const Scalar spread = orientation_at_[leg + 1] - orientation_at_[leg];
  const Scalar fraction =
      spread > 0.0
          ? std::clamp((along - orientation_at_[leg]) / spread, Scalar{0.0}, Scalar{1.0})
          : 0.0;
  return SE3{orientations_[leg].slerp(orientations_[leg + 1], fraction), position};
}

namespace {

/// One knot's worth of joint values inside the flat plan storage.
Scalar& at(std::array<Scalar, kMaxPathKnots * kMaxJoints>& knots, std::size_t knot,
           std::size_t joint) noexcept {
  return knots[(knot * kMaxJoints) + joint];
}

Scalar at(const std::array<Scalar, kMaxPathKnots * kMaxJoints>& knots, std::size_t knot,
          std::size_t joint) noexcept {
  return knots[(knot * kMaxJoints) + joint];
}

/// Solves the chain onto every knot of the path, seeding each from the last.
///
/// Seeding matters more here than anywhere else in the library. A 6R arm
/// reaches most poses in up to eight configurations, and an unseeded solve at
/// each knot is free to pick a different one from its neighbour -- which is a
/// plan that asks the arm to turn itself inside out midway along a straight
/// line, while every individual pose on it is correct.
CartesianError solveKnots(const SerialChain& chain, std::span<const Scalar> q_start,
                          const BlendedPath& path, std::span<const Scalar> schedule,
                          const IkOptions& options,
                          std::array<Scalar, kMaxPathKnots * kMaxJoints>& knots,
                          Scalar& least_manipulability) noexcept {
  const std::size_t joints = chain.jointCount();
  const std::size_t count = schedule.size();
  std::array<Scalar, kMaxJoints> seed{};
  for (std::size_t j = 0; j < joints; ++j) {
    seed[j] = q_start[j];
    at(knots, 0, j) = q_start[j];
  }
  least_manipulability = chain.manipulability(q_start).value;

  for (std::size_t k = 1; k < count; ++k) {
    const SE3 target = path.poseAt(schedule[k]);
    const std::span<Scalar> working{seed.data(), joints};
    const auto solved = chain.inverse(target, working, options);
    if (!solved) {
      // Which of the two it is matters to the caller: a joint out of travel is
      // an arm-configuration problem, and a failure to converge is a workspace
      // problem. Collapsing them sends people to fix the wrong thing.
      return solved.error == KinematicsError::OutsideJointLimits
                 ? CartesianError::JointLimitAlongPath
                 : CartesianError::UnreachableAlongPath;
    }
    for (std::size_t j = 0; j < joints; ++j) {
      at(knots, k, j) = seed[j];
    }
    least_manipulability =
        std::min(least_manipulability, solved.value.least_manipulability);
  }
  return CartesianError::None;
}

}  // namespace

namespace {

/// The limits the path parameter may be driven under, and what set them.
struct Pacing {
  MotionLimits limits;
  std::size_t binding_joint{0};
  Scalar binding_s{0.0};
  bool moves{false};
};

/// Joint displacement per unit path parameter, and its rate of change.
struct KnotSlope {
  Scalar first{0.0};
  Scalar second{0.0};
};

KnotSlope slopeAt(const std::array<Scalar, kMaxPathKnots * kMaxJoints>& knots,
                  std::size_t joint, std::size_t k, std::size_t count,
                  Scalar step) noexcept {
  KnotSlope slope;
  if (k == 0) {
    slope.first = (at(knots, 1, joint) - at(knots, 0, joint)) / step;
  } else if (k + 1 == count) {
    slope.first = (at(knots, k, joint) - at(knots, k - 1, joint)) / step;
  } else {
    slope.first = (at(knots, k + 1, joint) - at(knots, k - 1, joint)) / (2.0 * step);
  }
  // The second difference needs a neighbour on each side, so the ends borrow
  // the nearest interior value rather than reporting a curvature of zero --
  // which would let the plan corner hardest exactly where it measured least.
  const std::size_t interior = std::clamp<std::size_t>(k, 1, count - 2);
  slope.second = (at(knots, interior + 1, joint) - (2.0 * at(knots, interior, joint)) +
                  at(knots, interior - 1, joint)) /
                 (step * step);
  return slope;
}

/// Where along the path each knot sits.
///
/// Uniform in `s` is the obvious schedule and the wrong one. A single
/// badly-conditioned stretch caps the speed of the *whole* move, because one
/// profile drives one parameter and its limit is whatever the worst knot
/// requires. Spending the parameter unevenly -- densely where the arm is
/// struggling, sparsely where it is not -- makes the difficulty roughly
/// constant per unit of parameter, and then the single limit is no longer set
/// by a place the arm spends almost no time in.
///
/// The measure of difficulty is the reciprocal of the path speed each knot
/// permits, so the schedule is the normalised integral of that. Duration
/// becomes the integral of `ds / smax(s)` rather than `1 / min(smax)`, and the
/// first is never larger than the second.
void difficultySchedule(const std::array<Scalar, kMaxPathKnots * kMaxJoints>& knots,
                        std::size_t joints, std::size_t count,
                        std::span<const MotionLimits> joint_limits,
                        Scalar cornering_share, std::span<Scalar> out) noexcept {
  const auto span = static_cast<Scalar>(count - 1);
  const Scalar step = 1.0 / span;
  constexpr Scalar kStill = 1e-12;
  // A knot where nothing moves imposes no cost and must not divide by zero.
  constexpr Scalar kEasiest = 1e9;

  std::array<Scalar, kMaxPathKnots> effort{};
  for (std::size_t k = 0; k < count; ++k) {
    Scalar quickest = kEasiest;
    for (std::size_t j = 0; j < joints; ++j) {
      const KnotSlope slope = slopeAt(knots, j, k, count, step);
      const Scalar travel = std::abs(slope.first);
      if (travel > kStill) {
        quickest = std::min(quickest, joint_limits[j].max_velocity / travel);
      }
      // Cornering counts as difficulty too, and leaving it out is worse than
      // not reparameterising at all. Equalising only the velocity term makes
      // acceleration the binding constraint everywhere it was not already, and
      // on a route whose corners were already the limit the move comes out
      // *slower* than the schedule it replaced. Measured, before this was
      // added: a blended staircase went from 1.44 s to 1.80 s.
      const Scalar bend = std::abs(slope.second);
      if (bend > kStill) {
        quickest = std::min(quickest, std::sqrt(cornering_share *
                                                joint_limits[j].max_acceleration / bend));
      }
    }
    effort[k] = 1.0 / std::max(quickest, Scalar{1.0} / kEasiest);
  }

  std::array<Scalar, kMaxPathKnots> reached{};
  reached[0] = 0.0;
  for (std::size_t k = 1; k < count; ++k) {
    reached[k] = reached[k - 1] + ((effort[k - 1] + effort[k]) * 0.5 * step);
  }
  const Scalar total = reached[count - 1];
  if (!(total > 0.0)) {
    // Nothing moves anywhere. Any schedule is as good as any other, and the
    // uniform one is the one nothing downstream has to think about.
    for (std::size_t k = 0; k < count; ++k) {
      out[k] = static_cast<Scalar>(k) / span;
    }
    return;
  }

  // Invert: walk the cumulative effort and read off the s that reaches each
  // equal share of it.
  out[0] = 0.0;
  out[count - 1] = 1.0;
  std::size_t behind = 0;
  for (std::size_t m = 1; m + 1 < count; ++m) {
    const Scalar wanted = (static_cast<Scalar>(m) / span) * total;
    while (behind + 2 < count && reached[behind + 1] < wanted) {
      ++behind;
    }
    const Scalar gap = reached[behind + 1] - reached[behind];
    const Scalar fraction =
        gap > 0.0 ? std::clamp((wanted - reached[behind]) / gap, Scalar{0.0}, Scalar{1.0})
                  : 0.0;
    out[m] = (static_cast<Scalar>(behind) + fraction) * step;
  }
}

/// Turns per-joint limits into limits on the path parameter.
///
/// Velocity is the easy half: a joint moving `dq/ds` per unit path parameter
/// hits its own limit when the path is traversed at `v_max / |dq/ds|`, so the
/// path speed is the smallest such ratio anywhere on it.
///
/// Acceleration is the half that is usually got wrong. Joint acceleration has
/// two sources -- the path bending in joint space, which grows with the
/// *square* of path speed, and the profile changing speed along it -- and they
/// compete for one limit. Bounding only the second leaves a plan that respects
/// every acceleration limit on paper and exceeds them on a curve. Here the
/// budget is split explicitly, and the cornering half feeds back into the speed
/// bound rather than being discovered afterwards.
Pacing paceFromKnots(const std::array<Scalar, kMaxPathKnots * kMaxJoints>& knots,
                     std::size_t joints, std::size_t count,
                     std::span<const MotionLimits> joint_limits,
                     Scalar cornering_share) noexcept {
  Pacing pacing;
  const auto span = static_cast<Scalar>(count - 1);
  const Scalar step = 1.0 / span;
  constexpr Scalar kStill = 1e-12;

  Scalar speed = std::numeric_limits<Scalar>::infinity();
  Scalar acceleration = std::numeric_limits<Scalar>::infinity();
  Scalar jerk = std::numeric_limits<Scalar>::infinity();

  for (std::size_t k = 0; k < count; ++k) {
    for (std::size_t j = 0; j < joints; ++j) {
      const KnotSlope slope = slopeAt(knots, j, k, count, step);
      const Scalar travel = std::abs(slope.first);
      const Scalar bend = std::abs(slope.second);
      if (travel > kStill) {
        pacing.moves = true;
        const Scalar allowed = joint_limits[j].max_velocity / travel;
        if (allowed < speed) {
          speed = allowed;
          pacing.binding_joint = j;
          pacing.binding_s = static_cast<Scalar>(k) / span;
        }
        acceleration =
            std::min(acceleration,
                     (1.0 - cornering_share) * joint_limits[j].max_acceleration / travel);
        jerk = std::min(jerk, joint_limits[j].max_jerk / travel);
      }
      if (bend > kStill) {
        // |q''| * sdot^2 must fit inside the cornering share.
        const Scalar allowed =
            std::sqrt(cornering_share * joint_limits[j].max_acceleration / bend);
        if (allowed < speed) {
          speed = allowed;
          pacing.binding_joint = j;
          pacing.binding_s = static_cast<Scalar>(k) / span;
        }
      }
    }
  }

  pacing.limits = MotionLimits{speed, acceleration, jerk};
  return pacing;
}

}  // namespace

namespace {

/// Joint positions at path fraction `s`, by cubic Hermite between knots.
///
/// Not linear interpolation, and the reason is the acceleration limit. Linear
/// interpolation makes joint velocity piecewise constant, so it steps at every
/// knot crossing and joint acceleration is an impulse there -- unbounded, and
/// no acceleration budget computed anywhere can be honoured by a path shaped
/// like that. Hermite with the same finite-difference tangents the pacing uses
/// is C1, so velocity is continuous and acceleration is finite and testable.
///
/// The tangents are recomputed from neighbouring knots rather than stored,
/// which costs four reads and saves a second array the size of the first.
void interpolate(const std::array<Scalar, kMaxPathKnots * kMaxJoints>& knots,
                 std::size_t joints, std::size_t count, Scalar s,
                 std::span<Scalar> out) noexcept {
  const auto span = static_cast<Scalar>(count - 1);
  const Scalar step = 1.0 / span;
  const Scalar scaled = std::clamp(s, Scalar{0.0}, Scalar{1.0}) * span;
  auto index = static_cast<std::size_t>(scaled);
  if (index + 1 >= count) {
    index = count - 2;
  }
  const Scalar u = scaled - static_cast<Scalar>(index);
  const Scalar u2 = u * u;
  const Scalar u3 = u2 * u;
  const Scalar h00 = (2.0 * u3) - (3.0 * u2) + 1.0;
  const Scalar h10 = u3 - (2.0 * u2) + u;
  const Scalar h01 = (-2.0 * u3) + (3.0 * u2);
  const Scalar h11 = u3 - u2;

  for (std::size_t j = 0; j < joints; ++j) {
    const Scalar here = at(knots, index, j);
    const Scalar next = at(knots, index + 1, j);
    const Scalar tangent_here = slopeAt(knots, j, index, count, step).first;
    const Scalar tangent_next = slopeAt(knots, j, index + 1, count, step).first;
    out[j] = (h00 * here) + (h10 * step * tangent_here) + (h01 * next) +
             (h11 * step * tangent_next);
  }
}

/// How far the followed path leaves the straight line between knots.
///
/// Interpolation between knots is linear in *joint* space, and a straight line
/// in joint space is a curve in Cartesian space. The gap is largest halfway
/// between two knots, so that is where it is measured. Reporting it is the
/// honest alternative to claiming the tool travels in a straight line, which it
/// does only at the knots.
Scalar measureChordDeviation(const SerialChain& chain,
                             const std::array<Scalar, kMaxPathKnots * kMaxJoints>& knots,
                             std::size_t joints, std::size_t count,
                             std::span<const Scalar> schedule,
                             const BlendedPath& path) noexcept {
  Scalar worst = 0.0;
  const auto span = static_cast<Scalar>(count - 1);
  std::array<Scalar, kMaxJoints> middle{};
  for (std::size_t k = 0; k + 1 < count; ++k) {
    const Scalar between = (static_cast<Scalar>(k) + 0.5) / span;
    interpolate(knots, joints, count, between, std::span<Scalar>{middle.data(), joints});
    // Halfway between two knots in the parameter is halfway between their two
    // path fractions, which are not evenly spaced once difficulty decides them.
    const Scalar s = (schedule[k] + schedule[k + 1]) * 0.5;
    const auto pose = chain.forward(std::span<const Scalar>{middle.data(), joints});
    if (!pose) {
      continue;
    }
    const Scalar gap = (pose.value.translation() - path.poseAt(s).translation()).norm();
    worst = std::max(worst, gap);
  }
  return worst;
}

}  // namespace

namespace {

/// The checks that do not depend on how many waypoints there are.
CartesianError checkInputs(const SerialChain& chain, std::span<const Scalar> q_start,
                           std::span<const MotionLimits> joint_limits,
                           const CartesianOptions& options) noexcept {
  const std::size_t joints = chain.jointCount();
  if (joints == 0 || q_start.size() != joints || joint_limits.size() != joints) {
    return CartesianError::SizeMismatch;
  }
  if (options.knots < 3 || options.knots > kMaxPathKnots) {
    return CartesianError::KnotCountUnsupported;
  }
  if (!(options.cornering_share > 0.0) || !(options.cornering_share < 1.0)) {
    return CartesianError::LimitsNotUsable;
  }
  for (const MotionLimits& limit : joint_limits) {
    if (limit.validate() != TrajectoryError::None) {
      return CartesianError::LimitsNotUsable;
    }
  }
  return CartesianError::None;
}

}  // namespace

// A single straight move and a blended sequence differ only in the path they
// are given; the knot solve, the pacing and the profile are identical. Keeping
// them identical is deliberate -- a sequence that paced itself differently from
// a single move would be a second set of limits to be wrong about.
Expected<CartesianPlan, CartesianError> CartesianPlan::planAlong(
    const SerialChain& chain, std::span<const Scalar> q_start, const BlendedPath& path,
    std::span<const MotionLimits> joint_limits, const CartesianOptions& options) {
  const std::size_t joints = chain.jointCount();
  CartesianPlan built;
  built.joints_ = joints;
  built.path_ = path;
  built.report_.knots = options.knots;
  built.report_.waypoints = path.waypointCount();
  built.report_.corner_deviation = path.cornerDeviation();

  const auto span = static_cast<Scalar>(options.knots - 1);
  for (std::size_t k = 0; k < options.knots; ++k) {
    built.at_s_[k] = static_cast<Scalar>(k) / span;
  }
  const std::span<Scalar> schedule{built.at_s_.data(), options.knots};

  if (const CartesianError error =
          solveKnots(chain, q_start, built.path_, schedule, options.ik, built.knots_,
                     built.report_.least_manipulability);
      error != CartesianError::None) {
    return {CartesianPlan{}, error};
  }

  if (options.pace_by_difficulty) {
    // The first pass exists to find out where the difficulty is; the second
    // solves at the places that answer chose. Solving twice rather than
    // interpolating the first pass keeps every knot an exact inverse solution,
    // which is what the chord-deviation figure is measured against.
    std::array<Scalar, kMaxPathKnots> spread{};
    difficultySchedule(built.knots_, joints, options.knots, joint_limits,
                       options.cornering_share,
                       std::span<Scalar>{spread.data(), options.knots});
    for (std::size_t k = 0; k < options.knots; ++k) {
      built.at_s_[k] = spread[k];
    }
    if (const CartesianError error =
            solveKnots(chain, q_start, built.path_, schedule, options.ik, built.knots_,
                       built.report_.least_manipulability);
        error != CartesianError::None) {
      return {CartesianPlan{}, error};
    }
  }

  const Pacing pacing = paceFromKnots(built.knots_, joints, options.knots, joint_limits,
                                      options.cornering_share);
  built.report_.binding_joint = pacing.moves ? pacing.binding_joint : joints;
  built.report_.binding_s = pacing.binding_s;

  // A move whose endpoints coincide takes no time. Planning it under the
  // derived limits would be planning a unit path parameter for an arm that
  // does not move, and would report a duration for standing still.
  const MotionLimits limits = pacing.moves ? pacing.limits : MotionLimits{1.0, 1.0, 1.0};
  const Scalar goal_s = pacing.moves ? 1.0 : 0.0;
  const auto profile = ScurveProfile::plan(0.0, goal_s, limits);
  if (!profile) {
    return {CartesianPlan{}, CartesianError::LimitsNotUsable};
  }
  built.profile_ = profile.value;
  built.report_.chord_deviation = measureChordDeviation(
      chain, built.knots_, joints, options.knots, schedule, built.path_);
  return {built, CartesianError::None};
}

Expected<CartesianPlan, CartesianError> CartesianPlan::plan(
    const SerialChain& chain, std::span<const Scalar> q_start, const SE3& base_T_goal,
    std::span<const MotionLimits> joint_limits, const CartesianOptions& options) {
  const std::array<SE3, 1> only{base_T_goal};
  return planThrough(chain, q_start, only, joint_limits, options);
}

Expected<CartesianPlan, CartesianError> CartesianPlan::planThrough(
    const SerialChain& chain, std::span<const Scalar> q_start,
    std::span<const SE3> waypoints, std::span<const MotionLimits> joint_limits,
    const CartesianOptions& options) {
  if (const CartesianError error = checkInputs(chain, q_start, joint_limits, options);
      error != CartesianError::None) {
    return {CartesianPlan{}, error};
  }
  if (waypoints.empty() || waypoints.size() + 1 > kMaxWaypoints) {
    return {CartesianPlan{}, CartesianError::WaypointCountUnsupported};
  }
  for (const SE3& pose : waypoints) {
    if (!poseIsFinite(pose)) {
      return {CartesianPlan{}, CartesianError::NonFiniteInput};
    }
  }
  const auto start_pose = chain.forward(q_start);
  if (!start_pose) {
    return {CartesianPlan{}, CartesianError::NonFiniteInput};
  }

  // Where the arm is counts as the first waypoint. The caller supplies where to
  // go, not where it already stands.
  std::array<SE3, kMaxWaypoints> route{};
  route[0] = start_pose.value;
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    route[i + 1] = waypoints[i];
  }
  const auto path = BlendedPath::through(
      std::span<const SE3>{route.data(), waypoints.size() + 1}, options.blend_radius);
  if (!path) {
    return {CartesianPlan{}, path.error};
  }

  return planAlong(chain, q_start, path.value, joint_limits, options);
}

bool CartesianPlan::sample(Scalar t, std::span<Scalar> q) const noexcept {
  if (q.size() < joints_ || report_.knots < 2) {
    return false;
  }
  const Scalar s = profile_.sample(t).position;
  interpolate(knots_, joints_, report_.knots, s, q);
  return true;
}

SE3 CartesianPlan::poseAtTime(Scalar t) const noexcept {
  if (report_.knots < 2) {
    return path_.poseAt(0.0);
  }
  // The profile runs on knot index, not on distance along the path, so it has
  // to be read back through the schedule that decided where the knots went.
  const Scalar parameter =
      std::clamp(profile_.sample(t).position, Scalar{0.0}, Scalar{1.0});
  const auto span = static_cast<Scalar>(report_.knots - 1);
  const Scalar scaled = parameter * span;
  auto index = static_cast<std::size_t>(scaled);
  if (index + 1 >= report_.knots) {
    index = report_.knots - 2;
  }
  const Scalar fraction = scaled - static_cast<Scalar>(index);
  return path_.poseAt(at_s_[index] + (fraction * (at_s_[index + 1] - at_s_[index])));
}

}  // namespace motionkit
