// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/trajectory.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// Knots a Cartesian plan may hold. The plan stores a joint configuration per
/// knot, so this bounds its size and keeps sampling allocation-free.
inline constexpr std::size_t kMaxPathKnots = 65;

/// Why a Cartesian move could not be planned.
enum class CartesianError : std::uint8_t {
  None = 0,
  /// Joint counts, limit counts and buffer sizes that do not agree.
  SizeMismatch,
  /// A NaN or infinite input.
  NonFiniteInput,
  /// Fewer than three knots, or more than kMaxPathKnots. Two knots describe
  /// the endpoints and say nothing about the path between them.
  KnotCountUnsupported,
  /// Fewer than one waypoint, more than kMaxWaypoints, or two in a row at the
  /// same place. A repeated waypoint has no direction to leave by, so there is
  /// no corner to round and no honest way to guess one.
  WaypointCountUnsupported,
  /// The inverse solve did not converge somewhere along the path -- the line
  /// leaves the workspace, or passes too close to a singularity to solve.
  ///
  /// Reported rather than skipped: a plan that quietly omits the part of the
  /// line it could not reach is a plan that cuts a corner through whatever was
  /// standing there.
  UnreachableAlongPath,
  /// A joint would have run out of travel partway along the line.
  ///
  /// Distinct from UnreachableAlongPath because the fix is different. The pose
  /// is reachable and the line is fine; this arm cannot follow it *from this
  /// configuration*. Starting elsewhere, or remounting the workpiece, usually
  /// solves it -- and none of that is suggested by "unreachable".
  JointLimitAlongPath,
  /// A joint limit set that does not admit motion.
  LimitsNotUsable,
};

/// A human-readable name for a CartesianError.
std::string_view toString(CartesianError error) noexcept;

/// A straight line in space, with the orientation turning evenly along it.
///
/// Position interpolates linearly and orientation by SLERP, both on one
/// parameter `s` running 0 to 1. One parameter rather than two is the same
/// decision ADR-0006 makes for multi-axis moves: separate position and
/// orientation parameters would let the tool arrive pointing the wrong way and
/// then rotate on the spot.
class CartesianPath {
 public:
  /// An empty path of zero length, parked at the origin.
  CartesianPath() noexcept = default;

  /// The straight path from `start` to `goal`.
  static CartesianPath between(const SE3& start, const SE3& goal) noexcept;

  /// The pose at path fraction `s`, clamped to [0, 1].
  [[nodiscard]] SE3 poseAt(Scalar s) const noexcept;

  /// Straight-line distance from start to goal, in metres.
  [[nodiscard]] Scalar length() const noexcept { return length_; }

  /// Total rotation from start to goal, in radians, along the shorter arc.
  [[nodiscard]] Scalar sweptAngle() const noexcept { return angle_; }

 private:
  SE3 start_;
  Vec3 travel_;
  SO3 goal_rotation_;
  Scalar length_{0.0};
  Scalar angle_{0.0};
};

/// Waypoints a blended move may pass through, endpoints included.
inline constexpr std::size_t kMaxWaypoints = 16;

/// A path through several waypoints, with the corners rounded.
///
/// A sequence of straight moves planned one at a time stops at every waypoint,
/// because each ends at rest. Planning them as one path does not stop -- but a
/// tool cannot turn a sharp corner at speed either, since its velocity would
/// have to change direction instantaneously. Something has to give, and what
/// gives is the corner.
///
/// Each interior waypoint is replaced by a quadratic Bezier that leaves the
/// incoming line `radius` before the corner and rejoins the outgoing line
/// `radius` after it. The tool no longer passes through the waypoint, and
/// `cornerDeviation()` says by how much. That is the trade stated plainly:
/// **exact corners cost a full stop, and speed costs the corner.** A caller who
/// needs the waypoint hit exactly asks for radius zero and gets the stop.
///
/// A path through two waypoints has no interior corner and is a straight line,
/// which is why this type also serves the single-move case.
class BlendedPath {
 public:
  /// An empty path of zero length.
  BlendedPath() noexcept = default;

  /// Builds a path through `waypoints`, rounding interior corners by at most
  /// `radius` metres.
  ///
  /// The radius is reduced at any corner whose adjacent segments are too short
  /// to give it up -- never more than half of either -- so blends can never
  /// overlap or run past a neighbouring waypoint however large a radius is
  /// asked for. Requires at least two waypoints and no more than
  /// kMaxWaypoints.
  static Expected<BlendedPath, CartesianError> through(std::span<const SE3> waypoints,
                                                       Scalar radius);

  /// The pose at path fraction `s`, clamped to [0, 1].
  [[nodiscard]] SE3 poseAt(Scalar s) const noexcept;

  /// Total path length in metres, following the rounded corners rather than
  /// the sharp ones.
  [[nodiscard]] Scalar length() const noexcept { return length_; }

  /// The furthest any rounded corner falls from the waypoint it replaced, in
  /// metres. Zero for a straight path.
  [[nodiscard]] Scalar cornerDeviation() const noexcept { return corner_deviation_; }

  /// How many waypoints the path was built from.
  [[nodiscard]] constexpr std::size_t waypointCount() const noexcept { return turns_; }

 private:
  /// A straight run or a rounded corner. `via` is the Bezier control point and
  /// is unused when the piece is straight.
  struct Piece {
    Vec3 from;
    Vec3 via;
    Vec3 to;
    Scalar begins_at{0.0};
    Scalar length{0.0};
    bool curved{false};
  };

  std::array<Piece, 2 * kMaxWaypoints> pieces_{};
  std::array<SO3, kMaxWaypoints> orientations_{};
  /// Distance along the path at which each waypoint's orientation applies.
  std::array<Scalar, kMaxWaypoints> orientation_at_{};
  /// What the path parameter is measured in: the geometric length when the
  /// tool moves, and an even share per segment when it only turns.
  Scalar measure_{0.0};
  std::size_t pieces_used_{0};
  std::size_t turns_{0};
  Scalar length_{0.0};
  Scalar corner_deviation_{0.0};
};

/// How a Cartesian move should be discretised and solved.
struct CartesianOptions {
  /// Knots the path is sampled at, including both endpoints. More knots follow
  /// the line more closely and cost one inverse solve each.
  std::size_t knots{33};
  /// Settings for the inverse solve at each knot. Each knot is seeded from the
  /// one before, so the tolerances can be tighter than for a cold solve.
  IkOptions ik{};
  /// The share of each joint's acceleration budget reserved for turning the
  /// corner, as opposed to changing speed along the path.
  ///
  /// Joint acceleration has two sources: the path bending in joint space,
  /// which grows with the square of path speed, and the profile speeding up or
  /// slowing down. They compete for one limit, and something has to divide it.
  /// A half each is the symmetric choice and no measurement argues for
  /// another; raise it to corner faster and accelerate more gently, lower it
  /// for the reverse.
  Scalar cornering_share{0.5};
  /// Whether to spend the path parameter unevenly, densely where the arm is
  /// struggling and sparsely where it is not.
  ///
  /// One profile drives one parameter, so its speed limit is whatever the worst
  /// knot demands -- and with the parameter spread evenly in distance, one
  /// awkward stretch paces the entire move however short that stretch is.
  /// Spreading it by difficulty makes the limit describe the path as a whole.
  ///
  /// **Off by default, because it is not a universal win.** On a single
  /// straight move it is a large one, and the more awkward the move the larger:
  /// a 50 mm move through a badly-conditioned wrist falls from 8.15 s to
  /// 3.27 s, and an ordinary 250 mm move from 3.52 s to 2.23 s.
  ///
  /// On a route with blended corners it makes things **worse** -- 1.44 s to
  /// 2.31 s on a three-legged staircase. Blending has already arranged for the
  /// difficulty to sit at the corners, and concentrating the parameter there as
  /// well overshoots: the reparameterisation is computed from a curvature it
  /// then changes, and at a corner it manufactures more of it.
  ///
  /// It also costs path accuracy, because knots serve two masters. Spending
  /// them where the arm struggles takes them from where it does not, and the
  /// straight stretches are then followed less exactly: `chord_deviation` on
  /// that 250 mm move rises from 2.9e-05 m to 4.4e-04 m. Raising `knots` buys
  /// it back.
  ///
  /// And it doubles the planning cost, since the knots are solved once to find
  /// where the difficulty is and again where that answer put them.
  bool pace_by_difficulty{false};
  /// How far a rounded corner may cut inside an interior waypoint, in metres.
  ///
  /// Ignored by a move with no interior waypoints. Zero rounds nothing, which
  /// makes the path pass exactly through every waypoint and pays for it in the
  /// speed the corners then permit.
  Scalar blend_radius{0.05};
};

/// What the plan discovered about the move it just planned.
struct CartesianReport {
  /// The joint whose limit set the pace, or jointCount() when nothing bound.
  ///
  /// On a disappointing cycle time this names the joint to argue with, which
  /// on a Cartesian move is rarely the one anybody guesses.
  std::size_t binding_joint{0};
  /// Where along the path that joint bound, as a fraction.
  Scalar binding_s{0.0};
  /// The smallest manipulability seen at any knot.
  ///
  /// This is the number that explains a slow Cartesian move. Near a
  /// singularity a modest tool speed demands enormous joint speed, so the
  /// whole move is paced by the worst point on the line -- and the fix is
  /// usually to move the line, not to raise a limit.
  Scalar least_manipulability{0.0};
  /// The largest distance, in metres, between the true straight line and the
  /// path actually followed between knots.
  ///
  /// Interpolation between knots is linear in joint space, which is not
  /// straight in Cartesian space. Measured at knot midpoints, where the error
  /// is largest, so it is an honest bound on how far the tool leaves the line
  /// rather than a claim that it does not.
  Scalar chord_deviation{0.0};
  /// Knots actually used.
  std::size_t knots{0};
  /// Waypoints the path was built from, endpoints included.
  std::size_t waypoints{0};
  /// The furthest a rounded corner fell from the waypoint it replaced, in
  /// metres. Zero when nothing was rounded.
  ///
  /// Distinct from `chord_deviation`, which comes from discretising the path
  /// into knots and shrinks as knots are added. This one is deliberate and does
  /// not shrink: it is the corner the caller agreed to cut in exchange for not
  /// stopping.
  Scalar corner_deviation{0.0};
};

/// A straight-line Cartesian move, timed so that no joint exceeds its limits.
///
/// **This is the module that joins the two halves of the library.** Everywhere
/// else, kinematics answers *where* and knows nothing about time, while the
/// trajectory profiles answer *when* over scalar axes and know nothing about
/// poses. A Cartesian move needs both at once, and the reason it is harder than
/// either is that the relationship between them changes as the arm moves: the
/// Jacobian mapping joint rates to tool velocity is different at every point on
/// the path, and near a singularity it demands unbounded joint rates for a
/// perfectly ordinary tool speed.
///
/// The approach is deliberately not full time-optimal path parameterisation.
/// The path is sampled at knots, solved by inverse kinematics with each knot
/// seeded from the one before, and then driven by a **single** jerk-limited
/// profile on the path parameter whose limits are the tightest any knot
/// requires. That gives one speed for the whole move rather than a speed that
/// varies along it, so the arm crawls past a bad point and then keeps crawling.
/// It is conservative, it is never wrong about the limits, and `binding_s` and
/// `least_manipulability` say exactly where the cost was incurred.
///
/// **Threading**: const methods are pure and reentrant, and sample() neither
/// allocates nor throws.
class CartesianPlan {
 public:
  /// An empty plan of zero duration.
  CartesianPlan() noexcept = default;

  /// Plans a straight move from the pose at `q_start` to `base_T_goal`.
  ///
  /// `q_start` is both the starting configuration and the seed for the first
  /// inverse solve, so the plan begins from where the arm actually is rather
  /// than from whichever configuration reaches the same pose. `joint_limits`
  /// holds one entry per joint, in joint order.
  static Expected<CartesianPlan, CartesianError> plan(
      const SerialChain& chain, std::span<const Scalar> q_start, const SE3& base_T_goal,
      std::span<const MotionLimits> joint_limits, const CartesianOptions& options = {});

  /// Plans one continuous move through several waypoints, without stopping.
  ///
  /// `waypoints` are the poses to visit, in order, not including the pose the
  /// arm starts at -- that comes from `q_start`. Interior corners are rounded
  /// by `options.blend_radius`, so the tool passes near those waypoints rather
  /// than through them, by at most `report().corner_deviation`. The first and
  /// last are reached exactly.
  ///
  /// The whole sequence is one path under one profile, so it begins and ends at
  /// rest and does not stop in between -- which is the entire point. It also
  /// means the sequence is paced by its worst point throughout, exactly as a
  /// single move is.
  static Expected<CartesianPlan, CartesianError> planThrough(
      const SerialChain& chain, std::span<const Scalar> q_start,
      std::span<const SE3> waypoints, std::span<const MotionLimits> joint_limits,
      const CartesianOptions& options = {});

  /// Total time of the move, in seconds.
  [[nodiscard]] Scalar duration() const noexcept { return profile_.duration(); }

  /// The number of joints the plan drives.
  [[nodiscard]] constexpr std::size_t jointCount() const noexcept { return joints_; }

  /// The geometric path, independent of how fast it is traversed.
  [[nodiscard]] const BlendedPath& path() const noexcept { return path_; }

  /// What the plan discovered while planning.
  [[nodiscard]] const CartesianReport& report() const noexcept { return report_; }

  /// The profile of the path parameter itself, on [0, 1].
  [[nodiscard]] const ScurveProfile& pathProfile() const noexcept { return profile_; }

  /// Writes jointCount() joint positions for time `t` into `q`.
  ///
  /// Returns false without writing when `q` is too small. Times outside
  /// [0, duration()] are clamped, so sampling early or late gives the start or
  /// the goal rather than an extrapolation off the end of the path.
  [[nodiscard]] bool sample(Scalar t, std::span<Scalar> q) const noexcept;

  /// Where along the path knot `index` sits, as a fraction. Unchecked.
  ///
  /// Uniform only when `pace_by_difficulty` was off. Exposed because a plan
  /// that spends its parameter unevenly is hard to reason about without being
  /// able to see where it spent it.
  [[nodiscard]] Scalar knotAt(std::size_t index) const noexcept { return at_s_[index]; }

  /// The tool pose the plan intends at time `t`.
  ///
  /// The pose on the ideal line, not the pose the sampled joints produce.
  /// Between knots those differ by at most `chord_deviation`.
  [[nodiscard]] SE3 poseAtTime(Scalar t) const noexcept;

 private:
  /// Everything both entry points share once the geometry is decided. Private
  /// because it reaches into the object it is building.
  static Expected<CartesianPlan, CartesianError> planAlong(
      const SerialChain& chain, std::span<const Scalar> q_start, const BlendedPath& path,
      std::span<const MotionLimits> joint_limits, const CartesianOptions& options);

  BlendedPath path_;
  ScurveProfile profile_;
  /// The path fraction each knot sits at. The profile runs on knot index, and
  /// this is what turns that back into a place on the path.
  std::array<Scalar, kMaxPathKnots> at_s_{};
  std::array<Scalar, kMaxPathKnots * kMaxJoints> knots_{};
  CartesianReport report_;
  std::size_t joints_{0};
};

}  // namespace motionkit
