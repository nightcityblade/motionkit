// SPDX-License-Identifier: Apache-2.0
//
// Straight-line Cartesian moves.
//
// This is the module that joins the two halves of the library, and the tests
// that matter are the ones that check the join actually holds. It is easy to
// produce a plan that follows a line and easy to produce one that respects
// joint limits; the whole difficulty is doing both at once, because the
// relationship between tool speed and joint speed changes at every point on
// the path and blows up near a singularity.
//
// So the central tests differentiate the *sampled output* and compare it
// against the joint limits directly. Not the internal model, not the knots --
// the numbers a controller would actually receive.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <span>
#include <vector>

#include "motionkit/core/cartesian.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/so3.hpp"
#include "motionkit/core/trajectory.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

using Vec = std::vector<Scalar>;

const Vec& startConfiguration() {
  static const Vec kStart{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  return kStart;
}

std::vector<MotionLimits> jointLimits() {
  return std::vector<MotionLimits>(6, MotionLimits{2.0, 8.0, 60.0});
}

/// Worst joint velocity and acceleration in the sampled output.
///
/// Central differences on the plan's own samples, which is what a controller
/// consuming it would experience. Deliberately not the internal derivative
/// estimates: a plan can satisfy its own model and still hand out a stream that
/// violates the limits.
struct Extremes {
  Scalar velocity{0.0};
  Scalar acceleration{0.0};
};

Extremes measureExtremes(const CartesianPlan& plan, std::size_t steps = 4000) {
  const std::size_t joints = plan.jointCount();
  const Scalar dt = plan.duration() / static_cast<Scalar>(steps);
  std::vector<Vec> samples(steps + 1, Vec(joints, 0.0));
  for (std::size_t i = 0; i <= steps; ++i) {
    EXPECT_TRUE(plan.sample(static_cast<Scalar>(i) * dt, samples[i]));
  }
  Extremes worst;
  for (std::size_t i = 1; i < steps; ++i) {
    for (std::size_t j = 0; j < joints; ++j) {
      const Scalar v = (samples[i + 1][j] - samples[i - 1][j]) / (2.0 * dt);
      const Scalar a =
          (samples[i + 1][j] - (2.0 * samples[i][j]) + samples[i - 1][j]) / (dt * dt);
      worst.velocity = std::max(worst.velocity, std::abs(v));
      worst.acceleration = std::max(worst.acceleration, std::abs(a));
    }
  }
  return worst;
}

SE3 poseOf(const Vec& q) { return SerialChain::sixAxisExample().forward(q).value; }

/// A three-legged staircase, not including where the arm starts. Two right
/// angles, each 80 mm on a side, chosen because the example arm follows it
/// comfortably -- an L-shaped route in the horizontal plane drives the base
/// joint to its stop, which is a real answer to a different question.
std::vector<SE3> routeOf(const SE3& from) {
  const SO3 facing = from.rotation();
  const Vec3 origin = from.translation();
  return {
      SE3{facing, origin + Vec3{-0.08, 0.00, 0.00}},
      SE3{facing, origin + Vec3{-0.08, 0.00, -0.08}},
      SE3{facing, origin + Vec3{-0.16, 0.00, -0.08}},
  };
}

/// Tool position and speed at a moment, as a controller would see them.
struct ToolMotion {
  Vec3 where;
  Scalar speed{0.0};
};

std::vector<ToolMotion> toolTrack(const CartesianPlan& plan, std::size_t steps = 2000) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Scalar dt = plan.duration() / static_cast<Scalar>(steps);
  std::vector<Vec3> points(steps + 1);
  Vec q(plan.jointCount(), 0.0);
  for (std::size_t i = 0; i <= steps; ++i) {
    EXPECT_TRUE(plan.sample(static_cast<Scalar>(i) * dt, q));
    points[i] = arm.forward(q).value.translation();
  }
  std::vector<ToolMotion> track(steps + 1);
  for (std::size_t i = 0; i <= steps; ++i) {
    track[i].where = points[i];
    if (i > 0 && i < steps && dt > 0.0) {
      track[i].speed = (points[i + 1] - points[i - 1]).norm() / (2.0 * dt);
    }
  }
  return track;
}

}  // namespace

// --- geometry ----------------------------------------------------------------

TEST(CartesianPathGeometry, InterpolatesBothEndsExactlyAndTheMiddleEvenly) {
  const SE3 start{SO3::fromRPY(0.1, -0.2, 0.3), Vec3{0.4, 0.1, 0.5}};
  const SE3 goal{SO3::fromRPY(-0.4, 0.6, 1.2), Vec3{0.7, -0.2, 0.3}};
  const CartesianPath path = CartesianPath::between(start, goal);

  EXPECT_TRUE(path.poseAt(0.0).isApprox(start, 1e-15, 1e-15));
  EXPECT_TRUE(path.poseAt(1.0).isApprox(goal, 1e-15, 1e-15));
  // Clamped, not extrapolated: asking for a pose off the end of the path
  // should give the end of the path, not a pose beyond the goal.
  EXPECT_TRUE(path.poseAt(-3.0).isApprox(start, 1e-15, 1e-15));
  EXPECT_TRUE(path.poseAt(4.0).isApprox(goal, 1e-15, 1e-15));

  const Vec3 middle = path.poseAt(0.5).translation();
  const Vec3 expected = (start.translation() + goal.translation()) * 0.5;
  EXPECT_LT((middle - expected).norm(), 1e-15);
  EXPECT_NEAR(path.length(), (goal.translation() - start.translation()).norm(), 1e-15);
  EXPECT_NEAR(path.sweptAngle(), start.rotation().angleTo(goal.rotation()), 1e-15);
}

TEST(CartesianPathGeometry, TurnsAtAConstantRateAlongThePath) {
  const SE3 start{SO3{}, Vec3{}};
  const SE3 goal{SO3::rotZ(1.2), Vec3{0.5, 0.0, 0.0}};
  const CartesianPath path = CartesianPath::between(start, goal);
  // SLERP, so the angle from the start grows linearly in s. A pose split into
  // separate position and orientation parameters would not have this property.
  for (const Scalar s : {0.25, 0.5, 0.75}) {
    const Scalar turned = start.rotation().angleTo(path.poseAt(s).rotation());
    EXPECT_NEAR(turned, 1.2 * s, 1e-12) << "at s = " << s;
  }
}

// --- the join: a line, and joint limits, at the same time --------------------

TEST(CartesianPlanning, TheSampledStreamRespectsEveryJointVelocityAndAccelerationLimit) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 from = poseOf(start);
  const SE3 to{from.rotation(), from.translation() + Vec3{0.10, -0.15, 0.08}};

  const auto planned = CartesianPlan::plan(arm, start, to, jointLimits());
  ASSERT_TRUE(planned) << toString(planned.error);

  const Extremes worst = measureExtremes(planned.value);
  std::printf("cartesian stream: peak |qd| %.4f of 2.0, peak |qdd| %.4f of 8.0\n",
              worst.velocity, worst.acceleration);
  // A small margin over the limit for the finite differencing and for the
  // Hermite segments, which are not exactly the piecewise-quadratic the
  // pacing's second-difference estimate assumed.
  EXPECT_LT(worst.velocity, 2.0 * 1.02);
  EXPECT_LT(worst.acceleration, 8.0 * 1.05);
  // And it must actually use the budget -- a planner that crawls satisfies
  // every limit and is useless.
  EXPECT_GT(std::max(worst.velocity / 2.0, worst.acceleration / 8.0), 0.5);
}

TEST(CartesianPlanning, ArrivesAtTheGoalPoseItWasGiven) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 from = poseOf(start);
  const SE3 to{SO3::rotY(0.25) * from.rotation(),
               from.translation() + Vec3{0.12, 0.05, -0.10}};

  const auto planned = CartesianPlan::plan(arm, start, to, jointLimits());
  ASSERT_TRUE(planned) << toString(planned.error);

  Vec finished(arm.jointCount(), 0.0);
  ASSERT_TRUE(planned.value.sample(planned.value.duration(), finished));
  const SE3 reached = arm.forward(finished).value;
  EXPECT_TRUE(reached.isApprox(to, 1e-6, 1e-6));

  Vec begun(arm.jointCount(), 0.0);
  ASSERT_TRUE(planned.value.sample(0.0, begun));
  for (std::size_t j = 0; j < arm.jointCount(); ++j) {
    EXPECT_NEAR(begun[j], start[j], 1e-12) << "joint " << j;
  }
}

TEST(CartesianPlanning, TheToolStaysOnTheLineToWithinTheDeviationItReports) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 from = poseOf(start);
  const SE3 to{from.rotation(), from.translation() + Vec3{0.20, 0.0, 0.0}};

  const auto planned = CartesianPlan::plan(arm, start, to, jointLimits());
  ASSERT_TRUE(planned) << toString(planned.error);

  Scalar worst = 0.0;
  Vec q(arm.jointCount(), 0.0);
  for (int i = 0; i <= 500; ++i) {
    const Scalar t = planned.value.duration() * static_cast<Scalar>(i) / 500.0;
    ASSERT_TRUE(planned.value.sample(t, q));
    const Vec3 actual = arm.forward(q).value.translation();
    const Vec3 intended = planned.value.poseAtTime(t).translation();
    worst = std::max(worst, (actual - intended).norm());
  }
  std::printf("straight-line deviation: measured %.3e m, reported %.3e m\n", worst,
              planned.value.report().chord_deviation);
  // The reported number is measured at knot midpoints, where the gap is
  // largest, so a dense sweep must not beat it by much.
  EXPECT_LT(worst, planned.value.report().chord_deviation * 1.5 + 1e-9);
  EXPECT_LT(worst, 1e-4);
}

// --- what the straight line costs -------------------------------------------

TEST(CartesianPlanning, NearASingularityTheSameMoveTakesLonger) {
  const SerialChain arm = SerialChain::sixAxisExample();
  // The wrist is singular when joint 4 is zero, so the same arm with joint 4
  // at 0.02 rad instead of 0.7 is badly conditioned in exactly one way and
  // identical in every other -- which is what makes the comparison mean
  // something.
  const Vec awkward{0.3, -0.6, 1.0, 0.4, 0.02, -0.2};
  const Vec comfortable = startConfiguration();
  const Vec3 travel{0.05, 0.0, 0.0};

  const SE3 awkward_goal{poseOf(awkward).rotation(),
                         poseOf(awkward).translation() + travel};
  const SE3 easy_goal{poseOf(comfortable).rotation(),
                      poseOf(comfortable).translation() + travel};

  const auto near_singular =
      CartesianPlan::plan(arm, awkward, awkward_goal, jointLimits());
  const auto well_posed = CartesianPlan::plan(arm, comfortable, easy_goal, jointLimits());
  ASSERT_TRUE(near_singular) << toString(near_singular.error);
  ASSERT_TRUE(well_posed) << toString(well_posed.error);

  std::printf("same 50 mm move: %.4f s at manipulability %.3e, %.4f s at %.3e -- %.1fx\n",
              near_singular.value.duration(),
              near_singular.value.report().least_manipulability,
              well_posed.value.duration(), well_posed.value.report().least_manipulability,
              near_singular.value.duration() / well_posed.value.duration());

  // The point of reporting least_manipulability: it is the number that
  // explains the slower move, and the fix is usually to move the line rather
  // than to raise a limit.
  EXPECT_LT(near_singular.value.report().least_manipulability,
            well_posed.value.report().least_manipulability);
  EXPECT_GT(near_singular.value.duration(), well_posed.value.duration());
}

TEST(CartesianPlanning, HoldingTheLineCostsTimeAgainstLettingTheJointsGoWhereTheyLike) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 from = poseOf(start);
  const SE3 to{from.rotation(), from.translation() + Vec3{0.25, -0.10, 0.05}};
  const std::vector<MotionLimits> limits = jointLimits();

  const auto cartesian = CartesianPlan::plan(arm, start, to, limits);
  ASSERT_TRUE(cartesian) << toString(cartesian.error);

  // The same endpoints, but free to take whatever route joint space prefers.
  Vec finished(arm.jointCount(), 0.0);
  ASSERT_TRUE(cartesian.value.sample(cartesian.value.duration(), finished));
  const auto joint_space = SynchronizedTrajectory::plan(start, finished, limits);
  ASSERT_TRUE(joint_space) << static_cast<int>(joint_space.error);

  const Scalar ratio = cartesian.value.duration() / joint_space.value.duration();
  std::printf("straight line %.4f s vs joint-space %.4f s -- %.2fx for the line\n",
              cartesian.value.duration(), joint_space.value.duration(), ratio);
  // Constraining the tool to a line can only cost time: the joint-space move
  // is free to use every joint's full budget at once, and the Cartesian one is
  // paced by whichever joint the geometry loads hardest.
  EXPECT_GT(ratio, 1.0);
}

TEST(CartesianPlanning, ConsecutiveKnotsStayInOneConfiguration) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 from = poseOf(start);
  const SE3 to{SO3::rotY(0.4) * from.rotation(),
               from.translation() + Vec3{0.15, 0.10, -0.05}};

  const auto planned = CartesianPlan::plan(arm, start, to, jointLimits());
  ASSERT_TRUE(planned) << toString(planned.error);

  // A 6R arm reaches most poses in up to eight configurations. Solving each
  // knot cold would let neighbouring knots land in different ones, producing a
  // plan whose every pose is correct and which asks the arm to turn itself
  // inside out halfway along. Seeding is what prevents that, and a large jump
  // between adjacent samples is how the failure would show.
  Scalar biggest = 0.0;
  Vec previous(arm.jointCount(), 0.0);
  Vec current(arm.jointCount(), 0.0);
  ASSERT_TRUE(planned.value.sample(0.0, previous));
  for (int i = 1; i <= 200; ++i) {
    const Scalar t = planned.value.duration() * static_cast<Scalar>(i) / 200.0;
    ASSERT_TRUE(planned.value.sample(t, current));
    for (std::size_t j = 0; j < arm.jointCount(); ++j) {
      biggest = std::max(biggest, std::abs(current[j] - previous[j]));
    }
    previous = current;
  }
  std::printf("largest joint step between samples: %.4f rad\n", biggest);
  EXPECT_LT(biggest, 0.1);
}

// --- blending through waypoints ----------------------------------------------

TEST(CartesianBlending, TheToolDoesNotStopAtTheWaypointsItPassesThrough) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const std::vector<SE3> route = routeOf(poseOf(start));

  const auto blended = CartesianPlan::planThrough(arm, start, route, jointLimits());
  ASSERT_TRUE(blended) << toString(blended.error);

  const std::vector<ToolMotion> track = toolTrack(blended.value);
  Scalar peak = 0.0;
  for (const ToolMotion& m : track) {
    peak = std::max(peak, m.speed);
  }

  // For each interior waypoint, find where the tool passes closest to it and
  // ask how fast it was going. Planned as separate moves that speed is zero by
  // construction, because each move ends at rest. That is the whole difference.
  Scalar slowest = peak;
  for (std::size_t w = 0; w + 1 < route.size(); ++w) {
    Scalar nearest = std::numeric_limits<Scalar>::max();
    Scalar speed_there = 0.0;
    for (const ToolMotion& m : track) {
      const Scalar gap = (m.where - route[w].translation()).norm();
      if (gap < nearest) {
        nearest = gap;
        speed_there = m.speed;
      }
    }
    std::printf("waypoint %zu: passed %.4f m away at %.4f m/s (peak %.4f)\n", w, nearest,
                speed_there, peak);
    slowest = std::min(slowest, speed_there);
  }
  EXPECT_GT(slowest, peak * 0.3) << "a blended move must carry speed through the corners";
}

TEST(CartesianBlending, BlendingBeatsPlanningTheSameWaypointsSeparately) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const std::vector<SE3> route = routeOf(poseOf(start));
  const std::vector<MotionLimits> limits = jointLimits();

  const auto blended = CartesianPlan::planThrough(arm, start, route, limits);
  ASSERT_TRUE(blended) << toString(blended.error);

  // The same route as three moves, each starting and ending at rest.
  Scalar separately = 0.0;
  Vec here = start;
  Vec finished(arm.jointCount(), 0.0);
  for (const SE3& waypoint : route) {
    const auto leg = CartesianPlan::plan(arm, here, waypoint, limits);
    ASSERT_TRUE(leg) << toString(leg.error);
    separately += leg.value.duration();
    ASSERT_TRUE(leg.value.sample(leg.value.duration(), finished));
    here = finished;
  }

  std::printf("blended %.4f s against %.4f s stopping at each waypoint -- %.2fx\n",
              blended.value.duration(), separately,
              separately / blended.value.duration());
  EXPECT_LT(blended.value.duration(), separately);
}

TEST(CartesianBlending, TheToolPassesEveryWaypointWithinTheDeviationItReports) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const std::vector<SE3> route = routeOf(poseOf(start));

  const auto blended = CartesianPlan::planThrough(arm, start, route, jointLimits());
  ASSERT_TRUE(blended) << toString(blended.error);
  const std::vector<ToolMotion> track = toolTrack(blended.value);

  Scalar worst = 0.0;
  for (std::size_t w = 0; w + 1 < route.size(); ++w) {
    Scalar nearest = std::numeric_limits<Scalar>::max();
    for (const ToolMotion& m : track) {
      nearest = std::min(nearest, (m.where - route[w].translation()).norm());
    }
    worst = std::max(worst, nearest);
  }
  const CartesianReport& report = blended.value.report();
  std::printf("corner cut: measured %.4f m, reported %.4f m\n", worst,
              report.corner_deviation);
  // The reported figure is the geometric corner cut; the tool also leaves the
  // path a little between knots, so the measured miss may exceed it slightly.
  EXPECT_LT(worst, report.corner_deviation + report.chord_deviation + 1e-9);
  EXPECT_GT(report.corner_deviation, 0.0) << "an L-shaped route has corners to cut";
  EXPECT_EQ(report.waypoints, route.size() + 1) << "the start counts as a waypoint";
}

TEST(CartesianBlending, ARadiusOfZeroPassesExactlyThroughEveryWaypoint) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const std::vector<SE3> route = routeOf(poseOf(start));
  CartesianOptions exact;
  exact.blend_radius = 0.0;

  const auto sharp = CartesianPlan::planThrough(arm, start, route, jointLimits(), exact);
  ASSERT_TRUE(sharp) << toString(sharp.error);
  EXPECT_NEAR(sharp.value.report().corner_deviation, 0.0, 1e-15);

  // Which is the honest trade: hitting the corners exactly costs the time that
  // rounding them would have saved.
  const auto rounded = CartesianPlan::planThrough(arm, start, route, jointLimits());
  ASSERT_TRUE(rounded);
  std::printf("exact corners %.4f s against rounded %.4f s\n", sharp.value.duration(),
              rounded.value.duration());
  EXPECT_GT(sharp.value.duration(), rounded.value.duration());
}

TEST(CartesianBlending, ABlendedMoveStillRespectsEveryJointLimit) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const auto blended =
      CartesianPlan::planThrough(arm, start, routeOf(poseOf(start)), jointLimits());
  ASSERT_TRUE(blended) << toString(blended.error);

  const Extremes worst = measureExtremes(blended.value);
  std::printf("blended stream: peak |qd| %.4f of 2.0, peak |qdd| %.4f of 8.0\n",
              worst.velocity, worst.acceleration);
  EXPECT_LT(worst.velocity, 2.0 * 1.02);
  EXPECT_LT(worst.acceleration, 8.0 * 1.05);
}

TEST(CartesianBlending, TheFirstAndLastWaypointsAreReachedExactly) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const std::vector<SE3> route = routeOf(poseOf(start));
  const auto blended = CartesianPlan::planThrough(arm, start, route, jointLimits());
  ASSERT_TRUE(blended);

  Vec q(arm.jointCount(), 0.0);
  ASSERT_TRUE(blended.value.sample(0.0, q));
  for (std::size_t j = 0; j < arm.jointCount(); ++j) {
    EXPECT_NEAR(q[j], start[j], 1e-12) << "joint " << j;
  }
  ASSERT_TRUE(blended.value.sample(blended.value.duration(), q));
  EXPECT_TRUE(arm.forward(q).value.isApprox(route.back(), 1e-6, 1e-6));
}

TEST(CartesianBlending, OneWaypointIsExactlyASingleStraightMove) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 to{poseOf(start).rotation(), poseOf(start).translation() + Vec3{0.15, 0, 0}};

  const auto direct = CartesianPlan::plan(arm, start, to, jointLimits());
  const std::array<SE3, 1> single{to};
  const auto through = CartesianPlan::planThrough(arm, start, single, jointLimits());
  ASSERT_TRUE(direct);
  ASSERT_TRUE(through);
  // plan() delegates to planThrough(), so this checks the delegation is
  // faithful rather than making an independent claim.
  EXPECT_NEAR(direct.value.duration(), through.value.duration(), 1e-15);
  EXPECT_NEAR(direct.value.report().corner_deviation, 0.0, 1e-15);
}

TEST(CartesianBlending, AnEnormousRadiusIsClampedRatherThanAllowedToOverlap) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const std::vector<SE3> route = routeOf(poseOf(start));
  CartesianOptions greedy;
  // Far larger than any segment on the route. Uncorrected, adjacent blends
  // would reach past each other and the path would fold back on itself.
  greedy.blend_radius = 10.0;

  const auto blended =
      CartesianPlan::planThrough(arm, start, route, jointLimits(), greedy);
  ASSERT_TRUE(blended) << toString(blended.error);
  // No corner may be cut by more than half the shorter of its two segments,
  // and every leg here is 80 mm.
  EXPECT_LT(blended.value.report().corner_deviation, 0.08 * 0.5 + 1e-9);
  for (const ToolMotion& m : toolTrack(blended.value)) {
    EXPECT_TRUE(std::isfinite(m.where.squaredNorm()));
  }
}

TEST(CartesianBlending, RefusesRepeatedInteriorWaypointsAndOverlongRoutes) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 from = poseOf(start);
  const SO3 facing = from.rotation();

  // A waypoint repeated in the middle leaves no direction to turn through.
  const std::array<SE3, 3> repeated{
      SE3{facing, from.translation() + Vec3{-0.08, 0.0, 0.0}},
      SE3{facing, from.translation() + Vec3{-0.08, 0.0, 0.0}},
      SE3{facing, from.translation() + Vec3{-0.08, 0.0, -0.08}}};
  EXPECT_EQ(CartesianPlan::planThrough(arm, start, repeated, jointLimits()).error,
            CartesianError::WaypointCountUnsupported);

  EXPECT_EQ(
      CartesianPlan::planThrough(arm, start, std::span<const SE3>{}, jointLimits()).error,
      CartesianError::WaypointCountUnsupported);

  std::vector<SE3> too_many;
  for (std::size_t i = 0; i < kMaxWaypoints; ++i) {
    too_many.emplace_back(
        facing, from.translation() + Vec3{-0.01 * static_cast<Scalar>(i + 1), 0.0, 0.0});
  }
  EXPECT_EQ(CartesianPlan::planThrough(arm, start, too_many, jointLimits()).error,
            CartesianError::WaypointCountUnsupported);
}

// --- refusals ----------------------------------------------------------------

TEST(CartesianPlanning, ReportsAGoalItCannotReachAlongTheWholeLine) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 from = poseOf(start);
  // Five metres away, well outside a 1.07 m arm.
  const SE3 unreachable{from.rotation(), from.translation() + Vec3{5.0, 0.0, 0.0}};
  EXPECT_EQ(CartesianPlan::plan(arm, start, unreachable, jointLimits()).error,
            CartesianError::UnreachableAlongPath);
}

TEST(CartesianPlanning, DistinguishesAJointRunningOutOfTravelFromAnUnreachableLine) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 from = poseOf(start);
  // Every pose on this line is reachable and the line is short. Turning the
  // tool about world Z from here drags the base joint round to its stop --
  // an arm-configuration problem, not a workspace one, and worth saying so.
  const SE3 to{SO3::rotZ(0.25) * from.rotation(),
               from.translation() + Vec3{0.12, 0.05, -0.10}};
  EXPECT_EQ(CartesianPlan::plan(arm, start, to, jointLimits()).error,
            CartesianError::JointLimitAlongPath);
}

TEST(CartesianPlanning, AGoalWhereTheArmAlreadyIsTakesNoTime) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const auto planned = CartesianPlan::plan(arm, start, poseOf(start), jointLimits());
  ASSERT_TRUE(planned) << toString(planned.error);
  EXPECT_NEAR(planned.value.duration(), 0.0, 1e-15);
  EXPECT_EQ(planned.value.report().binding_joint, arm.jointCount())
      << "nothing moves, so no joint set the pace";

  Vec q(arm.jointCount(), 0.0);
  ASSERT_TRUE(planned.value.sample(0.0, q));
  for (std::size_t j = 0; j < arm.jointCount(); ++j) {
    EXPECT_NEAR(q[j], start[j], 1e-12);
  }
}

TEST(CartesianPlanning, RefusesKnotCountsThatCannotDescribeAPath) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 to{poseOf(start).rotation(), poseOf(start).translation() + Vec3{0.05, 0, 0}};
  CartesianOptions options;
  // Two knots are the endpoints and say nothing about what happens between.
  options.knots = 2;
  EXPECT_EQ(CartesianPlan::plan(arm, start, to, jointLimits(), options).error,
            CartesianError::KnotCountUnsupported);
  options.knots = kMaxPathKnots + 1;
  EXPECT_EQ(CartesianPlan::plan(arm, start, to, jointLimits(), options).error,
            CartesianError::KnotCountUnsupported);
}

TEST(CartesianPlanning, RefusesUnusableLimitsAndMismatchedSizes) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 to{poseOf(start).rotation(), poseOf(start).translation() + Vec3{0.05, 0, 0}};

  // Default-constructed limits are all zero, which MotionLimits defines as
  // "may not move" rather than "unlimited" -- see ADR-0006.
  const std::vector<MotionLimits> silent(6, MotionLimits{});
  EXPECT_EQ(CartesianPlan::plan(arm, start, to, silent).error,
            CartesianError::LimitsNotUsable);

  const std::vector<MotionLimits> too_few(4, MotionLimits{2.0, 8.0, 60.0});
  EXPECT_EQ(CartesianPlan::plan(arm, start, to, too_few).error,
            CartesianError::SizeMismatch);

  const Vec short_start{0.1, 0.2, 0.3};
  EXPECT_EQ(CartesianPlan::plan(arm, short_start, to, jointLimits()).error,
            CartesianError::SizeMismatch);

  CartesianOptions options;
  options.cornering_share = 0.0;
  EXPECT_EQ(CartesianPlan::plan(arm, start, to, jointLimits(), options).error,
            CartesianError::LimitsNotUsable);
}

TEST(CartesianPlanning, RejectsANonFiniteGoal) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 bad{SO3{}, Vec3{std::numeric_limits<Scalar>::quiet_NaN(), 0.0, 0.0}};
  EXPECT_EQ(CartesianPlan::plan(arm, start, bad, jointLimits()).error,
            CartesianError::NonFiniteInput);
}

TEST(CartesianPlanning, SamplingIntoATooSmallBufferFailsRatherThanWriting) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start = startConfiguration();
  const SE3 to{poseOf(start).rotation(), poseOf(start).translation() + Vec3{0.05, 0, 0}};
  const auto planned = CartesianPlan::plan(arm, start, to, jointLimits());
  ASSERT_TRUE(planned);
  std::array<Scalar, 3> cramped{};
  EXPECT_FALSE(planned.value.sample(0.0, cramped));
}

TEST(CartesianErrors, EveryEnumeratorHasAName) {
  EXPECT_EQ(toString(CartesianError::None), "None");
  EXPECT_EQ(toString(CartesianError::SizeMismatch), "SizeMismatch");
  EXPECT_EQ(toString(CartesianError::NonFiniteInput), "NonFiniteInput");
  EXPECT_EQ(toString(CartesianError::KnotCountUnsupported), "KnotCountUnsupported");
  EXPECT_EQ(toString(CartesianError::WaypointCountUnsupported),
            "WaypointCountUnsupported");
  EXPECT_EQ(toString(CartesianError::UnreachableAlongPath), "UnreachableAlongPath");
  EXPECT_EQ(toString(CartesianError::JointLimitAlongPath), "JointLimitAlongPath");
  EXPECT_EQ(toString(CartesianError::LimitsNotUsable), "LimitsNotUsable");
}

}  // namespace motionkit
