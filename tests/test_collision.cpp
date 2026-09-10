// SPDX-License-Identifier: Apache-2.0
//
// Capsule clearance, and self-collision on a serial chain.
//
// The geometry is one function -- the distance between two segments -- and it
// is testable exactly, so it is tested exactly: parallel, crossing, skew,
// degenerate to spheres, and overlapping. No tolerances anyone negotiated.
//
// The test that matters most is duller than any of those. It checks the example
// arm reports positive clearance while standing still. A self-collision model
// whose links overlap at rest reports a collision always, and the response to
// that is not to fix the model -- it is to switch collision checking off, which
// is how a machine ends up with none.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <span>
#include <vector>

#include "motionkit/core/cartesian.hpp"
#include "motionkit/core/collision.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/trajectory.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

using Vec = std::vector<Scalar>;

Capsule segment(const Vec3& from, const Vec3& to, Scalar radius) {
  return Capsule{from, to, radius};
}

}  // namespace

// --- the geometry ------------------------------------------------------------

TEST(CapsuleDistance, MatchesHandComputedCasesExactly) {
  // Parallel, one metre apart, both thin.
  EXPECT_NEAR(distanceBetween(segment({0, 0, 0}, {1, 0, 0}, 0.0),
                              segment({0, 1, 0}, {1, 1, 0}, 0.0)),
              1.0, 1e-15);

  // The same pair, thickened until they touch and then until they overlap.
  EXPECT_NEAR(distanceBetween(segment({0, 0, 0}, {1, 0, 0}, 0.4),
                              segment({0, 1, 0}, {1, 1, 0}, 0.6)),
              0.0, 1e-15);
  EXPECT_NEAR(distanceBetween(segment({0, 0, 0}, {1, 0, 0}, 0.5),
                              segment({0, 1, 0}, {1, 1, 0}, 0.7)),
              -0.2, 1e-15);

  // Skew: along x at z = 0, along y at z = 2. Closest approach is the vertical
  // gap of 2, which no amount of sliding along either segment shortens.
  EXPECT_NEAR(distanceBetween(segment({-1, 0, 0}, {1, 0, 0}, 0.0),
                              segment({0, -1, 2}, {0, 1, 2}, 0.0)),
              2.0, 1e-15);

  // Crossing in the same plane: they intersect, so the axes are zero apart and
  // the radii are the whole of the overlap.
  EXPECT_NEAR(distanceBetween(segment({-1, 0, 0}, {1, 0, 0}, 0.1),
                              segment({0, -1, 0}, {0, 1, 0}, 0.2)),
              -0.3, 1e-15);

  // End to end along one line, with a one-metre gap between the near ends.
  EXPECT_NEAR(distanceBetween(segment({0, 0, 0}, {1, 0, 0}, 0.0),
                              segment({2, 0, 0}, {3, 0, 0}, 0.0)),
              1.0, 1e-15);
}

TEST(CapsuleDistance, HandlesShapesThatDegenerateToSpheres) {
  // A capsule whose ends coincide is a sphere, which callers pass on purpose.
  // Two spheres three apart, radius one each.
  EXPECT_NEAR(distanceBetween(segment({0, 0, 0}, {0, 0, 0}, 1.0),
                              segment({3, 0, 0}, {3, 0, 0}, 1.0)),
              1.0, 1e-15);
  // A sphere against a segment it sits beside.
  EXPECT_NEAR(distanceBetween(segment({0.5, 2, 0}, {0.5, 2, 0}, 0.5),
                              segment({0, 0, 0}, {1, 0, 0}, 0.25)),
              1.25, 1e-15);
  // A sphere swallowed by a fat capsule: negative by the depth, not clamped.
  EXPECT_NEAR(distanceBetween(segment({0.5, 0, 0}, {0.5, 0, 0}, 0.1),
                              segment({0, 0, 0}, {1, 0, 0}, 0.4)),
              -0.5, 1e-15);
}

TEST(CapsuleDistance, IsSymmetricAndIndependentOfEndpointOrder) {
  const Capsule a = segment({0.2, -0.4, 1.1}, {0.9, 0.3, 0.5}, 0.07);
  const Capsule b = segment({-0.3, 0.8, 0.2}, {0.6, 0.1, 1.4}, 0.05);
  const Scalar forward = distanceBetween(a, b);
  EXPECT_NEAR(distanceBetween(b, a), forward, 1e-15);
  EXPECT_NEAR(distanceBetween(segment(a.to, a.from, a.radius), b), forward, 1e-15);
  EXPECT_NEAR(distanceBetween(a, segment(b.to, b.from, b.radius)), forward, 1e-15);
}

// --- the model on a real chain -----------------------------------------------

TEST(CollisionModel, TheExampleArmIsNotInCollisionWithItselfWhileStandingStill) {
  const auto model = CollisionModel::sixAxisExample();
  ASSERT_TRUE(model) << toString(model.error);
  const Vec upright(model.value.linkCount(), 0.0);

  const auto rest = model.value.clearance(upright, {});
  ASSERT_TRUE(rest) << toString(rest.error);
  std::printf("arm at zero: closest self pair %zu-%zu at %.4f m\n", rest.value.link,
              rest.value.other, rest.value.distance);
  // The whole point of the allowed-collision set. Without it links two apart at
  // the wrist overlap here, the model reports a collision while the arm is
  // parked, and the next person switches collision checking off entirely.
  EXPECT_GT(rest.value.distance, 0.0);

  // And in an ordinary working pose.
  const Vec working{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  const auto posed = model.value.clearance(working, {});
  ASSERT_TRUE(posed);
  std::printf("arm at work: closest self pair %zu-%zu at %.4f m\n", posed.value.link,
              posed.value.other, posed.value.distance);
  EXPECT_GT(posed.value.distance, 0.0);
}

TEST(CollisionModel, AnArmFoldedOntoItselfIsReported) {
  const auto model = CollisionModel::sixAxisExample();
  ASSERT_TRUE(model);
  // Shoulder up and elbow driven right over, which lays the forearm back down
  // across the base column. Bending the elbow the other way looks just as
  // folded and misses by 55 mm, which is why the configuration here was found
  // by sweeping rather than by picturing it.
  const Vec folded{0.0, 1.9, 2.6, 0.0, 0.0, 0.0};
  const auto result = model.value.clearance(folded, {});
  ASSERT_TRUE(result);
  std::printf("folded: pair %zu-%zu (self=%d) at %.4f m\n", result.value.link,
              result.value.other, static_cast<int>(result.value.self),
              result.value.distance);
  EXPECT_TRUE(result.value.self);
  EXPECT_LT(result.value.distance, 0.0) << "an arm folded into its own base is a hit";
  EXPECT_EQ(result.value.link, 0u);
  EXPECT_EQ(result.value.other, 2u) << "the forearm laid across the base column";
  EXPECT_LT(result.value.to_self, 0.0);
}

TEST(CollisionModel, AnObstacleInTheWayIsFoundWithTheRightDistance) {
  const auto model = CollisionModel::sixAxisExample();
  ASSERT_TRUE(model);
  const Vec upright(model.value.linkCount(), 0.0);

  // A sphere out to one side of the parked arm, level with the middle of the
  // forearm. The forearm axis is x = y = 0 over z in [0.58, 0.97] with radius
  // 0.06, so a sphere of radius 0.1 centred 1.0 away leaves 1.0 - 0.1 - 0.06.
  //
  // Level matters. At z = 0.70 the answer is 0.8372, not 0.84, because the
  // upper arm's thicker 0.07 sleeve more than makes up for its end being
  // slightly further off -- which is the kind of arithmetic that is quicker to
  // let the test do than to do in one's head.
  const std::array<Capsule, 1> ball{
      Capsule{Vec3{1.0, 0.0, 0.90}, Vec3{1.0, 0.0, 0.90}, 0.10}};
  const auto clear = model.value.clearance(upright, ball);
  ASSERT_TRUE(clear);
  // to_obstacles, not distance. The arm's own links sit 0.22 m apart while it
  // stands straight, so the global minimum reports that instead -- correctly,
  // and uselessly for anybody asking about the world.
  EXPECT_NEAR(clear.value.to_obstacles, 1.0 - 0.10 - 0.06, 1e-12);
  EXPECT_TRUE(clear.value.self) << "the arm really is nearer to itself than to that";

  // Slide it in until it overlaps, and the report goes negative by the depth.
  const std::array<Capsule, 1> intruding{
      Capsule{Vec3{0.10, 0.0, 0.90}, Vec3{0.10, 0.0, 0.90}, 0.10}};
  const auto hit = model.value.clearance(upright, intruding);
  ASSERT_TRUE(hit);
  EXPECT_NEAR(hit.value.to_obstacles, 0.10 - 0.10 - 0.06, 1e-12);
  // Now it is nearer than the arm is to itself, so it also wins outright.
  EXPECT_NEAR(hit.value.distance, 0.10 - 0.10 - 0.06, 1e-12);
  EXPECT_FALSE(hit.value.self);
  EXPECT_EQ(hit.value.link, 2u) << "the forearm is what it reaches first";
}

TEST(CollisionModel, AdjacentLinksAreNeverReportedHoweverMuchTheyOverlap) {
  // Two links sharing a joint always touch. A model that reports it is a model
  // that reports nothing else, because that pair wins every query.
  const SerialChain arm = SerialChain::sixAxisExample();
  std::array<Capsule, 6> fat{};
  for (Capsule& shape : fat) {
    // Every link a fat sphere at the origin: every pair overlaps enormously.
    shape = Capsule{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 0.5};
  }
  const auto model = CollisionModel::build(arm, fat);
  ASSERT_TRUE(model);
  const Vec upright(6, 0.0);
  const auto result = model.value.clearance(upright, {});
  ASSERT_TRUE(result);
  // Non-adjacent pairs still overlap here, so it must find one of those.
  EXPECT_TRUE(result.value.self);
  EXPECT_GT(result.value.other, result.value.link + 1)
      << "an adjacent pair was reported, which cannot be a finding";
}

TEST(CollisionModel, WithNothingToHitTheAnswerIsFiniteRatherThanInfinite) {
  const SerialChain arm = SerialChain::sixAxisExample();
  std::array<Capsule, 6> thin{};
  for (Capsule& shape : thin) {
    shape = Capsule{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.01}, 0.001};
  }
  // Every non-adjacent pair excluded and no obstacles: there is genuinely
  // nothing to measure. Infinity would be the honest answer and a terrible one
  // to hand back, because it invites arithmetic nobody checks.
  const std::array<std::array<std::size_t, 2>, 10> ignored{
      {{0, 2}, {0, 3}, {0, 4}, {0, 5}, {1, 3}, {1, 4}, {1, 5}, {2, 4}, {2, 5}, {3, 5}}};
  const auto model = CollisionModel::build(arm, thin, ignored);
  ASSERT_TRUE(model);
  const auto result = model.value.clearance(Vec(6, 0.0), {});
  ASSERT_TRUE(result);
  EXPECT_TRUE(std::isfinite(result.value.distance));
  EXPECT_NEAR(result.value.distance, kNothingNear, 1e-9);
  EXPECT_NEAR(result.value.to_self, kNothingNear, 1e-9);
  EXPECT_NEAR(result.value.to_obstacles, kNothingNear, 1e-9);
}

TEST(CollisionModel, ClearanceFallsAsAnObstacleApproachesTheParkedArm) {
  const auto model = CollisionModel::sixAxisExample();
  ASSERT_TRUE(model);
  const Vec upright(6, 0.0);
  Scalar previous = kNothingNear;
  for (const Scalar offset : {1.0, 0.8, 0.6, 0.4, 0.2, 0.1}) {
    const std::array<Capsule, 1> ball{
        Capsule{Vec3{offset, 0.0, 0.90}, Vec3{offset, 0.0, 0.90}, 0.05}};
    const auto result = model.value.clearance(upright, ball);
    ASSERT_TRUE(result);
    EXPECT_LT(result.value.to_obstacles, previous) << "at offset " << offset;
    previous = result.value.to_obstacles;
  }
  // The self clearance is a property of the pose and did not move at all while
  // the obstacle came in, which is exactly why the two are reported apart.
  const auto alone = model.value.clearance(upright, {});
  ASSERT_TRUE(alone);
  EXPECT_NEAR(alone.value.to_self, 0.22, 0.01);
  EXPECT_NEAR(alone.value.to_obstacles, kNothingNear, 1e-9);
}

TEST(CollisionModel, ClearanceCanBeFollowedAlongAPlannedCartesianMove) {
  // The pairing this exists for. A planner reports how close it came to a
  // singularity; nothing yet reported how close it came to the fence, and the
  // two are asked at the same moment by the same person.
  const auto model = CollisionModel::sixAxisExample();
  ASSERT_TRUE(model);
  const SerialChain arm = SerialChain::sixAxisExample();
  const Vec start{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  const SE3 from = arm.forward(start).value;
  const SE3 to{from.rotation(), from.translation() + Vec3{-0.16, 0.0, -0.08}};
  const std::vector<MotionLimits> limits(6, MotionLimits{2.0, 8.0, 60.0});

  const auto planned = CartesianPlan::plan(arm, start, to, limits);
  ASSERT_TRUE(planned) << toString(planned.error);

  // A post standing where the tool is heading.
  const std::array<Capsule, 1> post{Capsule{from.translation() + Vec3{-0.30, 0.14, -0.40},
                                            from.translation() + Vec3{-0.30, 0.14, 0.40},
                                            0.04}};

  Scalar worst = kNothingNear;
  Scalar at_time = 0.0;
  Vec q(arm.jointCount(), 0.0);
  for (int i = 0; i <= 400; ++i) {
    const Scalar t = planned.value.duration() * static_cast<Scalar>(i) / 400.0;
    ASSERT_TRUE(planned.value.sample(t, q));
    const auto found = model.value.clearance(q, post);
    ASSERT_TRUE(found) << toString(found.error);
    if (found.value.to_obstacles < worst) {
      worst = found.value.to_obstacles;
      at_time = t;
    }
  }
  std::printf("closest approach to the post: %.4f m at t = %.3f s of %.3f s\n", worst,
              at_time, planned.value.duration());
  EXPECT_LT(worst, kNothingNear) << "the post must be seen at all";
  EXPECT_GT(worst, 0.0) << "and this move must not actually hit it";
}

// --- refusals ----------------------------------------------------------------

TEST(CollisionModel, RefusesAnExclusionAimedAtALinkThatDoesNotExist) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const std::array<Capsule, 6> shapes{};
  const std::array<std::array<std::size_t, 2>, 1> beyond{{{2, 9}}};
  // Dropping it silently would leave a model that believes it is safer than it
  // is: somebody wrote an exclusion, and it is doing nothing.
  EXPECT_EQ(CollisionModel::build(arm, shapes, beyond).error,
            CollisionError::SizeMismatch);
}

TEST(CollisionModel, RefusesNegativeRadiiAndMismatchedCounts) {
  const SerialChain arm = SerialChain::sixAxisExample();
  std::array<Capsule, 6> shapes{};
  shapes[3].radius = -0.01;
  EXPECT_EQ(CollisionModel::build(arm, shapes).error, CollisionError::NegativeRadius);

  const std::array<Capsule, 4> too_few{};
  EXPECT_EQ(CollisionModel::build(arm, too_few).error, CollisionError::SizeMismatch);

  const auto model = CollisionModel::sixAxisExample();
  ASSERT_TRUE(model);
  std::array<Capsule, 1> bad_obstacle{};
  bad_obstacle[0].radius = -1.0;
  EXPECT_EQ(model.value.clearance(Vec(6, 0.0), bad_obstacle).error,
            CollisionError::NegativeRadius);
  EXPECT_EQ(model.value.clearance(Vec(3, 0.0), {}).error, CollisionError::SizeMismatch);
}

TEST(CollisionModel, RejectsNonFiniteInput) {
  const auto model = CollisionModel::sixAxisExample();
  ASSERT_TRUE(model);
  Vec bad(6, 0.0);
  bad[2] = std::numeric_limits<Scalar>::quiet_NaN();
  EXPECT_EQ(model.value.clearance(bad, {}).error, CollisionError::NonFiniteInput);

  std::array<Capsule, 1> nowhere{};
  nowhere[0].from = Vec3{std::numeric_limits<Scalar>::infinity(), 0.0, 0.0};
  EXPECT_EQ(model.value.clearance(Vec(6, 0.0), nowhere).error,
            CollisionError::NonFiniteInput);
}

TEST(CollisionErrors, EveryEnumeratorHasAName) {
  EXPECT_EQ(toString(CollisionError::None), "None");
  EXPECT_EQ(toString(CollisionError::SizeMismatch), "SizeMismatch");
  EXPECT_EQ(toString(CollisionError::NonFiniteInput), "NonFiniteInput");
  EXPECT_EQ(toString(CollisionError::NegativeRadius), "NegativeRadius");
}

}  // namespace motionkit
