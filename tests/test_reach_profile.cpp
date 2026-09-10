// SPDX-License-Identifier: Apache-2.0
//
// A jerk-limited move to a position, starting from any state.
//
// ScurveProfile starts at rest, which makes it easy to test: there is one
// shape, and its symmetry does most of the work. ReachProfile has no symmetry
// to lean on -- the initial velocity may point either way and the initial
// acceleration may already be past the limit -- so the tests here are mostly
// property tests over a spread of starting states, checking the sampled output
// rather than the construction.
//
// Two of them carry most of the weight. One asserts that from rest the profile
// reproduces ScurveProfile sample for sample, which is a cross-check against an
// independent construction rather than against itself. The other sweeps two
// hundred starting states and, for each, differentiates the output and checks
// every limit -- because "respects the limits" is a claim about the stream a
// controller receives, not about the arithmetic that produced it.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

#include "motionkit/core/motion_state.hpp"
#include "motionkit/core/trajectory.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

constexpr Scalar kVelocity = 2.0;
constexpr Scalar kAcceleration = 6.0;
constexpr Scalar kJerk = 40.0;

MotionLimits limits() { return MotionLimits{kVelocity, kAcceleration, kJerk}; }

/// The worst magnitudes the sampled stream reaches, and whether it arrives.
struct Trace {
  Scalar velocity{0.0};
  Scalar acceleration{0.0};
  Scalar jerk{0.0};
  Scalar arrival_error{0.0};
  Scalar consistency{0.0};
};

/// Samples a profile densely and reports what the stream actually did.
///
/// `consistency` differentiates position and compares it with the velocity the
/// profile reports. A profile whose position and velocity disagree is one whose
/// integrator and sampler were written from different equations, and nothing
/// downstream would notice.
Trace traceOf(const ReachProfile& profile, std::size_t steps = 3000) {
  Trace trace;
  const Scalar dt = profile.duration() / static_cast<Scalar>(steps);
  std::vector<MotionSample> samples;
  samples.reserve(steps + 1);
  for (std::size_t i = 0; i <= steps; ++i) {
    samples.push_back(profile.sample(static_cast<Scalar>(i) * dt));
  }
  for (const MotionSample& s : samples) {
    trace.velocity = std::max(trace.velocity, std::abs(s.velocity));
    trace.acceleration = std::max(trace.acceleration, std::abs(s.acceleration));
    trace.jerk = std::max(trace.jerk, std::abs(s.jerk));
  }
  // A profile lasting nanoseconds cannot be differentiated numerically: the
  // position differences are rounding and the quotient is noise divided by
  // almost nothing. Below a microsecond the check is skipped rather than
  // reported as a disagreement it is not.
  if (profile.duration() > 1e-6) {
    for (std::size_t i = 1; i < steps; ++i) {
      const Scalar slope =
          (samples[i + 1].position - samples[i - 1].position) / (2.0 * dt);
      trace.consistency =
          std::max(trace.consistency, std::abs(slope - samples[i].velocity));
    }
  }
  // Sampled just *before* the end, deliberately. sample() substitutes the
  // promised goal at and after duration(), so asking it where the axis finishes
  // is asking it what it was told to say -- a check that cannot fail and
  // therefore checks nothing. What matters is whether the seven segments
  // actually land there, which is the last interior sample.
  const Scalar last = profile.duration() > 0.0 ? profile.duration() * (1.0 - 1e-12) : 0.0;
  const MotionSample end = profile.sample(last);
  trace.arrival_error = std::abs(end.position - profile.goalPosition()) +
                        std::abs(end.velocity) + std::abs(end.acceleration);
  return trace;
}

}  // namespace

TEST(ReachProfile, FromRestItReproducesTheRestToRestProfileExactly) {
  // The strongest check available: two constructions that share no code should
  // agree. ScurveProfile solves the symmetric case in closed form; ReachProfile
  // reaches the same answer through a bisection that knows nothing about
  // symmetry. Agreement is evidence; a hand-written expected duration would
  // only be evidence that the author repeated the implementation.
  Scalar worst_duration = 0.0;
  Scalar worst_position = 0.0;
  for (const Scalar goal : {0.01, 0.05, 0.4, 1.0, 3.0, -0.7}) {
    const auto rest_to_rest = ScurveProfile::plan(0.0, goal, limits());
    const auto reached = ReachProfile::plan(MotionState{0.0, 0.0, 0.0}, goal, limits());
    ASSERT_TRUE(rest_to_rest);
    ASSERT_TRUE(reached) << static_cast<int>(reached.error);

    worst_duration = std::max(worst_duration, std::abs(rest_to_rest.value.duration() -
                                                       reached.value.duration()));
    for (int i = 0; i <= 200; ++i) {
      const Scalar t = rest_to_rest.value.duration() * static_cast<Scalar>(i) / 200.0;
      worst_position =
          std::max(worst_position, std::abs(rest_to_rest.value.sample(t).position -
                                            reached.value.sample(t).position));
    }
    EXPECT_FALSE(reached.value.reversed()) << "a move from rest never turns round";
  }
  std::printf("rest-to-rest agreement: %.3e s, %.3e position units\n", worst_duration,
              worst_position);
  EXPECT_LT(worst_duration, 1e-9);
  EXPECT_LT(worst_position, 1e-9);
}

TEST(ReachProfile, EveryStartingStateProducesAStreamInsideTheLimits) {
  // Two hundred starting states from a fixed seed, spanning velocities in both
  // directions and accelerations that sometimes begin past the limit.
  std::mt19937 generator(20260910U);
  std::uniform_real_distribution<Scalar> velocity(-kVelocity, kVelocity);
  std::uniform_real_distribution<Scalar> acceleration(-1.4 * kAcceleration,
                                                      1.4 * kAcceleration);
  std::uniform_real_distribution<Scalar> distance(-2.0, 2.0);

  Trace worst;
  Scalar velocity_ratio = 0.0;
  std::size_t reversals = 0;
  std::size_t outside = 0;
  for (int trial = 0; trial < 200; ++trial) {
    const MotionState from{0.0, velocity(generator), acceleration(generator)};
    const Scalar goal = distance(generator);
    const auto planned = ReachProfile::plan(from, goal, limits());
    ASSERT_TRUE(planned) << "trial " << trial;

    reversals += planned.value.reversed() ? 1U : 0U;
    outside += planned.value.startedOutsideAccelerationLimit() ? 1U : 0U;

    const Trace trace = traceOf(planned.value);
    worst.velocity = std::max(worst.velocity, trace.velocity);
    // An axis at the velocity limit while still accelerating must exceed it:
    // acceleration cannot step to zero, so a0 * a0 / 2j of further speed is
    // already committed before the planner does anything at all. The ceiling
    // is what the state arrives owing, not the limit on its own.
    const Scalar carried = std::abs(from.velocity) +
                           ((from.acceleration * from.acceleration) / (2.0 * kJerk));
    velocity_ratio =
        std::max(velocity_ratio, trace.velocity / std::max(kVelocity, carried));
    // The starting acceleration is allowed to be past the limit -- the profile
    // accepts a machine that is already misbehaving -- so what must hold is
    // that the profile never makes it worse.
    const Scalar ceiling = std::max(kAcceleration, std::abs(from.acceleration));
    worst.acceleration = std::max(worst.acceleration, trace.acceleration / ceiling);
    worst.jerk = std::max(worst.jerk, trace.jerk);
    worst.arrival_error = std::max(worst.arrival_error, trace.arrival_error);
    worst.consistency = std::max(worst.consistency, trace.consistency);
  }
  std::printf(
      "200 starts: peak |v| %.4f of %.1f, |a| %.4f of its ceiling, |j| %.3f of %.1f\n",
      worst.velocity, kVelocity, worst.acceleration, worst.jerk, kJerk);
  std::printf("            arrival error %.3e, position/velocity disagreement %.3e\n",
              worst.arrival_error, worst.consistency);
  std::printf("            %zu reversed, %zu started outside the acceleration limit\n",
              reversals, outside);

  std::printf("            peak speed was %.4f of what the start already owed\n",
              velocity_ratio);
  EXPECT_LT(velocity_ratio, 1.001);
  EXPECT_LT(worst.acceleration, 1.001);
  EXPECT_LT(worst.jerk, kJerk * 1.001);
  EXPECT_LT(worst.arrival_error, 1e-9);
  EXPECT_LT(worst.consistency, 1e-3);
  // The spread has to actually contain the interesting cases, or the sweep is
  // two hundred easy moves wearing a costume.
  EXPECT_GT(reversals, 20U);
  EXPECT_GT(outside, 20U);
}

TEST(ReachProfile, AnAxisRunningAtAGoalTooCloseToStopShortOfOvershootsAndComesBack) {
  // 2 m/s at a goal 10 mm away. There is no jerk-limited way to arrive at rest
  // without going past it first, and the honest answer is a move that does.
  const MotionState racing{0.0, 2.0, 0.0};
  const auto planned = ReachProfile::plan(racing, 0.01, limits());
  ASSERT_TRUE(planned);
  EXPECT_TRUE(planned.value.reversed());

  Scalar furthest = 0.0;
  for (int i = 0; i <= 500; ++i) {
    const Scalar t = planned.value.duration() * static_cast<Scalar>(i) / 500.0;
    furthest = std::max(furthest, planned.value.sample(t).position);
  }
  std::printf("overshoot: reached %.4f m before returning to %.3f m\n", furthest, 0.01);
  EXPECT_GT(furthest, 0.01) << "it must pass the goal";
  const MotionSample end = planned.value.sample(planned.value.duration());
  EXPECT_NEAR(end.position, 0.01, 1e-12);
  EXPECT_NEAR(end.velocity, 0.0, 1e-12);
}

TEST(ReachProfile, AnAxisTravellingAwayFromItsGoalTurnsRound) {
  const MotionState leaving{0.0, 1.5, 0.0};
  const auto planned = ReachProfile::plan(leaving, -0.8, limits());
  ASSERT_TRUE(planned);
  EXPECT_TRUE(planned.value.reversed());
  EXPECT_LT(planned.value.cruiseVelocity(), 0.0)
      << "the middle of the move runs the other way";
  const MotionSample end = planned.value.sample(planned.value.duration());
  EXPECT_NEAR(end.position, -0.8, 1e-12);
}

TEST(ReachProfile, AnAxisAlreadyHeadedThereWithRoomDoesNotTurnRound) {
  const MotionState running{0.0, 1.0, 0.0};
  const auto planned = ReachProfile::plan(running, 3.0, limits());
  ASSERT_TRUE(planned);
  EXPECT_FALSE(planned.value.reversed());
  EXPECT_GT(planned.value.cruiseVelocity(), 0.0);
  // And it is quicker than stopping first would be, which is the entire reason
  // this profile exists.
  const auto stop = StopProfile::plan(running, limits());
  ASSERT_TRUE(stop);
  const auto restart = ScurveProfile::plan(stop.value.restPosition(), 3.0, limits());
  ASSERT_TRUE(restart);
  const Scalar stop_then_go = stop.value.duration() + restart.value.duration();
  std::printf("carrying speed through: %.4f s against %.4f s for stop-then-go\n",
              planned.value.duration(), stop_then_go);
  EXPECT_LT(planned.value.duration(), stop_then_go);
}

TEST(ReachProfile, AcceptsAStateAlreadyPastTheAccelerationLimitAndSaysSo) {
  const MotionState misbehaving{0.0, 0.5, 3.0 * kAcceleration};
  const auto planned = ReachProfile::plan(misbehaving, 1.0, limits());
  ASSERT_TRUE(planned) << "refusing to plan for a machine already out of limits "
                          "has the logic backwards";
  EXPECT_TRUE(planned.value.startedOutsideAccelerationLimit());
  const Trace trace = traceOf(planned.value);
  // It may not make things worse: nothing after the start exceeds where it
  // began.
  EXPECT_LE(trace.acceleration, std::abs(misbehaving.acceleration) * 1.001);
  EXPECT_LT(trace.arrival_error, 1e-9);
}

TEST(ReachProfile, AGoalWhereTheAxisAlreadyRestsTakesNoTime) {
  const auto planned = ReachProfile::plan(MotionState{0.4, 0.0, 0.0}, 0.4, limits());
  ASSERT_TRUE(planned);
  EXPECT_NEAR(planned.value.duration(), 0.0, 1e-12);
  EXPECT_FALSE(planned.value.reversed());
  EXPECT_NEAR(planned.value.sample(0.0).position, 0.4, 1e-15);
}

TEST(ReachProfile, ClampsSamplesOutsideTheMoveRatherThanExtrapolating) {
  const auto planned = ReachProfile::plan(MotionState{0.0, 0.8, 0.0}, 1.0, limits());
  ASSERT_TRUE(planned);
  const MotionSample before = planned.value.sample(-5.0);
  EXPECT_NEAR(before.position, 0.0, 1e-15);
  EXPECT_NEAR(before.velocity, 0.8, 1e-15);
  const MotionSample after = planned.value.sample(planned.value.duration() * 10.0);
  EXPECT_NEAR(after.position, 1.0, 1e-15);
  EXPECT_NEAR(after.velocity, 0.0, 1e-15);
  EXPECT_NEAR(after.acceleration, 0.0, 1e-15);
}

TEST(ReachProfile, RejectsNonFiniteInputAndUnusableLimits) {
  const Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();
  EXPECT_EQ(ReachProfile::plan(MotionState{0.0, nan, 0.0}, 1.0, limits()).error,
            TrajectoryError::NonFiniteInput);
  EXPECT_EQ(ReachProfile::plan(MotionState{0.0, 0.0, 0.0}, nan, limits()).error,
            TrajectoryError::NonFiniteInput);
  EXPECT_NE(ReachProfile::plan(MotionState{0.0, 0.0, 0.0}, 1.0, MotionLimits{}).error,
            TrajectoryError::None);
}

}  // namespace motionkit
