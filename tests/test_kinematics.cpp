// SPDX-License-Identifier: Apache-2.0
//
// Forward kinematics, the Jacobian, and an iterative inverse.
//
// The Jacobian is the part worth testing hardest. It is a derivative, so it can
// be checked against the thing it claims to be the derivative of -- and a sign
// or frame error in one of its six rows produces a solver that converges for
// some targets and spirals for others, which is a far worse failure than one
// that never works.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/so3.hpp"

namespace motionkit {
namespace {

constexpr Scalar kPi = std::numbers::pi_v<Scalar>;

/// A configuration well away from any singularity, used wherever a test needs
/// an ordinary pose rather than an interesting one.
std::array<Scalar, 6> comfortablePose() { return {0.3, -0.6, 1.0, 0.4, 0.7, -0.2}; }

SE3 mustForward(const SerialChain& chain, std::span<const Scalar> q) {
  const auto pose = chain.forward(q);
  EXPECT_EQ(pose.error, KinematicsError::None) << toString(pose.error);
  return pose.value;
}

TEST(ForwardKinematics, TheZeroConfigurationPutsTheToolWhereTheGeometrySays) {
  const SerialChain arm = SerialChain::sixAxisExample();
  ASSERT_EQ(arm.jointCount(), 6u);

  const std::array<Scalar, 6> zero{};
  const SE3 tool = mustForward(arm, zero);
  // 0.15 base + 0.43 upper arm + 0.39 forearm + 0.10 wrist-to-tool, straight up.
  EXPECT_TRUE(tool.translation().isApprox(Vec3{0.0, 0.0, 1.07}, 1e-12));
  EXPECT_TRUE(tool.rotation().isApprox(SO3{}, 1e-12));
}

TEST(ForwardKinematics, RotatingTheBaseDoesNotMoveAToolOnItsAxis) {
  // At zero the tool sits on joint 1's axis, so turning joint 1 rotates the
  // tool without translating it. An invariant that a sign error in the screw
  // transform breaks immediately.
  const SerialChain arm = SerialChain::sixAxisExample();
  const std::array<Scalar, 6> q{kPi / 3.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const SE3 tool = mustForward(arm, q);
  EXPECT_TRUE(tool.translation().isApprox(Vec3{0.0, 0.0, 1.07}, 1e-12));
  EXPECT_NEAR(tool.rotation().angleTo(SO3{}), kPi / 3.0, 1e-12);
}

TEST(ForwardKinematics, PitchingTheShoulderSwingsTheArmIntoTheXAxis) {
  // Joint 2 rotates about +Y through (0, 0, 0.15). A quarter turn takes
  // everything above it from straight up to pointing along +X, so the tool
  // lands 1.07 - 0.15 = 0.92 out and at the shoulder's height.
  const SerialChain arm = SerialChain::sixAxisExample();
  const std::array<Scalar, 6> q{0.0, kPi / 2.0, 0.0, 0.0, 0.0, 0.0};
  const SE3 tool = mustForward(arm, q);
  EXPECT_TRUE(tool.translation().isApprox(Vec3{0.92, 0.0, 0.15}, 1e-12));
}

// ---------------------------------------------------------------------------
// The Jacobian, against the derivative it claims to be
// ---------------------------------------------------------------------------

/// Column `i` of the Jacobian, obtained by moving joint `i` and watching.
///
/// Central differences, and the angular part taken as the rotation vector of
/// the relative rotation applied on the LEFT -- the same base-frame convention
/// the analytic Jacobian uses. Taking it on the right would produce a
/// body-frame twist, and the two agree only when the tool happens to be
/// aligned with the base.
std::array<Scalar, kTwistSize> numericalColumn(const SerialChain& chain,
                                               std::span<const Scalar> q,
                                               std::size_t index, Scalar h) {
  std::vector<Scalar> forward_q(q.begin(), q.end());
  std::vector<Scalar> backward_q(q.begin(), q.end());
  forward_q[index] += h;
  backward_q[index] -= h;

  const SE3 plus = mustForward(chain, forward_q);
  const SE3 minus = mustForward(chain, backward_q);

  const Vec3 linear = (plus.translation() - minus.translation()) / (2.0 * h);
  const Vec3 angular =
      (plus.rotation() * minus.rotation().inverse()).rotationVector() / (2.0 * h);
  return {linear.x, linear.y, linear.z, angular.x, angular.y, angular.z};
}

TEST(Jacobian, EveryColumnMatchesTheNumericalDerivative) {
  const SerialChain arm = SerialChain::sixAxisExample();
  constexpr Scalar h = 1e-6;

  // Several configurations, because a frame error can vanish at one pose. The
  // zero configuration is included deliberately: it is singular, and a
  // derivative is still perfectly well defined there.
  const std::array<std::array<Scalar, 6>, 4> poses{{
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      comfortablePose(),
      {-1.1, 0.9, -0.7, 2.0, -1.3, 0.5},
      {kPi / 2.0, -kPi / 4.0, kPi / 3.0, -kPi / 6.0, kPi / 5.0, kPi},
  }};

  Scalar worst = 0.0;
  for (const auto& q : poses) {
    std::array<Scalar, kTwistSize * 6> analytic{};
    ASSERT_EQ(arm.jacobian(q, analytic), KinematicsError::None);

    for (std::size_t i = 0; i < 6; ++i) {
      const auto numeric = numericalColumn(arm, q, i, h);
      for (std::size_t r = 0; r < kTwistSize; ++r) {
        const Scalar difference = std::fabs(analytic[r * 6 + i] - numeric[r]);
        worst = std::max(worst, difference);
        EXPECT_LT(difference, 1e-7) << "column " << i << ", row " << r;
      }
    }
  }
  std::printf("  worst analytic-vs-numerical Jacobian element: %.3e\n", worst);
}

// ---------------------------------------------------------------------------
// Manipulability and the inverse solve
// ---------------------------------------------------------------------------

TEST(Manipulability, IsZeroWhereTheArmIsSingularAndPositiveWhereItIsNot) {
  const SerialChain arm = SerialChain::sixAxisExample();

  // Straight up: joints 1, 4 and 6 are collinear, so the arm cannot produce
  // angular velocity about one direction at all.
  const std::array<Scalar, 6> straight_up{};
  const auto degenerate = arm.manipulability(straight_up);
  ASSERT_TRUE(degenerate.hasValue()) << toString(degenerate.error);
  EXPECT_EQ(degenerate.value, 0.0);

  const auto ordinary = arm.manipulability(comfortablePose());
  ASSERT_TRUE(ordinary.hasValue());
  EXPECT_GT(ordinary.value, 1e-4);
}

TEST(InverseKinematics, ReachesAPoseItWasGivenTheAnswerTo) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const std::array<Scalar, 6> truth = comfortablePose();
  const SE3 target = mustForward(arm, truth);

  // Seeded away from the answer, because a solver handed its own solution is
  // not being tested.
  std::array<Scalar, 6> q = truth;
  for (Scalar& value : q) {
    value += 0.2;
  }

  const auto report = arm.inverse(target, q);
  ASSERT_TRUE(report.hasValue()) << toString(report.error);
  EXPECT_LT(report.value.position_error, 1e-6);
  EXPECT_LT(report.value.orientation_error, 1e-6);

  // The POSE matches. The joint angles need not: a six-axis arm reaches most
  // poses in up to eight configurations, and an iterative solver returns
  // whichever one its seed falls into. Asserting on the angles would be
  // asserting on which basin the seed happened to be in.
  const SE3 reached = mustForward(arm, q);
  EXPECT_TRUE(reached.isApprox(target, 1e-6, 1e-6));
  std::printf("  IK converged in %zu iterations, worst step %.3f rad\n",
              report.value.iterations, report.value.largest_step);
}

// The reason IkOptions::damping is not optional.
//
// Near a singularity the Jacobian's smallest singular value goes to zero, and
// the undamped step -- the Moore-Penrose pseudo-inverse -- divides by it. The
// arm is asked to move a fraction of a degree and answers with radians.
TEST(KinematicsSingularity, DampingBoundsTheStepThatOtherwiseDiverges) {
  const SerialChain arm = SerialChain::sixAxisExample();
  // Comfortable everywhere except the wrist, which is nearly straight: joints
  // 4 and 6 are then almost parallel and one rotational freedom is nearly gone.
  const std::array<Scalar, 6> near_singular{0.3, -0.6, 1.0, 0.4, 1e-4, -0.2};

  const auto measure = arm.manipulability(near_singular);
  ASSERT_TRUE(measure.hasValue());

  // A one-milliradian rotation, in a direction the wrist has almost no
  // authority over.
  const SE3 here = mustForward(arm, near_singular);
  const SE3 target(SO3::fromAxisAngle(Vec3{1.0, 0.0, 0.0}, 1e-3) * here.rotation(),
                   here.translation());

  IkOptions options;
  options.max_iterations = 1;
  options.max_step = 1e9;  // so the clamp cannot hide the magnitude
  options.enforce_joint_limits = false;

  options.damping = 0.0;
  std::array<Scalar, 6> undamped_q = near_singular;
  const auto undamped = arm.inverse(target, undamped_q, options);

  options.damping = 1e-3;
  std::array<Scalar, 6> damped_q = near_singular;
  const auto damped = arm.inverse(target, damped_q, options);

  std::printf(
      "  manipulability %.3e near the wrist singularity\n"
      "  for a 1 mrad rotation: undamped step %.3f rad, damped step %.5f rad\n",
      measure.value, undamped.value.largest_step, damped.value.largest_step);

  // The undamped solver asks for a joint motion enormously larger than the tool
  // motion requested. The damped one does not.
  EXPECT_GT(undamped.value.largest_step, 1.0);
  EXPECT_LT(damped.value.largest_step, 0.5);
  EXPECT_GT(undamped.value.largest_step, damped.value.largest_step * 10.0);
}

// ---------------------------------------------------------------------------
// Limits and refusals
// ---------------------------------------------------------------------------

TEST(InverseKinematics, PinningAJointAtItsStopIsReportedAsSuchNotAsGivingUp) {
  // One joint about Z, allowed to turn barely at all, asked for a quarter turn.
  // The distinction matters to a caller: a solve that ran out of iterations
  // might succeed with more, and one pressed against a mechanical stop never
  // will.
  const std::array<RevoluteJoint, 1> stiff{
      RevoluteJoint{Vec3{0.0, 0.0, 1.0}, Vec3{}, -0.1, 0.1}};
  const auto chain = SerialChain::build(stiff, SE3::fromTranslation(Vec3{1.0, 0.0, 0.0}));
  ASSERT_TRUE(chain.hasValue());

  const SE3 quarter_turn(SO3::fromAxisAngle(Vec3{0.0, 0.0, 1.0}, kPi / 2.0),
                         Vec3{0.0, 1.0, 0.0});
  std::array<Scalar, 1> q{0.0};
  const auto report = chain.value.inverse(quarter_turn, q);
  EXPECT_EQ(report.error, KinematicsError::OutsideJointLimits) << toString(report.error);
  EXPECT_LE(q[0], 0.1);
  EXPECT_GE(q[0], -0.1);
}

TEST(InverseKinematics, AnUnreachableTargetLeavesEveryJointSomewhereTheArmCanBe) {
  const SerialChain arm = SerialChain::sixAxisExample();
  // Five metres away, against a reach of about one. The arm extends as far as
  // it can and stops -- without, as it happens, pinning any joint, because
  // what stops it is geometry rather than a mechanical stop.
  const SE3 unreachable = SE3::fromTranslation(Vec3{5.0, 0.0, 0.0});
  std::array<Scalar, 6> q = comfortablePose();

  const auto report = arm.inverse(unreachable, q);
  EXPECT_EQ(report.error, KinematicsError::DidNotConverge) << toString(report.error);

  // Whatever it did, the configuration it leaves behind is one the arm can
  // physically hold. Returning an out-of-range answer would be worse than
  // failing, because the failure would then be discovered by the machine.
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_GE(q[i], arm.joint(i).lower) << "joint " << i;
    EXPECT_LE(q[i], arm.joint(i).upper) << "joint " << i;
  }
}

TEST(SerialChainConstruction, RefusesWhatItCannotInterpret) {
  const std::array<RevoluteJoint, 1> no_direction{
      RevoluteJoint{Vec3{0.0, 0.0, 0.0}, Vec3{}, -1.0, 1.0}};
  EXPECT_EQ(SerialChain::build(no_direction, SE3{}).error,
            KinematicsError::DegenerateAxis);

  EXPECT_EQ(SerialChain::build({}, SE3{}).error, KinematicsError::JointCountUnsupported);

  std::array<RevoluteJoint, kMaxJoints + 1> too_many{};
  EXPECT_EQ(SerialChain::build(too_many, SE3{}).error,
            KinematicsError::JointCountUnsupported);
}

TEST(SerialChainConstruction, AxesAreNormalisedOnceSoNoLaterCallHasToWonder) {
  const std::array<RevoluteJoint, 1> long_axis{
      RevoluteJoint{Vec3{0.0, 0.0, 7.0}, Vec3{}, -kPi, kPi}};
  const auto chain =
      SerialChain::build(long_axis, SE3::fromTranslation(Vec3{1.0, 0.0, 0.0}));
  ASSERT_TRUE(chain.hasValue());

  // A quarter turn is a quarter turn regardless of how long the axis vector
  // was. Without normalisation this would rotate seven times as far.
  const std::array<Scalar, 1> q{kPi / 2.0};
  const SE3 tool = mustForward(chain.value, q);
  EXPECT_TRUE(tool.translation().isApprox(Vec3{0.0, 1.0, 0.0}, 1e-12));
}

TEST(SerialChainConstruction, WrongSizesAndNonFiniteInputsAreRefused) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const std::array<Scalar, 3> too_few{};
  EXPECT_EQ(arm.forward(too_few).error, KinematicsError::SizeMismatch);

  std::array<Scalar, 6> q{};
  q[2] = std::numeric_limits<Scalar>::quiet_NaN();
  EXPECT_EQ(arm.forward(q).error, KinematicsError::NonFiniteInput);

  std::array<Scalar, kTwistSize * 6> jac{};
  const std::array<Scalar, 6> good{};
  EXPECT_EQ(arm.jacobian(good, std::span<Scalar>(jac.data(), 6)),
            KinematicsError::SizeMismatch);
}

}  // namespace
}  // namespace motionkit
