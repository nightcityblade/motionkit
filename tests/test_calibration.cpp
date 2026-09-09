// SPDX-License-Identifier: Apache-2.0
//
// Tool-point and hand-eye calibration.
//
// Calibration is unusually easy to test honestly, because the answer can be
// chosen first. Every round-trip test here picks a mounting, generates exactly
// the measurements a robot would have produced with that mounting, and asks
// whether the solver finds it again. There is no tolerance anybody negotiated:
// with noise-free input the recovery should be exact to rounding, and it is.
//
// The cases that matter as much are the refusals. A calibration that returns a
// plausible number from data that cannot determine it is worse than one that
// fails, because somebody will type the number into a controller.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include "motionkit/core/calibration.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/so3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

/// Poses that touch `point` with the tool tip at `tool`, from assorted
/// orientations. This is the measurement a careful operator produces.
std::vector<SE3> touchesOf(const Vec3& tool, const Vec3& point,
                           std::span<const SO3> orientations) {
  std::vector<SE3> poses;
  poses.reserve(orientations.size());
  for (const SO3& rotation : orientations) {
    // The flange must sit wherever puts the tip on the point.
    poses.emplace_back(rotation, point - (rotation * tool));
  }
  return poses;
}

std::vector<SO3> spreadOfOrientations() {
  return {
      SO3::fromRPY(0.0, 0.0, 0.0),  SO3::fromRPY(0.4, -0.3, 0.9),
      SO3::fromRPY(-0.7, 0.5, 0.2), SO3::fromRPY(0.2, 1.1, -0.6),
      SO3::fromRPY(1.0, 0.3, 0.4),  SO3::fromRPY(-0.5, -0.8, 1.3),
  };
}

/// What a camera on the flange would have seen, given the mounting `hand_eye`
/// and a target standing still at `base_T_target`.
std::vector<SE3> viewsOf(std::span<const SE3> base_T_flange, const SE3& hand_eye,
                         const SE3& base_T_target) {
  std::vector<SE3> views;
  views.reserve(base_T_flange.size());
  for (const SE3& flange : base_T_flange) {
    views.push_back((flange * hand_eye).inverse() * base_T_target);
  }
  return views;
}

std::vector<SE3> stations() {
  return {
      SE3{SO3::fromRPY(0.0, 0.0, 0.0), Vec3{0.60, 0.00, 0.40}},
      SE3{SO3::fromRPY(0.3, -0.4, 0.5), Vec3{0.55, 0.12, 0.46}},
      SE3{SO3::fromRPY(-0.6, 0.2, -0.3), Vec3{0.63, -0.09, 0.38}},
      SE3{SO3::fromRPY(0.1, 0.8, 0.9), Vec3{0.58, 0.05, 0.51}},
      SE3{SO3::fromRPY(-0.4, -0.7, 0.2), Vec3{0.66, -0.15, 0.43}},
      SE3{SO3::fromRPY(0.9, 0.1, -0.8), Vec3{0.52, 0.18, 0.35}},
  };
}

}  // namespace

// --- tool point --------------------------------------------------------------

TEST(ToolPointCalibration, RecoversAToolOffsetItWasNeverTold) {
  const Vec3 tool{0.031, -0.017, 0.184};
  const Vec3 point{0.612, -0.204, 0.338};
  const std::vector<SO3> orientations = spreadOfOrientations();
  const std::vector<SE3> poses = touchesOf(tool, point, orientations);

  const auto solved = calibrateToolPoint(poses);
  ASSERT_TRUE(solved) << toString(solved.error);
  const Scalar miss = (solved.value.flange_t_tool - tool).norm();
  std::printf("tool point recovered to %.3e m\n", miss);
  EXPECT_LT(miss, 1e-12);
  // The touched point is solved for at the same time and is just as exact.
  EXPECT_LT((solved.value.base_p_touched - point).norm(), 1e-12);
  EXPECT_LT(solved.value.residual_rms, 1e-12);
  EXPECT_EQ(solved.value.samples, poses.size());
}

TEST(ToolPointCalibration, TheResidualNoticesOneBadTouchTheAverageWouldHide) {
  const Vec3 tool{0.031, -0.017, 0.184};
  const Vec3 point{0.612, -0.204, 0.338};
  std::vector<SE3> poses = touchesOf(tool, point, spreadOfOrientations());

  // One touch a millimetre off, as happens when the operator's hand slips.
  poses[2] = SE3{poses[2].rotation(), poses[2].translation() + Vec3{0.001, 0.0, 0.0}};

  const auto solved = calibrateToolPoint(poses);
  ASSERT_TRUE(solved);
  // The RMS is dragged down by the five good poses; the worst case is not.
  // This is the entire reason both are reported.
  std::printf("one bad touch: rms %.3e m, worst %.3e m\n", solved.value.residual_rms,
              solved.value.residual_worst);
  EXPECT_GT(solved.value.residual_worst, solved.value.residual_rms * 1.5);
  EXPECT_GT(solved.value.residual_worst, 3e-4);
}

TEST(ToolPointCalibration, RefusesPosesThatDifferOnlyInPosition) {
  const SO3 same = SO3::fromRPY(0.2, -0.4, 0.6);
  const std::vector<SE3> poses{
      SE3{same, Vec3{0.5, 0.0, 0.3}},
      SE3{same, Vec3{0.6, 0.1, 0.3}},
      SE3{same, Vec3{0.4, -0.1, 0.35}},
      SE3{same, Vec3{0.55, 0.05, 0.28}},
  };
  // Four poses, all useless in the same way. Taking a fifth would not help,
  // which is why this is DegenerateGeometry and not NotEnoughSamples.
  EXPECT_EQ(calibrateToolPoint(poses).error, CalibrationError::DegenerateGeometry);
}

TEST(ToolPointCalibration, RefusesFewerThanThreePoses) {
  const std::vector<SE3> poses =
      touchesOf(Vec3{0.0, 0.0, 0.1}, Vec3{0.5, 0.0, 0.3},
                std::span<const SO3>{spreadOfOrientations()}.first(2));
  EXPECT_EQ(calibrateToolPoint(poses).error, CalibrationError::NotEnoughSamples);
}

TEST(ToolPointCalibration, RejectsANonFinitePose) {
  std::vector<SE3> poses =
      touchesOf(Vec3{0.0, 0.0, 0.1}, Vec3{0.5, 0.0, 0.3}, spreadOfOrientations());
  poses[1] =
      SE3{poses[1].rotation(), Vec3{std::numeric_limits<Scalar>::quiet_NaN(), 0.0, 0.0}};
  EXPECT_EQ(calibrateToolPoint(poses).error, CalibrationError::NonFiniteInput);
}

// --- hand-eye ----------------------------------------------------------------

TEST(HandEyeCalibration, RecoversACameraMountingItWasNeverTold) {
  const SE3 mounting{SO3::fromRPY(0.15, -0.62, 1.05), Vec3{0.043, -0.028, 0.091}};
  const SE3 target{SO3::fromRPY(0.0, 0.0, 0.7), Vec3{1.10, 0.25, 0.05}};
  const std::vector<SE3> flange = stations();
  const std::vector<SE3> views = viewsOf(flange, mounting, target);

  const auto solved = calibrateHandEye(flange, views);
  ASSERT_TRUE(solved) << toString(solved.error);

  const Scalar turned =
      solved.value.flange_T_camera.rotation().angleTo(mounting.rotation());
  const Scalar moved =
      (solved.value.flange_T_camera.translation() - mounting.translation()).norm();
  std::printf("hand-eye recovered to %.3e rad, %.3e m (axis spread %.3f rad)\n", turned,
              moved, solved.value.axis_spread);
  EXPECT_LT(turned, 1e-12);
  EXPECT_LT(moved, 1e-12);
  EXPECT_LT(solved.value.rotation_residual, 1e-12);
  EXPECT_LT(solved.value.translation_residual, 1e-12);
  EXPECT_EQ(solved.value.motions, flange.size() - 1);
}

TEST(HandEyeCalibration, RefusesMotionsThatAllTurnAboutTheSameAxis) {
  // Every station differs from the last by a rotation about Z and a slide.
  // A perfectly reasonable-looking set of six poses, and it cannot determine
  // where the camera sits along that axis.
  std::vector<SE3> flange;
  for (int i = 0; i < 6; ++i) {
    const Scalar angle = 0.3 * static_cast<Scalar>(i);
    flange.emplace_back(SO3::rotZ(angle), Vec3{0.6, 0.02 * static_cast<Scalar>(i), 0.4});
  }
  const SE3 mounting{SO3::fromRPY(0.15, -0.62, 1.05), Vec3{0.043, -0.028, 0.091}};
  const SE3 target{SO3{}, Vec3{1.10, 0.25, 0.05}};
  const std::vector<SE3> views = viewsOf(flange, mounting, target);

  const auto solved = calibrateHandEye(flange, views);
  EXPECT_EQ(solved.error, CalibrationError::DegenerateGeometry)
      << "six stations that cannot answer must be refused, not averaged";
}

TEST(HandEyeCalibration, RefusesStationsThatDoNotRotateAtAll) {
  std::vector<SE3> flange;
  for (int i = 0; i < 5; ++i) {
    flange.emplace_back(SO3{}, Vec3{0.6 + 0.05 * static_cast<Scalar>(i), 0.0, 0.4});
  }
  const SE3 mounting{SO3::fromRPY(0.1, -0.2, 0.3), Vec3{0.04, -0.02, 0.09}};
  const SE3 target{SO3{}, Vec3{1.1, 0.2, 0.0}};
  EXPECT_EQ(calibrateHandEye(flange, viewsOf(flange, mounting, target)).error,
            CalibrationError::DegenerateGeometry);
}

TEST(HandEyeCalibration, ReportsTheAxisSpreadThatDecidedTheAnswerWasSolvable) {
  const SE3 mounting{SO3::fromRPY(0.15, -0.62, 1.05), Vec3{0.043, -0.028, 0.091}};
  const SE3 target{SO3{}, Vec3{1.10, 0.25, 0.05}};
  const std::vector<SE3> flange = stations();
  const auto solved = calibrateHandEye(flange, viewsOf(flange, mounting, target));
  ASSERT_TRUE(solved);
  // Well-spread stations: the diagnostic should be a substantial angle, not a
  // number that merely cleared the threshold.
  EXPECT_GT(solved.value.axis_spread, 0.3);
  EXPECT_LE(solved.value.axis_spread, std::numbers::pi_v<Scalar> / 2.0 + 1e-12);
}

TEST(HandEyeCalibration, RefusesMismatchedOrTooFewSamples) {
  const std::vector<SE3> flange = stations();
  const SE3 mounting{SO3::fromRPY(0.1, -0.2, 0.3), Vec3{0.04, -0.02, 0.09}};
  const std::vector<SE3> views = viewsOf(flange, mounting, SE3{});
  EXPECT_EQ(calibrateHandEye(flange, std::span<const SE3>{views}.first(4)).error,
            CalibrationError::SizeMismatch);
  EXPECT_EQ(calibrateHandEye(std::span<const SE3>{flange}.first(2),
                             std::span<const SE3>{views}.first(2))
                .error,
            CalibrationError::NotEnoughSamples);
}

TEST(HandEyeCalibration, RejectsANonFiniteObservation) {
  const std::vector<SE3> flange = stations();
  const SE3 mounting{SO3::fromRPY(0.1, -0.2, 0.3), Vec3{0.04, -0.02, 0.09}};
  std::vector<SE3> views = viewsOf(flange, mounting, SE3{});
  views[3] =
      SE3{views[3].rotation(), Vec3{0.0, std::numeric_limits<Scalar>::infinity(), 0.0}};
  EXPECT_EQ(calibrateHandEye(flange, views).error, CalibrationError::NonFiniteInput);
}

TEST(HandEyeCalibration, WithNoisyObservationsTheResidualReportsIt) {
  const SE3 mounting{SO3::fromRPY(0.15, -0.62, 1.05), Vec3{0.043, -0.028, 0.091}};
  const SE3 target{SO3{}, Vec3{1.10, 0.25, 0.05}};
  const std::vector<SE3> flange = stations();
  std::vector<SE3> views = viewsOf(flange, mounting, target);

  // A deterministic wobble of about 0.2 mm and 0.2 mrad on each observation,
  // which is roughly what a decent camera and a printed target give you.
  // Written as a fixed formula rather than drawn from a generator: a
  // calibration test you cannot replay exactly is not evidence of anything.
  constexpr Scalar kShift = 2e-4;
  constexpr Scalar kTilt = 2e-4;
  for (std::size_t i = 0; i < views.size(); ++i) {
    const Scalar phase = static_cast<Scalar>(i) * 1.7;
    const Vec3 nudge{kShift * std::sin(phase), kShift * std::cos(phase * 1.3),
                     kShift * std::sin(phase * 0.7)};
    const SO3 tilt = SO3::fromRotationVector(
        Vec3{kTilt * std::cos(phase), kTilt * std::sin(phase * 1.1), kTilt});
    views[i] = SE3{tilt * views[i].rotation(), views[i].translation() + nudge};
  }

  const auto solved = calibrateHandEye(flange, views);
  ASSERT_TRUE(solved) << toString(solved.error);
  const Scalar turned =
      solved.value.flange_T_camera.rotation().angleTo(mounting.rotation());
  const Scalar moved =
      (solved.value.flange_T_camera.translation() - mounting.translation()).norm();
  std::printf("noisy hand-eye: off by %.3e rad, %.3e m; residual %.3e rad, %.3e m\n",
              turned, moved, solved.value.rotation_residual,
              solved.value.translation_residual);

  // Degrades in proportion rather than collapsing: sub-milliradian and
  // sub-millimetre from noise of that size.
  EXPECT_LT(turned, 1e-3);
  EXPECT_LT(moved, 1e-3);
  // And the residual is no longer zero, which is the whole point of reporting
  // it. A solver whose residual is always ~1e-16 has only ever been shown
  // exact data.
  EXPECT_GT(solved.value.rotation_residual, 1e-6);
  EXPECT_GT(solved.value.translation_residual, 1e-7);
}

TEST(CalibrationErrors, EveryEnumeratorHasAName) {
  EXPECT_EQ(toString(CalibrationError::None), "None");
  EXPECT_EQ(toString(CalibrationError::NotEnoughSamples), "NotEnoughSamples");
  EXPECT_EQ(toString(CalibrationError::DegenerateGeometry), "DegenerateGeometry");
  EXPECT_EQ(toString(CalibrationError::NonFiniteInput), "NonFiniteInput");
  EXPECT_EQ(toString(CalibrationError::SizeMismatch), "SizeMismatch");
  EXPECT_EQ(toString(CalibrationError::DidNotConverge), "DidNotConverge");
}

}  // namespace motionkit
