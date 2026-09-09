// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// Why a calibration could not be solved.
enum class CalibrationError : std::uint8_t {
  None = 0,
  /// Fewer measurements than the unknowns require.
  NotEnoughSamples,
  /// The measurements do not constrain the answer, however many there are.
  /// Taking more of the same useless pose does not help, which is why this is
  /// reported separately from NotEnoughSamples.
  DegenerateGeometry,
  /// A NaN or infinite input pose.
  NonFiniteInput,
  /// Two sample sets of different length, which cannot be paired.
  SizeMismatch,
  /// The rotation solve did not settle. Reported rather than returned as a
  /// best effort, because a rotation that has not converged is not a rotation
  /// anyone should mount a camera against.
  DidNotConverge,
};

/// A human-readable name for a CalibrationError.
std::string_view toString(CalibrationError error) noexcept;

/// Where the tool tip sits relative to the flange, and how well that is known.
struct ToolPointCalibration {
  /// The tool tip in the flange frame -- the answer, and the thing to put into
  /// the robot controller.
  Vec3 flange_t_tool;
  /// Where the touched point turned out to be, in the base frame.
  ///
  /// A nuisance parameter, and worth returning anyway: it is solved for at the
  /// same time, so a wrong value points at the fixture rather than the tool.
  Vec3 base_p_touched;
  /// Root-mean-square distance, in metres, by which the poses disagree about
  /// where the tip was.
  Scalar residual_rms{0.0};
  /// The worst single disagreement, in metres. Reported next to the RMS
  /// because one bad touch among twelve good ones barely moves the average and
  /// is exactly the thing worth noticing.
  Scalar residual_worst{0.0};
  /// How many poses were used.
  std::size_t samples{0};
};

/// Where the camera sits relative to the flange, and how well that is known.
struct HandEyeCalibration {
  /// The camera frame in the flange frame -- the answer.
  SE3 flange_T_camera;
  /// RMS disagreement in the rotation equation, in radians.
  Scalar rotation_residual{0.0};
  /// RMS disagreement in the translation equation, in metres.
  Scalar translation_residual{0.0};
  /// The largest angle between any two motion rotation axes, in radians.
  ///
  /// This is the number that decides whether the problem was solvable at all.
  /// Hand-eye calibration needs two motions whose rotation axes are not
  /// parallel; if every motion turns about the same axis, the camera position
  /// along that axis is invisible to the data and no amount of averaging
  /// recovers it. Near zero means the answer is not to be trusted whatever the
  /// residuals say.
  Scalar axis_spread{0.0};
  /// How many motions were formed from the supplied stations.
  std::size_t motions{0};
};

/// Finds the tool tip from several poses that touch one fixed point.
///
/// The standard shop-floor procedure: jog the tip onto the same physical point
/// -- a fixture pin, a scribed mark -- from several distinctly different
/// orientations, and record the flange pose each time. Every pose says the tip
/// is somewhere; the tip is where they agree.
///
/// Both the tool offset and the touched point come out of one linear solve, six
/// unknowns from three equations per pose, so three poses are the arithmetic
/// minimum and more are strictly better. What matters is not the count but the
/// **spread of orientations**: poses that differ only in position say the same
/// thing three times, and the solve is refused as degenerate rather than
/// answered from whatever the rounding left behind.
///
/// The residuals are the useful output. A tip known to a tenth of a millimetre
/// and one known to five are the same struct, and only `residual_rms` and
/// `residual_worst` tell them apart.
[[nodiscard]] Expected<ToolPointCalibration, CalibrationError> calibrateToolPoint(
    std::span<const SE3> base_T_flange);

/// Finds the camera pose on the flange, from a camera watching a fixed target.
///
/// The eye-in-hand problem. At each station the robot reports where its flange
/// is and the camera reports where the target is; the target has not moved and
/// the camera has not moved on the flange, and those two facts are enough.
/// Consecutive stations give the classic `A X = X B`, with `A` a flange motion,
/// `B` a camera motion and `X` the mounting sought.
///
/// The two spans are paired by index and must be the same length. Four stations
/// are the practical minimum; the errors that matter come from the geometry
/// rather than the count.
///
/// **Rotation first, then translation, and the rotation is solved as one
/// system.** Written with quaternions the rotation equation is *linear* --
/// `q_A q_X = q_X q_B` becomes `(L(q_A) - R(q_B)) q_X = 0` -- so every station
/// contributes to a single null-space problem rather than to an average of
/// pairwise guesses. Averaging rotations is where hand-eye implementations
/// usually go quietly wrong, because there is no way to average two rotations
/// that is both simple and correct.
///
/// Check `axis_spread` before believing the result. It is the one diagnostic
/// that separates "the data could not answer this" from "the answer is
/// imprecise", and those want different responses from whoever is holding the
/// teach pendant.
[[nodiscard]] Expected<HandEyeCalibration, CalibrationError> calibrateHandEye(
    std::span<const SE3> base_T_flange, std::span<const SE3> camera_T_target);

}  // namespace motionkit
