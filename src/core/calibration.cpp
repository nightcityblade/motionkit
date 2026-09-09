// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/calibration.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/so3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

/// Solves `A x = b` for a symmetric positive-definite `A`, in place.
///
/// Cholesky, and it returns false rather than a number when the matrix is not
/// positive definite -- which for a normal-equations matrix means the
/// measurements did not constrain the answer. That is the whole degeneracy
/// detector: a rank-deficient calibration shows up here as a non-positive
/// pivot, not as a plausible-looking result.
///
/// A second copy of the routine already in kinematics.cpp, which is not ideal.
/// It is a textbook algorithm rather than a convention, so two copies cannot
/// disagree about anything a reader would have to reconcile -- unlike two
/// spellings of a frame convention, which is the duplication ADR-0008 refuses.
template <std::size_t N>
bool solveSymmetric(std::array<Scalar, N * N> a, const std::array<Scalar, N>& b,
                    std::array<Scalar, N>& x) noexcept {
  Scalar scale = 0.0;
  for (std::size_t i = 0; i < N; ++i) {
    scale = std::max(scale, std::abs(a[(i * N) + i]));
  }
  if (!(scale > 0.0)) {
    return false;
  }
  const Scalar floor_pivot = 1e-12 * scale;

  for (std::size_t i = 0; i < N; ++i) {
    for (std::size_t j = 0; j <= i; ++j) {
      Scalar acc = a[(i * N) + j];
      for (std::size_t k = 0; k < j; ++k) {
        acc -= a[(i * N) + k] * a[(j * N) + k];
      }
      if (i == j) {
        if (acc <= floor_pivot) {
          return false;
        }
        a[(i * N) + i] = std::sqrt(acc);
      } else {
        a[(i * N) + j] = acc / a[(j * N) + j];
      }
    }
  }
  for (std::size_t i = 0; i < N; ++i) {
    Scalar acc = b[i];
    for (std::size_t k = 0; k < i; ++k) {
      acc -= a[(i * N) + k] * x[k];
    }
    x[i] = acc / a[(i * N) + i];
  }
  for (std::size_t i = N; i-- > 0;) {
    Scalar acc = x[i];
    for (std::size_t k = i + 1; k < N; ++k) {
      acc -= a[(k * N) + i] * x[k];
    }
    x[i] = acc / a[(i * N) + i];
  }
  return true;
}

using Quat4 = std::array<Scalar, 4>;
using Mat4 = std::array<Scalar, 16>;

Quat4 asQuaternion(const SO3& rotation) noexcept {
  return {rotation.w(), rotation.x(), rotation.y(), rotation.z()};
}

/// The matrix `L` with `L(p) q == p * q`.
Mat4 leftMultiply(const Quat4& q) noexcept {
  return {q[0], -q[1], -q[2], -q[3], q[1], q[0],  -q[3], q[2],
          q[2], q[3],  q[0],  -q[1], q[3], -q[2], q[1],  q[0]};
}

/// The matrix `R` with `R(p) q == q * p`.
Mat4 rightMultiply(const Quat4& q) noexcept {
  return {q[0], -q[1], -q[2], -q[3], q[1], q[0], q[3],  -q[2],
          q[2], -q[3], q[0],  q[1],  q[3], q[2], -q[1], q[0]};
}

bool isFinite(const SE3& pose) noexcept {
  const SO3& r = pose.rotation();
  return std::isfinite(pose.translation().squaredNorm()) && std::isfinite(r.w()) &&
         std::isfinite(r.x()) && std::isfinite(r.y()) && std::isfinite(r.z());
}

/// The unit eigenvector of the smallest eigenvalue of a symmetric 4x4.
///
/// Power iteration on `c I - M`, which turns the smallest eigenvalue of `M`
/// into the largest and so into the one power iteration finds. `c` comes from
/// the Gershgorin bound rather than the trace: any upper bound works, but a
/// loose one flattens the gap between the shifted eigenvalues and slows
/// convergence to a crawl.
bool smallestEigenvector(const Mat4& m, Quat4& out) noexcept {
  Scalar bound = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    Scalar row = 0.0;
    for (std::size_t j = 0; j < 4; ++j) {
      row += std::abs(m[(i * 4) + j]);
    }
    bound = std::max(bound, row);
  }
  if (!(bound > 0.0)) {
    return false;
  }

  // Seeded away from any axis, so the start is not accidentally orthogonal to
  // the eigenvector being sought -- which would leave power iteration with
  // nothing to amplify.
  Quat4 v{0.5, 0.5, 0.5, 0.5};
  Quat4 next{};
  for (int iteration = 0; iteration < 400; ++iteration) {
    for (std::size_t i = 0; i < 4; ++i) {
      Scalar acc = bound * v[i];
      for (std::size_t j = 0; j < 4; ++j) {
        acc -= m[(i * 4) + j] * v[j];
      }
      next[i] = acc;
    }
    Scalar norm = 0.0;
    for (const Scalar component : next) {
      norm += component * component;
    }
    norm = std::sqrt(norm);
    if (!(norm > 0.0)) {
      return false;
    }
    Scalar movement = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
      const Scalar scaled = next[i] / norm;
      movement = std::max(movement, std::abs(scaled - v[i]));
      v[i] = scaled;
    }
    if (movement < 1e-15) {
      break;
    }
  }
  out = v;
  return true;
}

}  // namespace

std::string_view toString(CalibrationError error) noexcept {
  switch (error) {
    case CalibrationError::None:
      return "None";
    case CalibrationError::NotEnoughSamples:
      return "NotEnoughSamples";
    case CalibrationError::DegenerateGeometry:
      return "DegenerateGeometry";
    case CalibrationError::NonFiniteInput:
      return "NonFiniteInput";
    case CalibrationError::SizeMismatch:
      return "SizeMismatch";
    case CalibrationError::DidNotConverge:
      return "DidNotConverge";
  }
  return "Unknown";
}

Expected<ToolPointCalibration, CalibrationError> calibrateToolPoint(
    std::span<const SE3> base_T_flange) {
  // Six unknowns, three equations per pose. Two poses cannot answer however
  // good they are.
  if (base_T_flange.size() < 3) {
    return {ToolPointCalibration{}, CalibrationError::NotEnoughSamples};
  }
  for (const SE3& pose : base_T_flange) {
    if (!isFinite(pose)) {
      return {ToolPointCalibration{}, CalibrationError::NonFiniteInput};
    }
  }

  // Each pose contributes R_i * t - p = -p_i. Rather than hand-reduce that to
  // a normal matrix, the 3x6 block is built literally and accumulated -- the
  // reduction is easy to get subtly wrong and impossible to check by reading.
  std::array<Scalar, 36> normal{};
  std::array<Scalar, 6> rhs{};
  for (const SE3& pose : base_T_flange) {
    const Mat3 rotation = pose.rotation().matrix();
    std::array<Scalar, 18> block{};
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        block[(r * 6) + c] = rotation(r, c);
      }
      block[(r * 6) + 3 + r] = -1.0;
    }
    const Vec3 target = -pose.translation();
    const std::array<Scalar, 3> b{target.x, target.y, target.z};
    for (std::size_t i = 0; i < 6; ++i) {
      for (std::size_t r = 0; r < 3; ++r) {
        rhs[i] += block[(r * 6) + i] * b[r];
        for (std::size_t j = 0; j < 6; ++j) {
          normal[(i * 6) + j] += block[(r * 6) + i] * block[(r * 6) + j];
        }
      }
    }
  }

  std::array<Scalar, 6> solution{};
  if (!solveSymmetric<6>(normal, rhs, solution)) {
    // Poses that differ only in position say the same thing repeatedly, and
    // the normal matrix loses rank. More of them will not help, so this is not
    // NotEnoughSamples.
    return {ToolPointCalibration{}, CalibrationError::DegenerateGeometry};
  }

  ToolPointCalibration result;
  result.flange_t_tool = Vec3{solution[0], solution[1], solution[2]};
  result.base_p_touched = Vec3{solution[3], solution[4], solution[5]};
  result.samples = base_T_flange.size();

  Scalar sum_squared = 0.0;
  for (const SE3& pose : base_T_flange) {
    const Vec3 predicted = pose * result.flange_t_tool;
    const Scalar miss = (predicted - result.base_p_touched).norm();
    sum_squared += miss * miss;
    result.residual_worst = std::max(result.residual_worst, miss);
  }
  result.residual_rms =
      std::sqrt(sum_squared / static_cast<Scalar>(base_T_flange.size()));
  return {result, CalibrationError::None};
}

namespace {

/// One flange motion and the camera motion that must match it.
struct Motion {
  /// `A` -- how the flange moved between two stations.
  SE3 flange;
  /// `B` -- how the target moved in the camera between the same two.
  SE3 camera;
};

Motion motionBetween(std::span<const SE3> base_T_flange,
                     std::span<const SE3> camera_T_target, std::size_t i) noexcept {
  return {base_T_flange[i + 1].inverse() * base_T_flange[i],
          camera_T_target[i + 1] * camera_T_target[i].inverse()};
}

/// Adds one motion's block to the rotation normal matrix.
void accumulateRotation(const Motion& motion, Mat4& normal) noexcept {
  const Mat4 left = leftMultiply(asQuaternion(motion.flange.rotation()));
  const Mat4 right = rightMultiply(asQuaternion(motion.camera.rotation()));
  Mat4 difference{};
  for (std::size_t k = 0; k < 16; ++k) {
    difference[k] = left[k] - right[k];
  }
  for (std::size_t r = 0; r < 4; ++r) {
    for (std::size_t c = 0; c < 4; ++c) {
      Scalar acc = 0.0;
      for (std::size_t k = 0; k < 4; ++k) {
        acc += difference[(k * 4) + r] * difference[(k * 4) + c];
      }
      normal[(r * 4) + c] += acc;
    }
  }
}

/// Adds one motion's block to `(R_A - I) t = R_X t_B - t_A`.
void accumulateTranslation(const Motion& motion, const SO3& rotation,
                           std::array<Scalar, 9>& normal,
                           std::array<Scalar, 3>& rhs) noexcept {
  const Mat3 rotation_a = motion.flange.rotation().matrix();
  const Vec3 target =
      (rotation * motion.camera.translation()) - motion.flange.translation();
  const std::array<Scalar, 3> d{target.x, target.y, target.z};
  std::array<Scalar, 9> block{};
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 0; c < 3; ++c) {
      block[(r * 3) + c] = rotation_a(r, c) - ((r == c) ? 1.0 : 0.0);
    }
  }
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t r = 0; r < 3; ++r) {
      rhs[i] += block[(r * 3) + i] * d[r];
      for (std::size_t j = 0; j < 3; ++j) {
        normal[(i * 3) + j] += block[(r * 3) + i] * block[(r * 3) + j];
      }
    }
  }
}

}  // namespace

Expected<HandEyeCalibration, CalibrationError> calibrateHandEye(
    std::span<const SE3> base_T_flange, std::span<const SE3> camera_T_target) {
  if (base_T_flange.size() != camera_T_target.size()) {
    return {HandEyeCalibration{}, CalibrationError::SizeMismatch};
  }
  // Three stations give two motions, and two motions are the minimum that can
  // pin a rotation in three dimensions.
  if (base_T_flange.size() < 3) {
    return {HandEyeCalibration{}, CalibrationError::NotEnoughSamples};
  }
  for (std::size_t i = 0; i < base_T_flange.size(); ++i) {
    if (!isFinite(base_T_flange[i]) || !isFinite(camera_T_target[i])) {
      return {HandEyeCalibration{}, CalibrationError::NonFiniteInput};
    }
  }

  const std::size_t motions = base_T_flange.size() - 1;
  HandEyeCalibration result;
  result.motions = motions;

  Mat4 rotation_normal{};
  Vec3 reference_axis;
  std::size_t turning = 0;
  for (std::size_t i = 0; i < motions; ++i) {
    const Motion motion = motionBetween(base_T_flange, camera_T_target, i);
    accumulateRotation(motion, rotation_normal);

    const Vec3 turn = motion.flange.rotation().rotationVector();
    const Scalar angle = turn.norm();
    if (angle <= 1e-6) {
      continue;
    }
    const Vec3 axis = turn / angle;
    if (turning == 0) {
      reference_axis = axis;
    } else {
      const Scalar alignment =
          std::clamp(std::abs(axis.dot(reference_axis)), Scalar{0.0}, Scalar{1.0});
      result.axis_spread = std::max(result.axis_spread, std::acos(alignment));
    }
    ++turning;
  }

  // The geometry test, and it is not a formality. If every motion turns about
  // the same axis, the camera's position along that axis never changes what it
  // sees and no quantity of data recovers it.
  //
  // Every axis is measured against the first rather than against every other.
  // That is not an approximation of the test that matters: the axes are all
  // parallel exactly when they are all parallel to the first. It decides
  // degeneracy correctly in one pass, and without a pose history whose length
  // would then cap how many stations a caller may supply.
  if (turning < 2 || result.axis_spread < 1e-3) {
    return {HandEyeCalibration{}, CalibrationError::DegenerateGeometry};
  }

  Quat4 solved{};
  if (!smallestEigenvector(rotation_normal, solved)) {
    return {HandEyeCalibration{}, CalibrationError::DidNotConverge};
  }
  const SO3 rotation = SO3::fromQuaternion(solved[0], solved[1], solved[2], solved[3]);

  // With the rotation known the translation equation is linear. Each individual
  // (R_A - I) is singular -- its own rotation axis is in the null space --
  // which is the axis test above seen from the other side.
  std::array<Scalar, 9> translation_normal{};
  std::array<Scalar, 3> translation_rhs{};
  for (std::size_t i = 0; i < motions; ++i) {
    accumulateTranslation(motionBetween(base_T_flange, camera_T_target, i), rotation,
                          translation_normal, translation_rhs);
  }

  std::array<Scalar, 3> offset{};
  if (!solveSymmetric<3>(translation_normal, translation_rhs, offset)) {
    return {HandEyeCalibration{}, CalibrationError::DegenerateGeometry};
  }
  result.flange_T_camera = SE3{rotation, Vec3{offset[0], offset[1], offset[2]}};

  Scalar rotation_sum = 0.0;
  Scalar translation_sum = 0.0;
  for (std::size_t i = 0; i < motions; ++i) {
    const Motion motion = motionBetween(base_T_flange, camera_T_target, i);
    // The residual is the equation the solve was supposed to satisfy, measured
    // rather than assumed: how far A X is from X B, split into rotation and
    // translation because a radian and a metre do not add.
    const SE3 left = motion.flange * result.flange_T_camera;
    const SE3 right = result.flange_T_camera * motion.camera;
    const Scalar turned = left.rotation().angleTo(right.rotation());
    const Scalar moved = (left.translation() - right.translation()).norm();
    rotation_sum += turned * turned;
    translation_sum += moved * moved;
  }
  const auto count = static_cast<Scalar>(motions);
  result.rotation_residual = std::sqrt(rotation_sum / count);
  result.translation_residual = std::sqrt(translation_sum / count);
  return {result, CalibrationError::None};
}

}  // namespace motionkit
