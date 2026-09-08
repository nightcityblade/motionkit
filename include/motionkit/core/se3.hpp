// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>

#include "motionkit/core/so3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// A rigid-body transform: rotation followed by translation.
///
/// Read `A_T_B` as "the pose of frame B expressed in frame A". Then
/// `A_T_B * B_T_C == A_T_C`, and `A_T_B * p_B == p_A`. Keeping to that naming
/// makes frame-mismatch bugs visible at the call site instead of at the robot.
class SE3 {
 public:
  /// Identity transform.
  constexpr SE3() noexcept = default;

  /// Constructs from a rotation and a translation, applied in that order: a
  /// point is rotated first, then displaced. The translation is therefore the
  /// origin of the child frame expressed in the parent, which is what a CAD
  /// model reports, and not the displacement applied before rotating.
  SE3(const SO3& rotation, const Vec3& translation) noexcept
      : r_(rotation), t_(translation) {}

  /// A pure translation, with no rotation.
  static SE3 fromTranslation(const Vec3& t) noexcept { return {SO3{}, t}; }
  /// A pure rotation about the origin, with no translation.
  static SE3 fromRotation(const SO3& r) noexcept { return {r, Vec3{}}; }

  /// The rotation part.
  [[nodiscard]] const SO3& rotation() const noexcept { return r_; }
  /// The translation part -- the child frame's origin, in the parent frame.
  [[nodiscard]] const Vec3& translation() const noexcept { return t_; }

  /// The inverse transform: `A_T_B.inverse()` is `B_T_A`. Computed as
  /// `(R^-1, -R^-1 t)` rather than by solving anything, so it is exact to
  /// within rounding and cannot fail.
  [[nodiscard]] SE3 inverse() const noexcept;
  /// Composition, applying `rhs` first: `A_T_B * B_T_C` is `A_T_C`. The frame
  /// letters cancelling across the operator is the point of the convention --
  /// a composition that does not cancel is a bug the reader can see.
  SE3 operator*(const SE3& rhs) const noexcept;

  /// Transforms a point: rotate, then translate.
  Vec3 operator*(const Vec3& point) const noexcept;

  /// Transforms a free vector (a direction or velocity): rotation only.
  [[nodiscard]] Vec3 rotateVector(const Vec3& v) const noexcept { return r_ * v; }

  /// Row-major 4x4 homogeneous matrix, for interop with CAD and vision stacks
  /// that speak matrices rather than quaternions.
  [[nodiscard]] std::array<Scalar, 16> matrix() const noexcept;

  /// True when the two poses agree to within `linear_tol` in position and
  /// `angular_tol` radians in orientation.
  ///
  /// Two tolerances rather than one, because there is no defensible way to add
  /// a distance to an angle. Any single-number pose metric secretly multiplies
  /// the rotation by a length -- and the length it picks is a property of the
  /// robot, not of the comparison.
  [[nodiscard]] bool isApprox(const SE3& other, Scalar linear_tol,
                              Scalar angular_tol) const noexcept;

 private:
  SO3 r_;
  Vec3 t_;
};

}  // namespace motionkit
