// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace motionkit {

/// Single scalar typedef so the whole stack can be retargeted (double -> float)
/// without touching call sites. Fixed to double: industrial pose accuracy is
/// specified in micrometres over metre-scale workspaces, which is ~1e-7
/// relative precision -- outside float's comfortable range once transforms are
/// chained through a six-link frame graph.
using Scalar = double;

/// Tolerance used for "is this a unit quaternion / orthonormal matrix" checks.
inline constexpr Scalar kOrthoTolerance = 1e-9;
/// Below this, a rotation angle is treated as zero and the axis as undefined.
inline constexpr Scalar kAngleEpsilon = 1e-12;

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------
/// A three-component vector, used for both points and free vectors.
///
/// Deliberately untagged: the type does not record which frame the components
/// are expressed in, or whether the value is a position or a direction. That
/// information lives in the variable name -- `p_base`, `axis_tool` -- and in
/// the `A_T_B` convention described on SE3. A frame-tagged vector type would
/// catch mismatches at compile time, at the cost of a template parameter on
/// every signature in the library and a conversion at every boundary with a
/// caller who does not use the tags. The naming convention was judged the
/// better trade for a library meant to be called from existing code.
struct Vec3 {
  /// First component. In a Cartesian frame, distance along the x axis.
  Scalar x{0.0};
  /// Second component.
  Scalar y{0.0};
  /// Third component.
  Scalar z{0.0};

  /// The zero vector.
  constexpr Vec3() noexcept = default;
  /// Constructs from three components.
  constexpr Vec3(Scalar vx, Scalar vy, Scalar vz) noexcept : x(vx), y(vy), z(vz) {}

  /// The zero vector. Named for use where `Vec3{}` would not read as a value.
  static constexpr Vec3 zero() noexcept { return {}; }
  /// The unit vector along x.
  static constexpr Vec3 unitX() noexcept { return {1.0, 0.0, 0.0}; }
  /// The unit vector along y.
  static constexpr Vec3 unitY() noexcept { return {0.0, 1.0, 0.0}; }
  /// The unit vector along z.
  static constexpr Vec3 unitZ() noexcept { return {0.0, 0.0, 1.0}; }

  /// Component-wise sum.
  constexpr Vec3 operator+(const Vec3& o) const noexcept {
    return {x + o.x, y + o.y, z + o.z};
  }
  /// Component-wise difference.
  constexpr Vec3 operator-(const Vec3& o) const noexcept {
    return {x - o.x, y - o.y, z - o.z};
  }
  /// Negation.
  constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }
  /// Scales every component by `s`.
  constexpr Vec3 operator*(Scalar s) const noexcept { return {x * s, y * s, z * s}; }
  /// Divides every component by `s`. Division by zero is not checked and
  /// yields infinities or NaN under IEEE 754, as it does for a bare Scalar.
  constexpr Vec3 operator/(Scalar s) const { return {x / s, y / s, z / s}; }

  /// Adds `o` in place.
  constexpr Vec3& operator+=(const Vec3& o) noexcept {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
  /// Subtracts `o` in place.
  constexpr Vec3& operator-=(const Vec3& o) noexcept {
    x -= o.x;
    y -= o.y;
    z -= o.z;
    return *this;
  }
  /// Scales in place by `s`.
  constexpr Vec3& operator*=(Scalar s) noexcept {
    x *= s;
    y *= s;
    z *= s;
    return *this;
  }

  /// Euclidean inner product.
  [[nodiscard]] constexpr Scalar dot(const Vec3& o) const noexcept {
    return x * o.x + y * o.y + z * o.z;
  }
  /// Right-handed cross product: `a.cross(b)` is perpendicular to both, with
  /// magnitude equal to the area of the parallelogram they span.
  [[nodiscard]] constexpr Vec3 cross(const Vec3& o) const noexcept {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  /// Squared Euclidean length. Preferred over norm() for comparisons, since it
  /// avoids the square root and is usable in a constant expression.
  [[nodiscard]] constexpr Scalar squaredNorm() const noexcept { return dot(*this); }
  /// Euclidean length.
  [[nodiscard]] Scalar norm() const noexcept { return std::sqrt(squaredNorm()); }

  /// Throws if the vector is too close to zero to define a direction; callers
  /// that must not throw should test squaredNorm() first.
  [[nodiscard]] Vec3 normalized() const {
    const Scalar n = norm();
    if (n < kAngleEpsilon) {
      throw std::domain_error("motionkit: cannot normalize a near-zero Vec3");
    }
    return *this / n;
  }

  /// Component by index, 0 for x through 2 for z.
  ///
  /// Throws std::out_of_range for any other index. Note that Mat3::operator()
  /// does *not* check its indices. The inconsistency is deliberate: x, y and z
  /// are separate members, so selecting one already costs a branch and the
  /// bounds check rides along free, while Mat3 indexes a std::array by
  /// arithmetic, where checking would add a branch that is not otherwise there.
  constexpr Scalar operator[](std::size_t i) const {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      default:
        throw std::out_of_range("Vec3 index");
    }
  }

  /// True when the distance to `o` is at most `tol`. An absolute test, so
  /// `tol` carries the units of the vector.
  [[nodiscard]] constexpr bool isApprox(const Vec3& o, Scalar tol) const noexcept {
    return (*this - o).squaredNorm() <= tol * tol;
  }
};

/// Scalar-on-the-left multiplication, so `2.0 * v` reads as it does on paper.
constexpr Vec3 operator*(Scalar s, const Vec3& v) noexcept { return v * s; }

// ---------------------------------------------------------------------------
// Mat3 -- row-major 3x3
// ---------------------------------------------------------------------------
/// A 3x3 matrix in row-major storage.
///
/// General-purpose, and not constrained to be a rotation: SO3 owns that
/// invariant and stores a quaternion instead. Mat3 exists for the places a
/// matrix is genuinely the right object -- interop with CAD and vision stacks,
/// and intermediate results such as an inertia tensor or a covariance -- and
/// for isRotation(), which is how a matrix arriving from outside is checked
/// before it is trusted.
struct Mat3 {
  /// Row-major: element (r, c) lives at d[r * 3 + c].
  std::array<Scalar, 9> d{};

  /// The zero matrix, not the identity. Matching `Vec3{}` and C++'s usual
  /// value-initialisation; use identity() where that is what is meant.
  constexpr Mat3() noexcept = default;

  /// The 3x3 identity.
  static constexpr Mat3 identity() noexcept {
    Mat3 m;
    m.d = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    return m;
  }
  /// The zero matrix.
  static constexpr Mat3 zero() noexcept { return {}; }

  /// Column-wise construction -- the natural form for a rotation matrix built
  /// from the images of the basis vectors.
  static constexpr Mat3 fromColumns(const Vec3& c0, const Vec3& c1,
                                    const Vec3& c2) noexcept {
    Mat3 m;
    m.d = {c0.x, c1.x, c2.x, c0.y, c1.y, c2.y, c0.z, c1.z, c2.z};
    return m;
  }

  /// Mutable element access by row and column, both zero-based.
  ///
  /// Indices are *not* checked; passing anything above 2 is undefined
  /// behaviour, as it is for the underlying std::array. See Vec3::operator[]
  /// for why that operator checks and this one does not.
  constexpr Scalar& operator()(std::size_t r, std::size_t c) { return d[r * 3 + c]; }
  /// Element access by row and column, both zero-based and unchecked.
  constexpr Scalar operator()(std::size_t r, std::size_t c) const { return d[r * 3 + c]; }

  /// Column `c` as a vector. Unchecked, as operator() is.
  [[nodiscard]] constexpr Vec3 column(std::size_t c) const {
    return {(*this)(0, c), (*this)(1, c), (*this)(2, c)};
  }
  /// Row `r` as a vector. Unchecked, as operator() is.
  [[nodiscard]] constexpr Vec3 row(std::size_t r) const {
    return {(*this)(r, 0), (*this)(r, 1), (*this)(r, 2)};
  }

  /// The transpose. For a matrix that is a rotation, this is also its inverse.
  [[nodiscard]] constexpr Mat3 transpose() const noexcept {
    Mat3 t;
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        t(r, c) = (*this)(c, r);
      }
    }
    return t;
  }

  /// Matrix product, applying `o` first and then `*this`.
  constexpr Mat3 operator*(const Mat3& o) const noexcept {
    Mat3 p;
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        Scalar acc = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
          acc += (*this)(r, k) * o(k, c);
        }
        p(r, c) = acc;
      }
    }
    return p;
  }

  /// Applies the matrix to a column vector.
  constexpr Vec3 operator*(const Vec3& v) const noexcept {
    return {d[0] * v.x + d[1] * v.y + d[2] * v.z, d[3] * v.x + d[4] * v.y + d[5] * v.z,
            d[6] * v.x + d[7] * v.y + d[8] * v.z};
  }

  /// The determinant. For a rotation it is +1; a value near -1 means the
  /// matrix is a reflection, which is the usual sign of a mirrored CAD export
  /// or a left-handed convention on the other side of an interface.
  [[nodiscard]] constexpr Scalar determinant() const noexcept {
    return d[0] * (d[4] * d[8] - d[5] * d[7]) - d[1] * (d[3] * d[8] - d[5] * d[6]) +
           d[2] * (d[3] * d[7] - d[4] * d[6]);
  }

  /// The sum of the diagonal. For a rotation, `1 + 2 * cos(angle)`.
  [[nodiscard]] constexpr Scalar trace() const noexcept { return d[0] + d[4] + d[8]; }

  /// True when the columns are orthonormal and right-handed, i.e. the matrix is
  /// a member of SO(3) to within `tol`.
  [[nodiscard]] bool isRotation(Scalar tol = kOrthoTolerance) const noexcept;

  /// True when every element is within `tol` of the matching element of `o`.
  [[nodiscard]] bool isApprox(const Mat3& o, Scalar tol) const noexcept;
};

}  // namespace motionkit
