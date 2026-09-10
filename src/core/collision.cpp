// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/collision.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

/// Squared distance between two segments, and nothing else.
///
/// The one piece of geometry in this file. Two capsules are two segments and
/// two radii, so every query here reduces to this and a subtraction -- which is
/// the whole argument for capsules over boxes or meshes.
///
/// Follows the standard clamped-parameter construction: solve for the closest
/// points on the infinite lines, clamp each to its segment, and re-solve the
/// other against the clamped one. The degenerate branches are not decoration.
/// A zero-length segment is a sphere, which is a shape callers pass on purpose,
/// and parallel segments make the shared denominator vanish.
Scalar squaredDistanceBetweenSegments(const Vec3& p0, const Vec3& p1, const Vec3& q0,
                                      const Vec3& q1) noexcept {
  constexpr Scalar kTiny = 1e-18;
  const Vec3 u = p1 - p0;
  const Vec3 v = q1 - q0;
  const Vec3 w = p0 - q0;
  const Scalar a = u.dot(u);
  const Scalar e = v.dot(v);
  const Scalar f = v.dot(w);

  Scalar s = 0.0;
  Scalar t = 0.0;
  if (a <= kTiny && e <= kTiny) {
    return w.squaredNorm();
  }
  if (a <= kTiny) {
    t = std::clamp(f / e, Scalar{0.0}, Scalar{1.0});
  } else {
    const Scalar c = u.dot(w);
    if (e <= kTiny) {
      s = std::clamp(-c / a, Scalar{0.0}, Scalar{1.0});
    } else {
      const Scalar b = u.dot(v);
      const Scalar denominator = (a * e) - (b * b);
      // Zero exactly when the segments are parallel, and then any point will
      // do for one of them; the clamping below finds the other.
      s = denominator > kTiny
              ? std::clamp(((b * f) - (c * e)) / denominator, Scalar{0.0}, Scalar{1.0})
              : 0.0;
      t = ((b * s) + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, Scalar{0.0}, Scalar{1.0});
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, Scalar{0.0}, Scalar{1.0});
      }
    }
  }
  return (w + (u * s) - (v * t)).squaredNorm();
}

bool capsuleIsFinite(const Capsule& shape) noexcept {
  return std::isfinite(shape.from.squaredNorm()) &&
         std::isfinite(shape.to.squaredNorm()) && std::isfinite(shape.radius);
}

Capsule carriedBy(const SE3& pose, const Capsule& shape) noexcept {
  return Capsule{pose * shape.from, pose * shape.to, shape.radius};
}

/// Keeps `closest` pointing at the nearest pair seen so far.
void consider(Clearance& closest, Scalar gap, std::size_t a, std::size_t b,
              bool self) noexcept {
  if (gap >= closest.distance) {
    return;
  }
  closest.distance = gap;
  closest.link = a;
  closest.other = b;
  closest.self = self;
}

CollisionError checkObstacles(std::span<const Capsule> obstacles) noexcept {
  if (obstacles.size() > kMaxObstacles) {
    return CollisionError::SizeMismatch;
  }
  for (const Capsule& obstacle : obstacles) {
    if (!capsuleIsFinite(obstacle)) {
      return CollisionError::NonFiniteInput;
    }
    if (obstacle.radius < 0.0) {
      return CollisionError::NegativeRadius;
    }
  }
  return CollisionError::None;
}

}  // namespace

std::string_view toString(CollisionError error) noexcept {
  switch (error) {
    case CollisionError::None:
      return "None";
    case CollisionError::SizeMismatch:
      return "SizeMismatch";
    case CollisionError::NonFiniteInput:
      return "NonFiniteInput";
    case CollisionError::NegativeRadius:
      return "NegativeRadius";
  }
  return "Unknown";
}

Scalar distanceBetween(const Capsule& a, const Capsule& b) noexcept {
  const Scalar between =
      std::sqrt(squaredDistanceBetweenSegments(a.from, a.to, b.from, b.to));
  return between - a.radius - b.radius;
}

bool CollisionModel::checks(std::size_t a, std::size_t b) const noexcept {
  // Neighbours share a joint and touch always, so they are never a finding.
  if (a == b || a + 1 == b || b + 1 == a) {
    return false;
  }
  return (ignored_ & (std::uint64_t{1} << ((a * kMaxJoints) + b))) == 0;
}

Expected<CollisionModel, CollisionError> CollisionModel::build(
    const SerialChain& chain, std::span<const Capsule> shapes,
    std::span<const std::array<std::size_t, 2>> ignored) {
  const std::size_t count = chain.jointCount();
  if (count == 0 || shapes.size() != count) {
    return {CollisionModel{}, CollisionError::SizeMismatch};
  }
  CollisionModel built;
  built.chain_ = chain;
  built.count_ = count;
  for (std::size_t i = 0; i < count; ++i) {
    if (!capsuleIsFinite(shapes[i])) {
      return {CollisionModel{}, CollisionError::NonFiniteInput};
    }
    if (shapes[i].radius < 0.0) {
      return {CollisionModel{}, CollisionError::NegativeRadius};
    }
    built.shapes_[i] = shapes[i];
  }
  for (const std::array<std::size_t, 2>& pair : ignored) {
    // An exclusion aimed at a link that does not exist is a model that believes
    // it is safer than it is, so it is refused rather than dropped.
    if (pair[0] >= count || pair[1] >= count) {
      return {CollisionModel{}, CollisionError::SizeMismatch};
    }
    built.ignored_ |= std::uint64_t{1} << ((pair[0] * kMaxJoints) + pair[1]);
    built.ignored_ |= std::uint64_t{1} << ((pair[1] * kMaxJoints) + pair[0]);
  }
  return {built, CollisionError::None};
}

Expected<Clearance, CollisionError> CollisionModel::clearance(
    std::span<const Scalar> q, std::span<const Capsule> obstacles) const noexcept {
  if (q.size() != count_) {
    return {Clearance{}, CollisionError::SizeMismatch};
  }
  if (const CollisionError error = checkObstacles(obstacles);
      error != CollisionError::None) {
    return {Clearance{}, error};
  }

  std::array<SE3, kMaxJoints> carried{};
  if (chain_.linkTransforms(q, std::span<SE3>{carried.data(), count_}) !=
      KinematicsError::None) {
    return {Clearance{}, CollisionError::NonFiniteInput};
  }

  std::array<Capsule, kMaxJoints> placed{};
  for (std::size_t i = 0; i < count_; ++i) {
    placed[i] = carriedBy(carried[i], shapes_[i]);
  }

  Clearance closest;
  closest.distance = kNothingNear;
  closest.to_obstacles = kNothingNear;
  closest.to_self = kNothingNear;
  for (std::size_t i = 0; i < count_; ++i) {
    for (std::size_t o = 0; o < obstacles.size(); ++o) {
      const Scalar gap = distanceBetween(placed[i], obstacles[o]);
      closest.to_obstacles = std::min(closest.to_obstacles, gap);
      consider(closest, gap, i, o, false);
    }
    for (std::size_t j = i + 1; j < count_; ++j) {
      if (!checks(i, j)) {
        continue;
      }
      const Scalar gap = distanceBetween(placed[i], placed[j]);
      closest.to_self = std::min(closest.to_self, gap);
      consider(closest, gap, i, j, true);
    }
  }
  return {closest, CollisionError::None};
}

Expected<CollisionModel, CollisionError> CollisionModel::sixAxisExample() {
  // Sleeves around the arm as it stands at zero: a stout base column, the upper
  // arm, the forearm, and three short wrist bodies. Radii are on the generous
  // side of the real thing on purpose -- a model invented in a header should
  // stop a move that would have been fine rather than allow one that would not.
  const std::array<Capsule, 6> shapes{{
      {Vec3{0.0, 0.0, 0.00}, Vec3{0.0, 0.0, 0.15}, 0.090},
      {Vec3{0.0, 0.0, 0.15}, Vec3{0.0, 0.0, 0.58}, 0.070},
      {Vec3{0.0, 0.0, 0.58}, Vec3{0.0, 0.0, 0.97}, 0.060},
      {Vec3{0.0, 0.0, 0.92}, Vec3{0.0, 0.0, 0.97}, 0.050},
      {Vec3{0.0, 0.0, 0.97}, Vec3{0.0, 0.0, 1.00}, 0.045},
      {Vec3{0.0, 0.0, 1.00}, Vec3{0.0, 0.0, 1.07}, 0.040},
  }};
  // The three wrist axes meet within a few centimetres, so these pairs overlap
  // at every configuration the arm can reach and mean nothing. Leaving them in
  // would make the model report a collision while the arm stood still, which is
  // the failure that teaches people to switch collision checking off.
  const std::array<std::array<std::size_t, 2>, 3> ignored{{{2, 4}, {2, 5}, {3, 5}}};
  return build(SerialChain::sixAxisExample(), shapes, ignored);
}

}  // namespace motionkit
