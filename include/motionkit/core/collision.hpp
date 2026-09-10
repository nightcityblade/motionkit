// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// Obstacles a single query may be given.
inline constexpr std::size_t kMaxObstacles = 32;

/// Reported when a query has nothing at all to measure against.
///
/// A large finite number rather than infinity. Infinity is the honest answer
/// and a terrible one to hand back, because it survives arithmetic that should
/// have failed and turns up much later as a NaN.
inline constexpr Scalar kNothingNear = 1e30;

/// Why a clearance query could not answer.
enum class CollisionError : std::uint8_t {
  None = 0,
  /// A shape count that does not match the chain, or too many obstacles.
  SizeMismatch,
  /// A NaN or infinite input.
  NonFiniteInput,
  /// A negative radius. A capsule of negative thickness is not a thinner
  /// capsule; it is nothing, and it would make every distance look larger.
  NegativeRadius,
};

/// A human-readable name for a CollisionError.
std::string_view toString(CollisionError error) noexcept;

/// A line segment thickened by a radius: the standard stand-in for a robot
/// link, a rail, a fence post or a cable.
///
/// A capsule is chosen over a box or a mesh because the distance between two of
/// them is the distance between two segments minus their radii -- one function,
/// no special cases, and no orientation to keep track of. A link that is not
/// very capsule-shaped is covered by two or three of them, which is still
/// cheaper than the mesh.
///
/// Like joints and inertias, a link's capsule is given **in the base frame with
/// every joint at zero**, so it can be read off the same CAD model in the same
/// pose as everything else. See ADR-0008 for why this library keeps asking for
/// that frame and no other.
struct Capsule {
  /// One end of the axis.
  Vec3 from;
  /// The other end. May equal `from`, which makes the capsule a sphere.
  Vec3 to;
  /// Thickness around the axis, in metres. Must not be negative.
  Scalar radius{0.0};
};

/// The closest approach found, and between which pair.
struct Clearance {
  /// Distance between the two nearest surfaces, in metres.
  ///
  /// **Negative when they overlap**, by the depth of the overlap. Signed rather
  /// than clamped at zero because "touching" and "driven 40 mm through" want
  /// different responses, and a caller that only wants a yes or no can compare
  /// against zero.
  Scalar distance{0.0};
  /// Index of the link involved.
  std::size_t link{0};
  /// Index of the obstacle involved, or the second link when `self` is true.
  std::size_t other{0};
  /// True when the closest pair was two links of the arm rather than a link and
  /// an obstacle.
  bool self{false};
  /// The closest link-to-obstacle approach, ignoring self-collision entirely.
  ///
  /// Reported separately because the two answer different questions and one
  /// routinely hides the other. A straight arm's own geometry holds its links
  /// a fixed distance apart -- 0.22 m on the example -- so a global minimum
  /// alone reports that number no matter where an obstacle is, until the
  /// obstacle is nearer than the arm is to itself. An obstacle 30 cm away is
  /// then invisible, which is not what anybody asking about obstacles wants.
  ///
  /// `kNothingNear` when there are no obstacles.
  Scalar to_obstacles{0.0};
  /// The closest link-to-link approach, ignoring obstacles entirely.
  ///
  /// `kNothingNear` when every pair is adjacent or excluded.
  Scalar to_self{0.0};
};

/// The distance between two capsules, negative when they overlap.
///
/// Exposed because it is the whole of the geometry here and is worth being able
/// to test and use on its own -- a fixture against a fence needs the same
/// answer as a link against an obstacle.
[[nodiscard]] Scalar distanceBetween(const Capsule& a, const Capsule& b) noexcept;

/// A serial chain wearing collision shapes, and the clearances that follow.
///
/// One capsule per link, in the same order as the joints. Link `i` is the body
/// that joint `i` moves, exactly as in DynamicChain -- the fixed base is not a
/// link here either, so a base that can be hit needs to be given as an
/// obstacle rather than as a shape.
///
/// **Adjacent links are never checked against each other.** They share a joint,
/// so their capsules touch by construction at every configuration, and a
/// self-collision check that includes them reports a collision always.
///
/// That rule alone is not enough, and assuming it is produces a model that
/// cries collision while standing still. A spherical wrist turns its last three
/// links about nearly one point, so links two apart overlap there at *every*
/// configuration too. Which further pairs to ignore is a property of the
/// machine rather than of the algorithm, so build() takes them -- the same
/// allowed-collision set a real cell keeps beside its geometry.
///
/// **Threading**: const methods are pure and reentrant, and clearance() neither
/// allocates nor throws.
class CollisionModel {
 public:
  /// An empty model with no links.
  constexpr CollisionModel() noexcept = default;

  /// Builds from a chain, one capsule per joint, and the pairs to ignore.
  ///
  /// `ignored` holds link index pairs that must never be reported against each
  /// other, in either order. Adjacent pairs are ignored whether they appear
  /// here or not. Out-of-range indices are rejected rather than dropped: an
  /// exclusion aimed at a link that does not exist is a model that thinks it is
  /// safer than it is.
  static Expected<CollisionModel, CollisionError> build(
      const SerialChain& chain, std::span<const Capsule> shapes,
      std::span<const std::array<std::size_t, 2>> ignored = {});

  /// The chain these shapes hang on.
  [[nodiscard]] const SerialChain& chain() const noexcept { return chain_; }

  /// The number of links, and so of shapes.
  [[nodiscard]] constexpr std::size_t linkCount() const noexcept { return count_; }

  /// The shape of link `index`, at the zero configuration. Unchecked.
  [[nodiscard]] const Capsule& shape(std::size_t index) const noexcept {
    return shapes_[index];
  }

  /// The closest approach at configuration `q`, against `obstacles`.
  ///
  /// Considers every link against every obstacle, and every pair of links more
  /// than one apart. Obstacles are in the base frame and do not move with the
  /// arm. Pass an empty span to ask only about self-collision.
  ///
  /// Returns the single closest pair rather than a list, and alongside it the
  /// closest of each kind. A caller deciding whether to move needs the worst
  /// number, one that wants to know why gets the pair that produced it, and one
  /// asking specifically about the world gets `to_obstacles`.
  [[nodiscard]] Expected<Clearance, CollisionError> clearance(
      std::span<const Scalar> q, std::span<const Capsule> obstacles) const noexcept;

  /// Capsules roughly wrapping the six-axis example arm, with the wrist pairs
  /// that always overlap already excluded.
  ///
  /// Deliberately generous: a real model comes from CAD, and one invented in a
  /// header should err towards reporting a collision that is not there rather
  /// than missing one that is.
  static Expected<CollisionModel, CollisionError> sixAxisExample();

 private:
  [[nodiscard]] bool checks(std::size_t a, std::size_t b) const noexcept;

  SerialChain chain_;
  std::array<Capsule, kMaxJoints> shapes_{};
  /// One bit per ordered link pair. kMaxJoints is 8, so 64 bits is the whole
  /// matrix and the model stays trivially copyable.
  std::uint64_t ignored_{0};
  std::size_t count_{0};
};

}  // namespace motionkit
