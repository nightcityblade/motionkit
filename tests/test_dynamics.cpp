// SPDX-License-Identifier: Apache-2.0
//
// Rigid-body dynamics: recursive Newton-Euler, and the composite-rigid-body
// mass matrix.
//
// Dynamics is unusually well supplied with independent checks, and this file
// leans on them rather than on tables of expected torques. Three matter:
//
//   * The mass matrix can be computed twice by unrelated routes -- once by
//     CRBA, once by asking RNEA for the torque of a unit acceleration on each
//     joint in turn. Agreement is evidence; a hand-written expected matrix
//     would only be evidence that two people multiplied the same way.
//   * The gravity torque is the gradient of the potential energy, and the
//     potential energy is a one-line sum over link heights that shares no code
//     with the recursion. Numerical differentiation of one must reproduce the
//     other.
//   * A one-link pendulum has a closed form, so at least one case is pinned to
//     arithmetic somebody can do on paper.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include "motionkit/core/dynamics.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

using Vec = std::vector<Scalar>;

/// A few configurations that between them exercise the interesting shapes:
/// the singular zero pose, a folded one, a stretched one, and one with no
/// symmetry left to hide behind.
const std::array<Vec, 4>& poses() {
  static const std::array<Vec, 4> kPoses{{
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.3, -0.7, 1.1, 0.4, -0.6, 0.2},
      {-1.2, 0.9, -1.4, 2.0, 1.1, -0.8},
      {2.5, 1.3, -2.2, -1.7, 0.5, 3.0},
  }};
  return kPoses;
}

/// A single link on one revolute joint: a pendulum, but inverted at q = 0.
/// Point-like, so the closed form is exact rather than approximate.
DynamicChain pendulum(Scalar mass, Scalar length) {
  const std::array<RevoluteJoint, 1> joints{{
      {Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 0.0}, -10.0, 10.0},
  }};
  const SerialChain chain =
      SerialChain::build(joints, SE3::fromTranslation(Vec3{0.0, 0.0, length})).value;
  Mat3 tiny = Mat3::zero();
  // Not zero: a point mass has no inertia about its own centre, but the
  // validator refuses a singular tensor, and rightly -- it cannot tell a
  // deliberate idealisation from a forgotten field.
  tiny(0, 0) = 1e-9;
  tiny(1, 1) = 1e-9;
  tiny(2, 2) = 1e-9;
  const std::array<RigidBody, 1> bodies{{{mass, Vec3{0.0, 0.0, length}, tiny}}};
  return DynamicChain::build(chain, bodies).value;
}

}  // namespace

// --- the two independent cross-checks ---------------------------------------

TEST(DynamicsMassMatrix, CrbaAgreesWithUnitAccelerationsThroughRecursiveNewtonEuler) {
  DynamicChain arm = DynamicChain::sixAxisExample();
  // Gravity off: the mass matrix is the coefficient of acceleration, and a
  // weight term would appear in the torques but not in the matrix.
  arm.setGravity(Vec3{});
  const std::size_t n = arm.jointCount();

  Scalar worst = 0.0;
  for (const Vec& q : poses()) {
    std::vector<Scalar> mass(n * n, 0.0);
    ASSERT_EQ(arm.massMatrix(q, mass), DynamicsError::None);

    for (std::size_t j = 0; j < n; ++j) {
      Vec qd(n, 0.0);
      Vec qdd(n, 0.0);
      qdd[j] = 1.0;
      Vec tau(n, 0.0);
      ASSERT_EQ(arm.inverseDynamics(q, qd, qdd, tau), DynamicsError::None);

      for (std::size_t i = 0; i < n; ++i) {
        worst = std::max(worst, std::abs(tau[i] - mass[(i * n) + j]));
      }
    }
  }
  std::printf("CRBA vs RNEA, worst element: %.3e kg m^2\n", worst);
  EXPECT_LT(worst, 1e-12);
}

TEST(DynamicsGravity, TorqueIsTheGradientOfThePotentialEnergy) {
  const DynamicChain arm = DynamicChain::sixAxisExample();
  const std::size_t n = arm.jointCount();
  constexpr Scalar kStep = 1e-6;

  Scalar worst = 0.0;
  for (const Vec& q : poses()) {
    Vec tau(n, 0.0);
    ASSERT_EQ(arm.gravityTorque(q, tau), DynamicsError::None);

    for (std::size_t j = 0; j < n; ++j) {
      Vec ahead = q;
      Vec behind = q;
      ahead[j] += kStep;
      behind[j] -= kStep;
      const auto up = arm.potentialEnergy(ahead);
      const auto down = arm.potentialEnergy(behind);
      ASSERT_TRUE(up) << toString(up.error);
      ASSERT_TRUE(down);
      const Scalar slope = (up.value - down.value) / (2.0 * kStep);
      worst = std::max(worst, std::abs(slope - tau[j]));
    }
  }
  std::printf("gravity torque vs dU/dq, worst: %.3e N m\n", worst);
  // Central differences on a 1e-6 step, so about 1e-9 is the floor here.
  EXPECT_LT(worst, 1e-7);
}

TEST(DynamicsPendulum, MatchesTheClosedFormOfAnInvertedPendulum) {
  constexpr Scalar kMass = 3.0;
  constexpr Scalar kLength = 0.75;
  const DynamicChain arm = pendulum(kMass, kLength);

  // Rotating about +Y carries +Z toward +X, so at angle t the mass sits at
  // (L sin t, 0, L cos t) and gravity exerts +m g L sin t about +Y. Holding it
  // still therefore needs the negative of that.
  for (const Scalar angle : {0.0, 0.3, 1.0, -0.7, std::numbers::pi_v<Scalar> / 2.0}) {
    const Vec q{angle};
    Vec tau(1, 0.0);
    ASSERT_EQ(arm.gravityTorque(q, tau), DynamicsError::None);
    const Scalar expected = -kMass * kStandardGravity * kLength * std::sin(angle);
    EXPECT_NEAR(tau[0], expected, 1e-9) << "at angle " << angle;
  }
}

TEST(DynamicsPendulum, BalancedExactlyUprightAndExactlyHangingItNeedsNoTorque) {
  const DynamicChain arm = pendulum(3.0, 0.75);
  Vec tau(1, 0.0);
  ASSERT_EQ(arm.gravityTorque(Vec{0.0}, tau), DynamicsError::None);
  EXPECT_NEAR(tau[0], 0.0, 1e-12);
  ASSERT_EQ(arm.gravityTorque(Vec{std::numbers::pi_v<Scalar>}, tau), DynamicsError::None);
  // An equilibrium either way: one stable, one not, and the torque cannot tell
  // them apart. Which is the honest limit of a statics calculation.
  EXPECT_NEAR(tau[0], 0.0, 1e-12);
}

// --- properties the mass matrix must have -----------------------------------

TEST(DynamicsMassMatrix, IsSymmetricAtEveryConfiguration) {
  const DynamicChain arm = DynamicChain::sixAxisExample();
  const std::size_t n = arm.jointCount();
  for (const Vec& q : poses()) {
    std::vector<Scalar> mass(n * n, 0.0);
    ASSERT_EQ(arm.massMatrix(q, mass), DynamicsError::None);
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < n; ++j) {
        EXPECT_NEAR(mass[(i * n) + j], mass[(j * n) + i], 1e-15) << i << "," << j;
      }
    }
  }
}

TEST(DynamicsMassMatrix, IsPositiveDefiniteEvenWhereTheJacobianIsSingular) {
  const DynamicChain arm = DynamicChain::sixAxisExample();
  // The zero configuration of this arm is a kinematic singularity -- joints 1,
  // 4 and 6 are collinear and the Jacobian loses rank there. The mass matrix
  // does not care: an arm at full stretch is still hard to accelerate about
  // every joint, so the kinetic energy of any non-zero motion stays positive.
  const auto singular = arm.chain().manipulability(poses()[0]);
  ASSERT_TRUE(singular);
  EXPECT_LT(singular.value, 1e-6) << "the zero pose was supposed to be singular";

  const std::array<Vec, 3> motions{{
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
      {0.4, -1.3, 0.8, -0.2, 1.7, -0.9},
  }};
  for (const Vec& q : poses()) {
    for (const Vec& qd : motions) {
      const auto energy = arm.kineticEnergy(q, qd);
      ASSERT_TRUE(energy) << toString(energy.error);
      EXPECT_GT(energy.value, 0.0);
    }
  }
}

TEST(DynamicsMassMatrix, AStationaryArmHasNoKineticEnergy) {
  const DynamicChain arm = DynamicChain::sixAxisExample();
  const Vec still(arm.jointCount(), 0.0);
  const auto energy = arm.kineticEnergy(poses()[1], still);
  ASSERT_TRUE(energy);
  EXPECT_NEAR(energy.value, 0.0, 1e-15);
}

TEST(DynamicsInverse, WithoutGravityAStationaryArmNeedsNoTorque) {
  DynamicChain arm = DynamicChain::sixAxisExample();
  arm.setGravity(Vec3{});
  const std::size_t n = arm.jointCount();
  const Vec zero(n, 0.0);
  Vec tau(n, 0.0);
  for (const Vec& q : poses()) {
    ASSERT_EQ(arm.inverseDynamics(q, zero, zero, tau), DynamicsError::None);
    for (const Scalar t : tau) {
      EXPECT_NEAR(t, 0.0, 1e-14);
    }
  }
}

TEST(DynamicsGravity, AnArmHungFromTheCeilingNeedsTheOppositeTorque) {
  DynamicChain upright = DynamicChain::sixAxisExample();
  DynamicChain inverted = DynamicChain::sixAxisExample();
  inverted.setGravity(-upright.gravity());
  const std::size_t n = upright.jointCount();

  // Gravity enters linearly, so reversing the field reverses every torque.
  // Worth pinning because mounting an arm on a ceiling or a wall is normally
  // discovered on site, and a library that silently assumed a floor would be
  // wrong in a way nobody tests for.
  Vec a(n, 0.0);
  Vec b(n, 0.0);
  for (const Vec& q : poses()) {
    ASSERT_EQ(upright.gravityTorque(q, a), DynamicsError::None);
    ASSERT_EQ(inverted.gravityTorque(q, b), DynamicsError::None);
    for (std::size_t i = 0; i < n; ++i) {
      EXPECT_NEAR(a[i], -b[i], 1e-12) << "joint " << i;
    }
  }
}

// --- what build() refuses ----------------------------------------------------

TEST(DynamicsValidation, RefusesAMasslessLink) {
  const SerialChain chain = SerialChain::sixAxisExample();
  std::array<RigidBody, 6> bodies{};
  for (RigidBody& body : bodies) {
    body = RigidBody{1.0, Vec3{0.0, 0.0, 0.1}, Mat3::identity()};
  }
  bodies[3].mass = 0.0;
  EXPECT_EQ(DynamicChain::build(chain, bodies).error, DynamicsError::NonPositiveMass);
  bodies[3].mass = -2.0;
  EXPECT_EQ(DynamicChain::build(chain, bodies).error, DynamicsError::NonPositiveMass);
}

TEST(DynamicsValidation, RefusesAnAsymmetricInertiaTensor) {
  const SerialChain chain = SerialChain::sixAxisExample();
  std::array<RigidBody, 6> bodies{};
  for (RigidBody& body : bodies) {
    body = RigidBody{1.0, Vec3{0.0, 0.0, 0.1}, Mat3::identity()};
  }
  // A transposed or half-filled tensor is the usual way this happens: someone
  // fills the upper triangle and forgets the lower.
  bodies[2].inertia(0, 1) = 0.3;
  EXPECT_EQ(DynamicChain::build(chain, bodies).error, DynamicsError::ImplausibleInertia);
}

TEST(DynamicsValidation, RefusesAnInertiaThatBreaksTheTriangleInequality) {
  const SerialChain chain = SerialChain::sixAxisExample();
  std::array<RigidBody, 6> bodies{};
  for (RigidBody& body : bodies) {
    body = RigidBody{1.0, Vec3{0.0, 0.0, 0.1}, Mat3::identity()};
  }
  // Symmetric and positive definite, and still impossible: no distribution of
  // mass makes one axis harder to spin than the other two together. This is
  // the case the other two checks pass and only the triangle inequality
  // catches -- which is the whole argument for computing eigenvalues.
  Mat3 impossible = Mat3::zero();
  impossible(0, 0) = 0.1;
  impossible(1, 1) = 0.1;
  impossible(2, 2) = 0.5;
  bodies[1].inertia = impossible;
  EXPECT_EQ(DynamicChain::build(chain, bodies).error, DynamicsError::ImplausibleInertia);

  // The same tensor with the third moment brought inside the bound is fine.
  impossible(2, 2) = 0.2;
  bodies[1].inertia = impossible;
  EXPECT_EQ(DynamicChain::build(chain, bodies).error, DynamicsError::None);
}

TEST(DynamicsValidation, RefusesAMismatchedBodyCount) {
  const SerialChain chain = SerialChain::sixAxisExample();
  const std::array<RigidBody, 3> too_few{};
  EXPECT_EQ(DynamicChain::build(chain, too_few).error, DynamicsError::SizeMismatch);
}

TEST(DynamicsValidation, RejectsNonFiniteMotionRatherThanPropagatingNaN) {
  const DynamicChain arm = DynamicChain::sixAxisExample();
  ASSERT_EQ(arm.jointCount(), 6u);
  // Fixed-size storage rather than a vector: at -O3 GCC cannot prove a
  // vector's buffer is non-null from a runtime joint count, and raises
  // -Wnull-dereference on the subscript below. The array carries its size in
  // the type, so the question does not arise.
  const std::array<Scalar, 6> zero{};
  std::array<Scalar, 6> tau{};
  std::array<Scalar, 6> bad{};
  bad[2] = std::numeric_limits<Scalar>::quiet_NaN();
  EXPECT_EQ(arm.inverseDynamics(bad, zero, zero, tau), DynamicsError::NonFiniteInput);
  EXPECT_EQ(arm.inverseDynamics(zero, bad, zero, tau), DynamicsError::NonFiniteInput);
  EXPECT_EQ(arm.inverseDynamics(zero, zero, bad, tau), DynamicsError::NonFiniteInput);
  bad[2] = std::numeric_limits<Scalar>::infinity();
  EXPECT_EQ(arm.massMatrix(bad, tau), DynamicsError::SizeMismatch)
      << "a buffer too small should be reported before the contents are read";
}

TEST(DynamicsValidation, ReportsATooSmallOutputBuffer) {
  const DynamicChain arm = DynamicChain::sixAxisExample();
  const std::size_t n = arm.jointCount();
  const Vec q(n, 0.0);
  Vec cramped(n - 1, 0.0);
  EXPECT_EQ(arm.gravityTorque(q, cramped), DynamicsError::SizeMismatch);
  std::vector<Scalar> small_matrix((n * n) - 1, 0.0);
  EXPECT_EQ(arm.massMatrix(q, small_matrix), DynamicsError::SizeMismatch);
}

TEST(DynamicsErrors, EveryEnumeratorHasAName) {
  EXPECT_EQ(toString(DynamicsError::None), "None");
  EXPECT_EQ(toString(DynamicsError::SizeMismatch), "SizeMismatch");
  EXPECT_EQ(toString(DynamicsError::NonFiniteInput), "NonFiniteInput");
  EXPECT_EQ(toString(DynamicsError::NonPositiveMass), "NonPositiveMass");
  EXPECT_EQ(toString(DynamicsError::ImplausibleInertia), "ImplausibleInertia");
}

}  // namespace motionkit
