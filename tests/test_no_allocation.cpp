// SPDX-License-Identifier: Apache-2.0
//
// Proves that the hot paths of this library do not allocate, by replacing every
// global new/delete form in this executable and counting. This must remain a
// separate test binary: ThreadSanitizer provides its own replacements for the
// same symbols, and two sets cannot both win.

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

#if defined(_MSC_VER)
#include <malloc.h>
#endif

#include "motionkit/core/calibration.hpp"
#include "motionkit/core/cartesian.hpp"
#include "motionkit/core/collision.hpp"
#include "motionkit/core/dynamics.hpp"
#include "motionkit/core/frame_graph.hpp"
#include "motionkit/core/kinematics.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/so3.hpp"
#include "motionkit/core/trajectory.hpp"
#include "motionkit/core/types.hpp"

namespace {

/// Counts every trip through global operator new in this binary.
///
/// Global replacement is heavy-handed, but it is the only way to prove a
/// negative about allocation: a test that merely calls lookup() and passes
/// proves nothing, because an allocation would not fail anything.
std::atomic<std::size_t> g_allocation_count{0};

void* countedAllocate(std::size_t size) {
  g_allocation_count.fetch_add(1, std::memory_order_relaxed);
  return std::malloc(size == 0 ? 1 : size);
}

void* countedAllocateAligned(std::size_t size, std::size_t alignment) {
  g_allocation_count.fetch_add(1, std::memory_order_relaxed);
  // aligned_alloc requires a size that is a multiple of the alignment.
  const std::size_t rounded =
      ((size == 0 ? 1 : size) + alignment - 1) / alignment * alignment;
#if defined(_MSC_VER)
  return _aligned_malloc(rounded, alignment);
#else
  return std::aligned_alloc(alignment, rounded);
#endif
}

void countedDeallocateAligned(void* p) noexcept {
#if defined(_MSC_VER)
  _aligned_free(p);
#else
  std::free(p);
#endif
}

}  // namespace

// Every variant, not just the common one.
//
// Replacing only `operator new(size_t)` and `operator delete(void*)` leaves the
// nothrow and sized forms pointing at the runtime's implementation, and then
// memory obtained one way is released the other. Under AddressSanitizer that is
// caught immediately -- `alloc-dealloc-mismatch (operator new vs free)`, raised
// from inside GoogleTest's own stable_sort, which uses a temporary buffer. It
// is a real bug either way; the sanitizer just makes it loud.
//
// So: all of them route through a matching malloc-family pair, and the pairing
// stays consistent no matter which form the standard library reaches for.

void* operator new(std::size_t size) {
  void* p = countedAllocate(size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new[](std::size_t size) {
  void* p = countedAllocate(size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return countedAllocate(size);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  return countedAllocate(size);
}

void* operator new(std::size_t size, std::align_val_t alignment) {
  void* p = countedAllocateAligned(size, static_cast<std::size_t>(alignment));
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new[](std::size_t size, std::align_val_t alignment) {
  void* p = countedAllocateAligned(size, static_cast<std::size_t>(alignment));
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t&) noexcept {
  return countedAllocateAligned(size, static_cast<std::size_t>(alignment));
}

void* operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t&) noexcept {
  return countedAllocateAligned(size, static_cast<std::size_t>(alignment));
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { countedDeallocateAligned(p); }
void operator delete[](void* p, std::align_val_t) noexcept {
  countedDeallocateAligned(p);
}
void operator delete(void* p, std::size_t, std::align_val_t) noexcept {
  countedDeallocateAligned(p);
}
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept {
  countedDeallocateAligned(p);
}
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  countedDeallocateAligned(p);
}
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  countedDeallocateAligned(p);
}

namespace motionkit {
namespace {

/// Allocations performed while running `body`.
template <typename F>
std::size_t allocationsDuring(F&& body) {
  const std::size_t before = g_allocation_count.load(std::memory_order_relaxed);
  body();
  return g_allocation_count.load(std::memory_order_relaxed) - before;
}

struct DeepTree {
  FrameGraph graph;
  std::vector<FrameId> left;  ///< left[i] is at depth i+1 under the shared spine
  std::vector<FrameId> right;
};

/// A spine of `spine_depth` frames, then two branches of `branch_depth` each.
DeepTree buildDeepTree(std::uint32_t spine_depth, std::uint32_t branch_depth) {
  DeepTree tree;
  FrameId current = tree.graph.declareRoot("root").value;

  for (std::uint32_t i = 0; i < spine_depth; ++i) {
    const SE3 t(SO3::fromRPY(0.11, -0.07, 0.23), Vec3{0.13, -0.05, 0.21});
    current = tree.graph.declareFrame("spine" + std::to_string(i), current, t).value;
  }

  FrameId left = current;
  FrameId right = current;
  for (std::uint32_t i = 0; i < branch_depth; ++i) {
    const SE3 lt(SO3::fromRPY(0.31, 0.17, -0.09), Vec3{0.07, 0.02, -0.03});
    const SE3 rt(SO3::fromRPY(-0.19, 0.29, 0.13), Vec3{-0.04, 0.09, 0.06});
    left = tree.graph.declareFrame("left" + std::to_string(i), left, lt).value;
    right = tree.graph.declareFrame("right" + std::to_string(i), right, rt).value;
    tree.left.push_back(left);
    tree.right.push_back(right);
  }
  return tree;
}

TEST(FrameGraphRealtime, TheAllocationCounterItselfWorks) {
  // A test that counts allocations is worthless if the counter is inert -- this
  // is the positive control for every assertion below.
  //
  // The size arrives through a volatile read and the buffer is written and read
  // back afterwards. Both are needed: given a compile-time size and no use of
  // the contents, GCC at -O3 proves the vector is dead and deletes the
  // allocation outright, and the positive control then reports zero -- which is
  // indistinguishable from the counter being broken, and is the one failure
  // this test exists to make impossible.
  static volatile std::size_t requested = 1024;
  const std::size_t size = requested;
  std::size_t observed = 0;
  const std::size_t allocations = allocationsDuring([&] {
    std::vector<int> v(size, 7);
    v[size / 2] = 11;
    observed = v.size() + static_cast<std::size_t>(v[size / 2]);
  });
  EXPECT_EQ(observed, size + 11u);
  EXPECT_GT(allocations, 0u);
}

TEST(FrameGraphRealtime, LookupDoesNotAllocate) {
  DeepTree tree = buildDeepTree(12, 8);
  const FrameId a = tree.left.back();
  const FrameId b = tree.right.back();

  // Warm up outside the measurement; the first call must not be special.
  ASSERT_EQ(tree.graph.lookup(a, b).error, FrameError::None);

  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      const auto result = tree.graph.lookup(a, b);
      accumulator += result.value.translation().x;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);  // keep the loop alive
  EXPECT_EQ(allocations, 0u) << "lookup() allocated " << allocations
                             << " times over 1000 calls";
}

TEST(FrameGraphRealtime, SetTransformDoesNotAllocate) {
  DeepTree tree = buildDeepTree(10, 4);
  const FrameId joint = tree.left.front();
  const SE3 t(SO3::fromRPY(0.05, 0.0, 0.0), Vec3{0.01, 0.0, 0.0});

  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      (void)tree.graph.setTransform(joint, t);
    }
  });
  EXPECT_EQ(allocations, 0u);
}

TEST(FrameGraphRealtime, FindAllocatesNothingEither) {
  // Documented as setup-only because it is a linear scan, not because it
  // allocates -- worth pinning so the distinction stays true.
  DeepTree tree = buildDeepTree(6, 3);
  const std::size_t allocations =
      allocationsDuring([&] { (void)tree.graph.find("left2"); });
  EXPECT_EQ(allocations, 0u);
}

// ---------------------------------------------------------------------------
// Trajectories
// ---------------------------------------------------------------------------

constexpr MotionLimits kAxis{2.0, 8.0, 40.0};

TEST(TrajectoryRealtime, ProfileSamplingDoesNotAllocate) {
  const auto profile = ScurveProfile::plan(0.0, 2.0, kAxis);
  ASSERT_TRUE(profile.hasValue());

  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      const Scalar t = profile.value.duration() * static_cast<Scalar>(i) / 1000.0;
      accumulator += profile.value.sample(t).position;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);  // keep the loop alive
  EXPECT_EQ(allocations, 0u);
}

TEST(TrajectoryRealtime, SynchronizedSamplingDoesNotAllocate) {
  const std::array<Scalar, 3> start{0.0, 0.0, 0.5};
  const std::array<Scalar, 3> goal{1.0, 0.2, -0.4};
  const std::array<MotionLimits, 3> limits{kAxis, kAxis, kAxis};
  const auto trajectory = SynchronizedTrajectory::plan(start, goal, limits);
  ASSERT_TRUE(trajectory.hasValue());

  // Caller-provided storage, which is the whole reason sample() takes a span
  // rather than returning a container.
  std::array<MotionSample, 3> out{};
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      const Scalar t = trajectory.value.duration() * static_cast<Scalar>(i) / 1000.0;
      (void)trajectory.value.sample(t, out);
    }
  });
  EXPECT_EQ(allocations, 0u);
}

// Planning is normally setup work, so this is a stronger claim than it looks:
// it is what makes a mid-move re-plan -- a feed-rate override, a new goal --
// something the cyclic task can do itself rather than hand to another thread.
TEST(TrajectoryRealtime, PlanningDoesNotAllocateEither) {
  const std::array<Scalar, 3> start{0.0, 0.0, 0.5};
  const std::array<Scalar, 3> goal{1.0, 0.2, -0.4};
  const std::array<MotionLimits, 3> limits{kAxis, kAxis, kAxis};

  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 200; ++i) {
      const Scalar override_factor = 0.5 + 0.002 * static_cast<Scalar>(i);
      const std::array<MotionLimits, 3> scaled{limits[0].scaled(override_factor),
                                               limits[1].scaled(override_factor),
                                               limits[2].scaled(override_factor)};
      const auto planned = SynchronizedTrajectory::plan(start, goal, scaled);
      ASSERT_TRUE(planned.hasValue());
    }
  });
  EXPECT_EQ(allocations, 0u);
}

// A stop is planned by whatever notices that the machine has to stop, which may
// well be the safety task itself. Allocating there would make the stop depend
// on the allocator being in a good mood.
TEST(TrajectoryRealtime, PlanningAStopDoesNotAllocate) {
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 500; ++i) {
      const Scalar velocity = 0.004 * static_cast<Scalar>(i);
      const auto stop =
          StopProfile::plan(MotionState{0.0, velocity, 8.0 - 0.03 * velocity}, kAxis);
      ASSERT_TRUE(stop.hasValue());
    }
  });
  EXPECT_EQ(allocations, 0u);
}

TEST(TrajectoryRealtime, SamplingAStopDoesNotAllocate) {
  const auto stop = StopProfile::plan(MotionState{0.0, 2.0, 4.0}, kAxis);
  ASSERT_TRUE(stop.hasValue());

  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      const Scalar t = stop.value.duration() * static_cast<Scalar>(i) / 1000.0;
      accumulator += stop.value.sample(t).position;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);  // keep the loop alive
  EXPECT_EQ(allocations, 0u);
}

// ---------------------------------------------------------------------------
// Kinematics
// ---------------------------------------------------------------------------

TEST(TrajectoryRealtime, ReachingFromAnArbitraryStateDoesNotAllocate) {
  // Planning from the state the executor is already holding is the operation a
  // controller performs when a target changes mid-move, so it happens on the
  // cyclic path rather than ahead of it. The bisection runs a fixed sixty
  // iterations over stack values; nothing about it grows.
  const MotionLimits limits{2.0, 6.0, 40.0};
  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      const Scalar phase = static_cast<Scalar>(i) * 0.01;
      const MotionState from{0.0, 1.5 * std::sin(phase), 4.0 * std::cos(phase)};
      const auto planned = ReachProfile::plan(from, 1.0, limits);
      accumulator += planned.value.duration();
      accumulator += planned.value.sample(planned.value.duration() * 0.5).position;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);
  EXPECT_EQ(allocations, 0u);
}

TEST(CollisionRealtime, CheckingClearanceDoesNotAllocate) {
  // A clearance check belongs on the cyclic path -- it is what a supervisor
  // runs before letting a setpoint through -- so the shapes live in fixed
  // arrays and the query builds nothing.
  const auto model = CollisionModel::sixAxisExample();
  ASSERT_TRUE(model);
  const std::array<Scalar, 6> q{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  const std::array<Capsule, 3> obstacles{
      Capsule{Vec3{0.6, 0.0, 0.0}, Vec3{0.6, 0.0, 1.2}, 0.05},
      Capsule{Vec3{-0.5, 0.4, 0.3}, Vec3{-0.5, 0.4, 0.3}, 0.10},
      Capsule{Vec3{0.0, -0.7, 0.0}, Vec3{0.4, -0.7, 0.9}, 0.03}};

  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      accumulator += model.value.clearance(q, obstacles).value.distance;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);
  EXPECT_EQ(allocations, 0u);
}

TEST(CartesianRealtime, SamplingAPlannedMoveDoesNotAllocate) {
  // Planning a Cartesian move is expensive and happens once. Sampling it
  // happens every control cycle, which is why the plan stores its knots in a
  // fixed array and interpolates in place rather than holding a container.
  const SerialChain arm = SerialChain::sixAxisExample();
  const std::array<Scalar, 6> start{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  const SE3 from = arm.forward(start).value;
  const SE3 to{from.rotation(), from.translation() + Vec3{0.10, -0.05, 0.08}};
  const std::array<MotionLimits, 6> limits{
      MotionLimits{2.0, 8.0, 60.0}, MotionLimits{2.0, 8.0, 60.0},
      MotionLimits{2.0, 8.0, 60.0}, MotionLimits{2.0, 8.0, 60.0},
      MotionLimits{2.0, 8.0, 60.0}, MotionLimits{2.0, 8.0, 60.0}};
  const auto planned = CartesianPlan::plan(arm, start, to, limits);
  ASSERT_TRUE(planned);

  std::array<Scalar, 6> q{};
  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      const Scalar t = planned.value.duration() * static_cast<Scalar>(i) / 1000.0;
      (void)planned.value.sample(t, q);
      accumulator += q[0] + planned.value.poseAtTime(t).translation().x;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);
  EXPECT_EQ(allocations, 0u);
}

TEST(CalibrationRealtime, SolvingDoesNotAllocateHoweverManySamplesArrive) {
  // Calibration is a setup procedure, not a cyclic-task operation, so this is
  // not about deadlines. It pins a design property: both solvers accumulate
  // into fixed-size normal equations -- six unknowns for the tool point, four
  // and three for hand-eye -- so their working set does not grow with the
  // number of measurements. Thirty stations cost the same memory as three.
  std::array<SE3, 30> flange{};
  std::array<SE3, 30> views{};
  const SE3 mounting{SO3::fromRPY(0.15, -0.62, 1.05), Vec3{0.043, -0.028, 0.091}};
  const Vec3 tool{0.031, -0.017, 0.184};
  const Vec3 touched{0.612, -0.204, 0.338};
  std::array<SE3, 30> touches{};
  for (std::size_t i = 0; i < flange.size(); ++i) {
    const Scalar t = static_cast<Scalar>(i) * 0.21;
    const SO3 turn = SO3::fromRPY(0.4 * std::sin(t), 0.5 * std::cos(t * 1.3), t * 0.7);
    flange[i] = SE3{turn, Vec3{0.6 + 0.01 * t, 0.02 * t, 0.4 - 0.01 * t}};
    views[i] = (flange[i] * mounting).inverse() * SE3{SO3{}, Vec3{1.1, 0.25, 0.05}};
    touches[i] = SE3{turn, touched - (turn * tool)};
  }

  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 50; ++i) {
      accumulator += calibrateToolPoint(touches).value.flange_t_tool.z;
      accumulator +=
          calibrateHandEye(flange, views).value.flange_T_camera.translation().z;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);
  EXPECT_EQ(allocations, 0u);
}

TEST(DynamicsRealtime, InverseDynamicsDoesNotAllocate) {
  const DynamicChain arm = DynamicChain::sixAxisExample();
  const std::array<Scalar, 6> q{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  const std::array<Scalar, 6> qd{0.5, 0.2, -0.4, 0.9, -0.1, 0.3};
  const std::array<Scalar, 6> qdd{-0.2, 1.1, 0.6, -0.7, 0.4, 0.8};
  std::array<Scalar, 6> tau{};

  // The whole reason the recursion carries its intermediates in fixed arrays
  // rather than vectors: a torque computed on the control thread must not wait
  // on an allocator.
  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      (void)arm.inverseDynamics(q, qd, qdd, tau);
      accumulator += tau[0];
    }
  });
  EXPECT_NE(accumulator, 12345.6789);
  EXPECT_EQ(allocations, 0u);
}

TEST(DynamicsRealtime, TheMassMatrixAndTheEnergiesDoNotAllocate) {
  const DynamicChain arm = DynamicChain::sixAxisExample();
  const std::array<Scalar, 6> q{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  const std::array<Scalar, 6> qd{0.5, 0.2, -0.4, 0.9, -0.1, 0.3};
  std::array<Scalar, 36> mass{};

  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      (void)arm.massMatrix(q, mass);
      accumulator += mass[0];
      accumulator += arm.kineticEnergy(q, qd).value;
      accumulator += arm.potentialEnergy(q).value;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);
  EXPECT_EQ(allocations, 0u);
}

TEST(KinematicsRealtime, ForwardKinematicsAndTheJacobianDoNotAllocate) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const std::array<Scalar, 6> q{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  std::array<Scalar, kTwistSize * 6> jac{};

  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      accumulator += arm.forward(q).value.translation().x;
      (void)arm.jacobian(q, jac);
      accumulator += jac[0];
    }
  });
  EXPECT_NE(accumulator, 12345.6789);  // keep the loop alive
  EXPECT_EQ(allocations, 0u);
}

// The stronger claim. A solver that allocates cannot run in the cycle that
// needs its answer, which is how inverse kinematics ends up on another thread
// with a queue in front of it and a latency nobody measured.
TEST(KinematicsRealtime, SolvingInverseKinematicsDoesNotAllocate) {
  const SerialChain arm = SerialChain::sixAxisExample();
  const std::array<Scalar, 6> truth{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
  const SE3 target = arm.forward(truth).value;

  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 200; ++i) {
      std::array<Scalar, 6> q = truth;
      q[0] += 0.05 * static_cast<Scalar>(i % 5);
      const auto report = arm.inverse(target, q);
      ASSERT_TRUE(report.hasValue());
    }
  });
  EXPECT_EQ(allocations, 0u);
}

}  // namespace
}  // namespace motionkit
