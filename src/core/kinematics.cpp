// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/se3.hpp"
#include "motionkit/core/so3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

/// The screw motion of a revolute joint: translate to a point on the axis,
/// rotate about it, translate back.
///
/// This is the whole of the product-of-exponentials formula for a revolute
/// joint, and writing it this way rather than as a general screw exponential
/// makes it something that can be checked by inspection.
SE3 jointTransform(const RevoluteJoint& joint, Scalar angle) noexcept {
  const SO3 rotation = SO3::fromAxisAngle(joint.axis, angle);
  return {rotation, joint.point - (rotation * joint.point)};
}

using Matrix6 = std::array<Scalar, kTwistSize * kTwistSize>;
using Twist = std::array<Scalar, kTwistSize>;

/// Cholesky factorisation of a 6x6 symmetric matrix, in place into `lower`.
///
/// Returns false when the matrix is not positive definite, which for J*J^T
/// means the Jacobian has lost rank -- the arm is exactly at a singularity.
/// That is a real answer rather than a failure, and the callers here treat it
/// as one.
bool cholesky(const Matrix6& a, Matrix6& lower) noexcept {
  lower.fill(0.0);
  for (std::size_t i = 0; i < kTwistSize; ++i) {
    for (std::size_t j = 0; j <= i; ++j) {
      Scalar sum = a[i * kTwistSize + j];
      for (std::size_t k = 0; k < j; ++k) {
        sum -= lower[i * kTwistSize + k] * lower[j * kTwistSize + k];
      }
      if (i == j) {
        if (!(sum > 0.0)) {
          return false;
        }
        lower[i * kTwistSize + j] = std::sqrt(sum);
      } else {
        lower[i * kTwistSize + j] = sum / lower[j * kTwistSize + j];
      }
    }
  }
  return true;
}

/// Solves L * L^T * x = b for x, given the factor from cholesky().
void choleskySolve(const Matrix6& lower, const Twist& b, Twist& x) noexcept {
  Twist y{};
  for (std::size_t i = 0; i < kTwistSize; ++i) {
    Scalar sum = b[i];
    for (std::size_t k = 0; k < i; ++k) {
      sum -= lower[i * kTwistSize + k] * y[k];
    }
    y[i] = sum / lower[i * kTwistSize + i];
  }
  for (std::size_t i = kTwistSize; i-- > 0;) {
    Scalar sum = y[i];
    for (std::size_t k = i + 1; k < kTwistSize; ++k) {
      sum -= lower[k * kTwistSize + i] * x[k];
    }
    x[i] = sum / lower[i * kTwistSize + i];
  }
}

bool allFinite(std::span<const Scalar> values) noexcept {
  return std::ranges::all_of(values, [](Scalar value) { return std::isfinite(value); });
}

}  // namespace

std::string_view toString(KinematicsError error) noexcept {
  switch (error) {
    case KinematicsError::None:
      return "none";
    case KinematicsError::SizeMismatch:
      return "joint count or buffer size does not match the chain";
    case KinematicsError::JointCountUnsupported:
      return "no joints, or more than kMaxJoints";
    case KinematicsError::DegenerateAxis:
      return "a joint axis has no direction";
    case KinematicsError::NonFiniteInput:
      return "a joint angle or pose was NaN or infinite";
    case KinematicsError::DidNotConverge:
      return "the solver ran out of iterations";
    case KinematicsError::OutsideJointLimits:
      return "the solution lies outside the joint limits";
  }
  return "unrecognised KinematicsError";
}

Expected<SerialChain, KinematicsError> SerialChain::build(
    std::span<const RevoluteJoint> joints, const SE3& base_T_tool_at_zero) {
  if (joints.empty() || joints.size() > kMaxJoints) {
    return {SerialChain{}, KinematicsError::JointCountUnsupported};
  }

  SerialChain chain;
  chain.count_ = joints.size();
  chain.base_T_tool_at_zero_ = base_T_tool_at_zero;

  for (std::size_t i = 0; i < joints.size(); ++i) {
    const RevoluteJoint& source = joints[i];
    const Scalar length = source.axis.norm();
    // Normalised once, here, so that no later call has to wonder whether the
    // caller supplied a unit vector -- and a zero axis is refused rather than
    // producing an identity rotation that would look like a joint welded
    // solid.
    if (!std::isfinite(length) || length < kAngleEpsilon) {
      return {SerialChain{}, KinematicsError::DegenerateAxis};
    }
    if (!std::isfinite(source.point.x) || !std::isfinite(source.point.y) ||
        !std::isfinite(source.point.z)) {
      return {SerialChain{}, KinematicsError::NonFiniteInput};
    }
    chain.joints_[i] = source;
    chain.joints_[i].axis = source.axis / length;
  }
  return {chain, KinematicsError::None};
}

Expected<SE3, KinematicsError> SerialChain::forward(
    std::span<const Scalar> q) const noexcept {
  if (q.size() != count_) {
    return {SE3{}, KinematicsError::SizeMismatch};
  }
  if (!allFinite(q)) {
    return {SE3{}, KinematicsError::NonFiniteInput};
  }

  SE3 pose;
  for (std::size_t i = 0; i < count_; ++i) {
    pose = pose * jointTransform(joints_[i], q[i]);
  }
  return {pose * base_T_tool_at_zero_, KinematicsError::None};
}

KinematicsError SerialChain::linkTransforms(std::span<const Scalar> q,
                                            std::span<SE3> out) const noexcept {
  if (q.size() != count_ || out.size() < count_) {
    return KinematicsError::SizeMismatch;
  }
  if (!allFinite(q)) {
    return KinematicsError::NonFiniteInput;
  }
  SE3 carried;
  for (std::size_t i = 0; i < count_; ++i) {
    carried = carried * jointTransform(joints_[i], q[i]);
    out[i] = carried;
  }
  return KinematicsError::None;
}

KinematicsError SerialChain::jacobian(std::span<const Scalar> q,
                                      std::span<Scalar> out) const noexcept {
  if (q.size() != count_ || out.size() != kTwistSize * count_) {
    return KinematicsError::SizeMismatch;
  }
  if (!allFinite(q)) {
    return KinematicsError::NonFiniteInput;
  }

  const auto tool = forward(q);
  if (!tool) {
    return tool.error;
  }
  const Vec3 tool_point = tool.value.translation();

  // Walk the chain once, carrying the transform of everything upstream of the
  // joint being differentiated. Joint i's axis and the point on it have both
  // been moved by joints 1..i-1, and by nothing downstream -- which is the
  // entire content of the geometric Jacobian.
  SE3 upstream;
  for (std::size_t i = 0; i < count_; ++i) {
    const Vec3 axis = upstream.rotateVector(joints_[i].axis);
    const Vec3 point = upstream * joints_[i].point;
    const Vec3 linear = axis.cross(tool_point - point);

    out[0 * count_ + i] = linear.x;
    out[1 * count_ + i] = linear.y;
    out[2 * count_ + i] = linear.z;
    out[3 * count_ + i] = axis.x;
    out[4 * count_ + i] = axis.y;
    out[5 * count_ + i] = axis.z;

    upstream = upstream * jointTransform(joints_[i], q[i]);
  }
  return KinematicsError::None;
}

namespace {

/// J * J^T for a 6 x n Jacobian stored row-major.
Matrix6 gram(std::span<const Scalar> jacobian, std::size_t n) noexcept {
  Matrix6 out{};
  for (std::size_t r = 0; r < kTwistSize; ++r) {
    for (std::size_t c = 0; c <= r; ++c) {
      Scalar sum = 0.0;
      for (std::size_t k = 0; k < n; ++k) {
        sum += jacobian[r * n + k] * jacobian[c * n + k];
      }
      out[r * kTwistSize + c] = sum;
      out[c * kTwistSize + r] = sum;
    }
  }
  return out;
}

/// sqrt(det(J * J^T)) from the Gram matrix, or zero when it is singular.
///
/// The Cholesky factorisation answers both questions at once: whether the
/// matrix is positive definite, and -- through the product of its diagonal --
/// the square root of its determinant.
Scalar manipulabilityFromGram(const Matrix6& jjt) noexcept {
  Matrix6 factor{};
  if (!cholesky(jjt, factor)) {
    return 0.0;
  }
  Scalar product = 1.0;
  for (std::size_t i = 0; i < kTwistSize; ++i) {
    product *= factor[i * kTwistSize + i];
  }
  return product;
}

/// Applies one damped-least-squares step to `q`, returning the largest joint
/// motion it commanded.
///
/// Extracted so that inverse() reads as the loop it is. The step itself is
/// `dq = J^T y`, clamped twice: by max_step, because damping is a soft bound
/// and a solver handed an unreachable target will otherwise take one large
/// confident stride, and then into the joints' own range.
Scalar applyStep(std::span<const RevoluteJoint> joints, std::span<const Scalar> jacobian,
                 const Twist& y, const IkOptions& options, std::span<Scalar> q,
                 bool& pinned_at_limit) noexcept {
  const std::size_t count = q.size();
  Scalar largest = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    Scalar step = 0.0;
    for (std::size_t r = 0; r < kTwistSize; ++r) {
      step += jacobian[r * count + i] * y[r];
    }
    step = std::clamp(step, -options.max_step, options.max_step);
    largest = std::max(largest, std::fabs(step));

    Scalar next = q[i] + step;
    if (options.enforce_joint_limits) {
      const Scalar clamped = std::clamp(next, joints[i].lower, joints[i].upper);
      // Recorded rather than acted on: a joint resting against its stop is
      // ordinary, and only matters if the solve then fails to converge.
      pinned_at_limit = pinned_at_limit || clamped != next;
      next = clamped;
    }
    q[i] = next;
  }
  return largest;
}

/// The twist that carries `current` onto `target`, in the base frame.
///
/// Rotation error is taken as the rotation vector of target * current^-1 --
/// applied on the left, so it is a world-frame displacement and matches the
/// frame the Jacobian's angular rows are written in.
Twist poseError(const SE3& current, const SE3& target) noexcept {
  const Vec3 linear = target.translation() - current.translation();
  const Vec3 angular =
      (target.rotation() * current.rotation().inverse()).rotationVector();
  return Twist{linear.x, linear.y, linear.z, angular.x, angular.y, angular.z};
}

}  // namespace

Expected<Scalar, KinematicsError> SerialChain::manipulability(
    std::span<const Scalar> q) const noexcept {
  std::array<Scalar, kTwistSize * kMaxJoints> buffer{};
  const std::span<Scalar> view(buffer.data(), kTwistSize * count_);
  if (const KinematicsError error = jacobian(q, view); error != KinematicsError::None) {
    return {0.0, error};
  }

  // A chain of fewer than six joints cannot span the twist space, so its Gram
  // matrix is singular and the measure is zero everywhere -- which is the
  // honest answer to "how far is this arm from being unable to move in some
  // direction".
  return {manipulabilityFromGram(gram(view, count_)), KinematicsError::None};
}

Expected<IkReport, KinematicsError> SerialChain::inverse(
    const SE3& base_T_target, std::span<Scalar> q,
    const IkOptions& options) const noexcept {
  if (q.size() != count_) {
    return {IkReport{}, KinematicsError::SizeMismatch};
  }
  if (!allFinite(q) || !std::isfinite(options.damping) ||
      !std::isfinite(options.max_step)) {
    return {IkReport{}, KinematicsError::NonFiniteInput};
  }

  std::array<Scalar, kTwistSize * kMaxJoints> buffer{};
  const std::span<Scalar> jac(buffer.data(), kTwistSize * count_);
  const Scalar lambda_squared = options.damping * options.damping;

  IkReport report;
  report.least_manipulability = std::numeric_limits<Scalar>::infinity();
  bool pinned_at_limit = false;

  for (std::size_t iteration = 0; iteration <= options.max_iterations; ++iteration) {
    report.iterations = iteration;

    const auto current = forward(q);
    if (!current) {
      return {report, current.error};
    }
    const Twist error = poseError(current.value, base_T_target);
    report.position_error =
        std::sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);
    report.orientation_error =
        std::sqrt(error[3] * error[3] + error[4] * error[4] + error[5] * error[5]);

    if (report.position_error <= options.position_tolerance &&
        report.orientation_error <= options.orientation_tolerance) {
      return {report, KinematicsError::None};
    }
    if (iteration == options.max_iterations) {
      break;
    }

    if (const KinematicsError failed = jacobian(q, jac);
        failed != KinematicsError::None) {
      return {report, failed};
    }
    const Matrix6 jjt = gram(jac, count_);

    report.least_manipulability =
        std::min(report.least_manipulability, manipulabilityFromGram(jjt));

    // Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e.
    //
    // The damping is what makes this factorisation always succeed. Without it
    // the matrix is exactly the one cholesky() just refused at a singularity,
    // and the step it would produce is the one that goes to infinity there.
    Matrix6 damped = jjt;
    for (std::size_t i = 0; i < kTwistSize; ++i) {
      damped[i * kTwistSize + i] += lambda_squared;
    }
    Matrix6 factor{};
    if (!cholesky(damped, factor)) {
      // Only reachable with a damping of zero at a singularity, which is the
      // configuration IkOptions warns against rather than forbids.
      return {report, KinematicsError::DidNotConverge};
    }
    Twist y{};
    choleskySolve(factor, error, y);

    report.largest_step =
        std::max(report.largest_step,
                 applyStep(std::span<const RevoluteJoint>(joints_.data(), count_), jac, y,
                           options, q, pinned_at_limit));
  }

  // Out of iterations. Which of the two answers is more useful depends on why:
  // a solver pressed against a joint stop has been told something different
  // from one that simply needed more steps.
  return {report, pinned_at_limit ? KinematicsError::OutsideJointLimits
                                  : KinematicsError::DidNotConverge};
}

SerialChain SerialChain::sixAxisExample() {
  // A small anthropomorphic arm: a rotating base, two pitch joints, and a
  // spherical wrist whose last three axes meet at one point. Dimensions are in
  // metres and are of the order of a 5 kg-payload industrial robot.
  //
  // The zero configuration has the arm straight up, which puts joints 1, 4 and
  // 6 on the same line and makes it **singular**. That is not an artefact of
  // this example -- it is true of a great many real machines -- and it is the
  // clearest possible argument for why inverse() takes a seed rather than
  // starting from wherever seemed tidy.
  constexpr Scalar kBaseHeight = 0.15;
  constexpr Scalar kUpperArm = 0.43;
  constexpr Scalar kForearm = 0.39;
  constexpr Scalar kWristToTool = 0.10;
  constexpr Scalar kShoulder = kBaseHeight;
  constexpr Scalar kElbow = kShoulder + kUpperArm;
  constexpr Scalar kWrist = kElbow + kForearm;

  const std::array<RevoluteJoint, 6> joints{{
      {Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 0.0}, -2.9670, 2.9670},
      {Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, kShoulder}, -1.9199, 1.9199},
      {Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, kElbow}, -2.6180, 2.6180},
      {Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, kWrist}, -3.3161, 3.3161},
      {Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, kWrist}, -2.0944, 2.0944},
      {Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, kWrist}, -6.2832, 6.2832},
  }};
  const SE3 tool = SE3::fromTranslation(Vec3{0.0, 0.0, kWrist + kWristToTool});
  // build() only fails on a degenerate axis, and these are literals.
  return build(joints, tool).value;
}

}  // namespace motionkit
