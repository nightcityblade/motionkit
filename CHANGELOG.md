# Changelog

All notable changes to this project are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Versioning policy, and what 0.x means here

The project version is **0.1.0** and **nothing has been released**: there are no
tags, and every entry below is unreleased. Saying so is more useful than
back-dating a release history onto commits that were never published.

Under semver, `0.x` means the public API may change in any release. That is an
accurate description of the current state and not a disclaimer to be ignored —
`SerialChain` and `SynchronizedTrajectory` are both young enough that use will
change them. Two things are nonetheless already firm, because they are cheap to
keep and expensive to break:

- **Exported CMake target names.** `motionkit::core` and `motionkit::warnings`
  are stable, and CI builds a consumer against the installed package on every
  run so they cannot drift ([ADR-0004](docs/adr/0004-verify-the-installed-package-in-ci.md)).
- **Frame naming.** `A_T_B` reads as "the pose of B in A" throughout, and
  composition cancels. Reversing that convention would silently invert
  transforms in caller code rather than fail to compile, so it will not change.

At 1.0.0 these become the ordinary semver promises. Until then, read a minor
bump as "something may have moved".

## [Unreleased]

### Added

- **Straight-line Cartesian moves** (WP-12a). `CartesianPlan` follows a line in
  space with the orientation slerping along it, solving inverse kinematics at
  knots seeded from one another and pacing the whole move so that no joint
  exceeds its velocity or acceleration limit. Reports where the pace was set,
  the closest approach to a singularity, and how far the tool leaves the line
  between knots.
  ([ADR-0012](docs/adr/0012-cartesian-moves-are-paced-by-one-speed.md))
- **Tool-point and hand-eye calibration** (WP-06). `calibrateToolPoint` finds
  the tool tip from poses that touch one fixed point; `calibrateHandEye` finds a
  flange-mounted camera's pose from stations watching a fixed target. Both
  refuse geometry that cannot determine the answer rather than returning a
  plausible number, and both report residuals so a precise result can be told
  from an imprecise one.
  ([ADR-0011](docs/adr/0011-calibration-refuses-what-it-cannot-determine.md))
- **Rigid-body dynamics** (WP-04). `DynamicChain` computes joint torques by
  recursive Newton-Euler and the joint-space mass matrix by the
  composite-rigid-body algorithm, both allocation-free and callable from a
  cyclic task. Gravity is a settable field vector, so an arm on a wall or a
  ceiling is not a special case. Inertia tensors that could not belong to a
  real body are refused at construction rather than producing plausible wrong
  torques. ([ADR-0010](docs/adr/0010-dynamics-in-the-base-frame.md))
- **`SerialChain::linkTransforms`** — where each link has been carried at a
  given configuration. Added because dynamics needs it, and exposed rather than
  duplicated because the asymmetry it encodes (a joint is carried by everything
  upstream of it but not by itself) is worth stating once.
- **Forward and inverse kinematics for serial chains** (WP-03). `SerialChain`
  describes revolute joints by an axis and a point rather than DH parameters,
  computes forward kinematics as a product of exponentials, and solves the
  inverse by damped least squares with joint limits enforced. `IkReport`
  returns the closest approach to a singularity alongside the result.
  ([ADR-0008](docs/adr/0008-kinematics-by-screws-and-a-damped-inverse.md))
- **Stop planning from an arbitrary state** (WP-11). `StopProfile` plans a
  three-segment stop from any velocity and acceleration, including a state
  already outside the acceleration limit, and reports the stopping distance the
  safety envelope is built from.
  ([ADR-0007](docs/adr/0007-stopping-is-planned-to-zero-acceleration.md))
- **Jerk-limited trajectory planning** (WP-05). Seven-segment S-curve profiles,
  and `SynchronizedTrajectory` driving several axes from a single path
  parameter so they start and finish together.
  ([ADR-0006](docs/adr/0006-jerk-limited-profiles-and-a-single-path-parameter.md))
- **Frame graph** (WP-02). A named frame tree with allocation-free lookup
  routed through the lowest common ancestor.
  ([ADR-0005](docs/adr/0005-frame-graph-is-a-tree.md))
- **SO(3) and SE(3)** (WP-02). Quaternion-backed rotations with a canonical
  unit-norm invariant, renormalising composition, and conversions to matrix,
  rotation vector and Z-Y-X Euler angles.
  ([ADR-0001](docs/adr/0001-quaternion-storage-for-so3.md),
  [ADR-0003](docs/adr/0003-well-conditioned-rotation-metric.md))
- **Doxygen API reference**, built as a CI gate rather than only a website
  ([ADR-0009](docs/adr/0009-the-api-reference-is-a-gate.md)). Publication to
  GitHub Pages is opt-in via the `PUBLISH_DOCS` repository variable.
- **Architecture documentation** in [docs/architecture.md](docs/architecture.md):
  C4 context, container and component views, and the rules that decide where
  new code belongs.
- **Build system, CI and packaging** (WP-01). GCC and Clang in Debug and
  Release, ASan/UBSan and TSan, clang-tidy and clang-format, and an installed
  package verified by building a consumer against it.
  ([ADR-0002](docs/adr/0002-static-analysis-rule-set.md))

### Fixed

- `RevoluteJoint::upper` and `IkReport::orientation_error` were undocumented in
  the generated reference. In both cases a single comment above the preceding
  member appeared in the source to describe both, but bound only to the first.
  Found by turning on the documentation gate; invisible when reading the header.
