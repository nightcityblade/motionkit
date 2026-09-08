# ADR-0008: Kinematics described by screws, and an inverse that is damped on purpose

- **Status**: Accepted
- **Date**: 2026-09-06
- **Deciders**: Onur Can Urhan

## Context

WP-03 needs forward kinematics, a Jacobian and an inverse solve for a six-axis
arm. Everything above it wants them: the trajectory planner needs joint targets,
the frame graph needs somewhere for the joint values to come from, and the
runtime needs all of it inside a control cycle.

Three choices decide whether the result is usable, and each has an obvious
answer that is wrong in a way that only shows up later.

## Decision 1: describe joints by an axis and a point, not by DH parameters

A joint is a `Vec3` axis and a `Vec3` point on that axis, both given **in the
base frame with every joint at zero**. Forward kinematics is then a product of
exponentials, and for a revolute joint each factor is just "translate to the
point, rotate about the axis, translate back" — three lines that can be checked
by inspection.

Denavit-Hartenberg parameters are the traditional answer and they are worse
here for reasons that have nothing to do with elegance:

- A DH table requires a **specific frame attached to every link**, assigned by
  rules with genuine freedom in them. Two engineers parameterising the same arm
  produce different tables.
- There are **two incompatible conventions** in circulation — standard and
  modified — that produce *different arms* from identical numbers, and tables in
  the wild frequently do not say which they are.
- None of the four parameters is something you can measure directly. An axis
  direction and a point on it are both readable off a CAD model with a ruler.

The screw form also reuses the exponential map already in `SO3`, so there is one
rotation implementation rather than two.

## Decision 2: the inverse is iterative, and the damping is not optional

A spherical-wrist 6R admits a closed-form inverse, and a closed form is exact,
fast and returns **all eight** configurations at once. It also only exists for
that topology: change the wrist, add a joint, and it is gone. This library takes
the iterative route — damped least squares — and pays for generality with:

- one solution rather than eight, whichever the seed falls into
- convergence that is not guaranteed
- 2.3 µs instead of a few hundred nanoseconds

That last figure is the one that makes the trade acceptable. 2.3 µs is **0.23% of
a 1 kHz cycle** and allocates nothing, so the solve can happen in the loop that
needs the answer rather than on a thread with a queue in front of it.

### Why undamped is not a neutral default

The step is `dq = Jᵀ (J Jᵀ + λ²I)⁻¹ e`. Setting `λ = 0` gives the Moore-Penrose
pseudo-inverse, which divides by the Jacobian's smallest singular value — and
that value goes to zero at a singularity.

Measured on the example arm with its wrist a tenth of a milliradian from
straight, asked for a **one-milliradian** tool rotation
(`KinematicsSingularity.DampingBoundsTheStepThatOtherwiseDiverges`):

| damping | commanded joint step |
|---|---|
| 0 | **3.188 rad** (183°) |
| 1e-3 | **0.0019 rad** (0.11°) |

A factor of about **1700**. The arm is asked to turn the tool by a twentieth of a
degree and the undamped solver answers by swinging a joint half a turn. Damping
trades a little tracking error for a bounded step, and there is no configuration
in which the untraded version is what anyone wanted.

A separate hard clamp on the step size backs it up, because damping is a soft
bound and a solver handed an unreachable target will otherwise take one large
confident stride in a plausible direction.

## Decision 3: report manipulability, do not threshold it

`manipulability()` returns Yoshikawa's `sqrt(det(J Jᵀ))`. It is **not** compared
against a limit anywhere, because it has units — metres to some power times
radians to another — and is not comparable between arms of different size. A
threshold would be a number somebody invented, applied to a quantity whose scale
depends on the machine.

`IkReport` carries the smallest value seen during a solve, so a caller can notice
that a converged answer passed close to a singularity. That is a fact worth
having; what to do about it is the caller's decision and depends on the machine.

A chain of fewer than six joints has a singular Gram matrix everywhere, so the
measure is zero for it always. That is the honest answer to "how far is this arm
from being unable to move in some direction": a five-axis arm cannot produce an
arbitrary twist anywhere in its workspace.

## Consequences

- **The seed is part of the question.** A 6R arm reaches most poses in up to
  eight configurations and an iterative solver returns whichever basin the seed
  is in. Seeding from the current configuration is what stops the arm turning
  itself inside out between two nearby waypoints. The round-trip test therefore
  asserts on the **pose reached**, never on the joint angles — asserting on the
  angles would be asserting on which basin the seed happened to occupy.
- **The example arm's zero configuration is singular.** Straight up, joints 1, 4
  and 6 are collinear. That is not an artefact of the example — it is true of a
  great many real machines — and it is the clearest available argument for why
  `inverse()` takes a seed rather than starting somewhere tidy.
- **The Jacobian is checked against the derivative it claims to be.** Central
  differences at four configurations, including the singular one, agree with the
  analytic form to **3.8e-10**. A sign or frame error in one of six rows produces
  a solver that converges for some targets and spirals for others, which is far
  harder to diagnose than one that never works at all.
- Forward kinematics costs 177 ns, the Jacobian 427 ns, and none of it allocates.
- **Dynamics (WP-04) is not addressed.** No masses, no inertias, no torques. The
  Jacobian here is geometric, not the one a dynamics formulation needs, and
  pretending otherwise would be the same mistake as offering a partial
  non-zero-initial-state trajectory planner in ADR-0006.
