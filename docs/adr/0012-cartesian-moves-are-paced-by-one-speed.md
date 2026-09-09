# ADR-0012: A Cartesian move is paced by one speed, and the path between knots is C1

- **Status**: Accepted
- **Date**: 2026-09-09
- **Deciders**: Onur Can Urhan

## Context

WP-12 is the module that joins the two halves of this library. Until now
kinematics answered *where* and knew nothing about time, the trajectory profiles
answered *when* over scalar axes and knew nothing about poses, and the component
diagram in [docs/architecture.md](../architecture.md) showed two subtrees rising
from `types` that never met.

A straight-line Cartesian move needs both at once. It is harder than either half
because the relationship between them is not a constant: the Jacobian mapping
joint rates to tool velocity is different at every point on the path, and near a
singularity it demands unbounded joint rates for a perfectly ordinary tool
speed. Measured on the example arm, the same 50 mm move takes **8.15 s** from a
configuration with manipulability 6.2e-05 and **1.10 s** from one with 3.7e-03 --
a factor of **7.4** for a move of identical length and direction.

## Decision 1: one speed for the whole path, not time-optimal parameterisation

The path is sampled at knots, solved by inverse kinematics, and then driven by a
**single** jerk-limited profile on the path parameter whose limits are the
tightest any knot requires. True time-optimal path parameterisation would let
the speed vary along the path, running fast where the arm is well conditioned
and slowing only where it is not.

This is conservative, and the conservatism is real: an arm that must crawl past
one bad point crawls for the entire move. It is never *wrong* about the limits,
which is the property that matters, and `binding_s` and `least_manipulability`
say exactly where the cost was incurred -- which on a disappointing cycle time
is the information somebody actually needs.

TOPP was rejected for now rather than forever. It is a substantially larger
piece of work: the speed profile becomes a differential constraint solved
forward and backward along the path with switching points where the binding
constraint changes, and getting it wrong produces a plan that is fast and
occasionally exceeds a limit -- which is worse than one that is slow. Shipping
the conservative version with the diagnostics that identify when it hurts is the
honest intermediate step.

## Decision 2: between knots the path is cubic Hermite, not linear

This one is not a refinement; without it the acceleration limits are
unenforceable.

Linear interpolation between knots makes joint velocity **piecewise constant**.
It steps at every knot crossing, so joint acceleration there is an impulse --
unbounded, and no acceleration budget computed anywhere can survive it. A plan
built that way satisfies its own internal model and hands out a stream that
violates the limits at every knot along the move.

Cubic Hermite with the same finite-difference tangents the pacing already uses
is C1: velocity is continuous and acceleration is finite. The tangents are
recomputed from neighbouring knots at sample time rather than stored, which
costs four array reads and saves a second array the size of the first.

The test that matters differentiates the **sampled output** -- the numbers a
controller would actually receive -- rather than the internal model. Peak values
on a representative move, against limits of 2.0 rad/s and 8.0 rad/s squared:

    peak joint speed          1.7109  of 2.0
    peak joint acceleration   4.5142  of 8.0

Inside the limits, and using enough of the budget that the planner is not merely
being timid.

## Decision 3: the acceleration budget is split explicitly

Joint acceleration has two sources, and they compete for one limit: the path
bending in joint space, which grows with the **square** of path speed, and the
profile changing speed along the path, which is linear in it.

The first is invisible to anyone who only bounds the second. A planner that
limits the profile alone respects every acceleration limit on paper and exceeds
them on a curve.

`CartesianOptions::cornering_share` divides the budget, and the cornering half
feeds back into the *speed* bound rather than being discovered afterwards: the
bending term must fit inside its share, which caps the path speed independently
of the velocity limit. Half each is the symmetric choice and no measurement
argues for another; it is exposed because the right split depends on whether a
particular move is dominated by its corners or by its ends.

## Decision 4: each knot is seeded from the one before

A 6R arm reaches most poses in up to eight configurations. Solving each knot
cold would let neighbouring knots land in different ones, producing a plan in
which every individual pose is correct and which asks the arm to turn itself
inside out halfway along a straight line.

Seeding from the previous knot makes the solve continuous by construction. A
test samples the plan 200 times and asserts the largest joint step between
consecutive samples; on a representative move it is **0.034 rad**, which is a
plan that moves rather than one that jumps.

## Decision 5: the deviation from the line is measured and reported, not claimed

Interpolation between knots is in *joint* space -- Hermite, but still joint
space -- and a smooth curve in joint space is not a straight line in Cartesian
space. The tool is on the line exactly at the knots and near it in between.

`CartesianReport::chord_deviation` measures the gap at knot midpoints, where it
is largest. On a 200 mm move with 33 knots it is **2.9e-05 m**, and a dense
sweep of the actual sampled path finds **3.1e-05 m** -- so the reported figure is
an honest bound rather than a number chosen to look good. Reporting it is the
alternative to claiming the tool travels in a straight line, which it does not.

## Decision 6: running out of travel is a different failure from being unreachable

`JointLimitAlongPath` is separate from `UnreachableAlongPath`, and the split came
out of the tests rather than out of design. A goal that rotates the tool 0.25 rad
about world Z from the example arm's usual pose drags joint 0 round to its
2.967 rad stop -- while every pose on that line is perfectly reachable and the
line is 164 mm long.

Reporting that as "unreachable" sends somebody to check the workspace. The
actual fix is to start from a different arm configuration or to remount the
workpiece. The two errors point at different people.

## Consequences

- Planning costs **77 microseconds** for 33 knots -- 32 seeded inverse solves
  plus the pacing pass -- and sampling the result costs **18 nanoseconds**. The
  factor of four thousand between them is the design: plan once, sample every
  cycle. Sampling allocates nothing and a test asserts it.
- Holding the tool to a straight line costs time against letting the joints go
  where they like. Between the same endpoints with the same limits: **3.52 s**
  Cartesian against **1.53 s** joint-space, a factor of **2.30**. That is the
  price of the line, and it is worth knowing before promising a cycle time.
- The knot count is the caller's, bounded by `kMaxPathKnots`. More knots follow
  the line more closely and cost one inverse solve each; the plan stores a
  configuration per knot, which is what bounds it.
- No blending between consecutive moves. Each plan starts and ends at rest, so a
  sequence of Cartesian moves stops at every waypoint. Blending needs a profile
  that starts from a non-zero state, which `ScurveProfile` does not currently
  provide -- the same gap named in the work package, and still open.
- No collision checking and no CUDA. Both were listed under WP-12 and neither is
  here; batch inverse kinematics on a GPU is a different kind of work, and the
  single-solve cost of 2.3 microseconds has not yet made it necessary.

## Alternatives considered

**Interpolating in Cartesian space and solving inverse kinematics every control
cycle.** The tool would then be exactly on the line at every instant rather than
only at the knots. It costs a 2.3-microsecond solve per cycle, which fits inside
1 kHz, and it was rejected for a different reason: the solve can fail, and a
control loop that discovers at cycle four thousand that it cannot reach the next
setpoint has no good options left. Solving the whole path in advance means every
failure is a planning failure, which is a failure somebody can act on.

**Storing the tangents alongside the knots.** Would save four array reads per
sample and double the plan's storage. At 18 nanoseconds a sample there is
nothing to buy.

**Letting the caller supply the path speed and validating it.** Simpler, and it
moves the interesting question -- how fast *may* this move go -- back to the
caller, who has no way to answer it without the Jacobian along the path.
