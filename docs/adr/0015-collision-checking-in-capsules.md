# ADR-0015: Collision checking in capsules, with an allowed-collision set that is not optional

- **Status**: Accepted
- **Date**: 2026-09-10
- **Deciders**: Onur Can Urhan

## Context

WP-12d bundled three things: full time-optimal path parameterisation, CUDA
batch inverse kinematics, and collision checking. Only the third is here, and
the other two are open for reasons worth writing down rather than leaving to be
inferred from an empty directory.

**CUDA cannot be verified.** GitHub's runners have no GPU, and every claim this
library makes is currently gated by something that runs on every pull request.
A component whose correctness rested on a machine under a desk would be the
first exception, and the first exception is how a project stops being able to
say its tests mean anything.

**Full TOPP conflicts with a decision already made.** Classical TOPP optimises
in the phase plane of the path parameter and its rate, subject to velocity and
acceleration. It has **no jerk limit** -- and this library's README calls jerk
"the limit that distinguishes an S-curve from a trapezoid, and the one that
decides whether the mechanics ring". Shipping a planner that quietly dropped it
in exchange for a shorter cycle time would contradict
[ADR-0006](0006-jerk-limited-profiles-and-a-single-path-parameter.md). Adding
jerk to TOPP is a substantially harder problem than TOPP, not a refinement of
it, and pretending otherwise in a commit message would be the worst of the
options.

Collision checking has none of those problems. It is exactly testable, needs no
hardware, and answers a question the Cartesian planner already half-answers:
`least_manipulability` reports how close a move came to a singularity, and
nothing reported how close it came to the fence.

## Decision 1: everything is a capsule

A link, a fence post, a fixture, a cable: all of them a segment with a radius.

The distance between two capsules is the distance between two segments minus
their radii. That is **one function with no special cases and no orientation to
track**, and it is the entire geometry in the file. Boxes need separating-axis
tests and an orientation each; meshes need a broad phase before they are
affordable at all.

The cost is fidelity, and it is a real cost. A link that is not very
capsule-shaped is covered generously, so the model refuses moves that would
have been fine. That is the right direction to be wrong in, and it is cheaper to
fix by adding a second capsule than by adopting a mesh.

Capsules are given **in the base frame with every joint at zero**, the same
frame joints and inertias use, so all three can be read off one CAD model in one
pose. [ADR-0008](0008-kinematics-by-screws-and-a-damped-inverse.md) explains why
this library keeps asking for that frame and no other.

## Decision 2: skipping adjacent links is not enough, and believing it is breaks the model

Two links sharing a joint touch at every configuration, so a self-collision
check that includes them reports a collision always. Skipping adjacent pairs is
obvious and universal.

It is also insufficient, and the insufficiency is not obvious. A spherical wrist
turns its last three links about nearly one point, so links **two** apart
overlap there at every configuration as well. A model with only the adjacent
rule reports a collision while the arm is parked.

That failure has a predictable ending. Nobody debugs a collision checker that
fires on a stationary machine; they switch collision checking off, and then the
cell has none. So `build` takes the pairs to ignore -- the same
allowed-collision set a real cell keeps beside its geometry -- and the example
model ships with the three wrist pairs already excluded. The test that matters
most in the file is the dull one asserting the example arm reports positive
clearance while standing still.

An exclusion naming a link that does not exist is **refused**, not dropped.
Somebody wrote it meaning something, and a model that silently ignores it
believes it is safer than it is.

## Decision 3: self-clearance and obstacle clearance are reported apart

`clearance()` returns the closest pair overall, and beside it `to_self` and
`to_obstacles`.

Returning only the overall minimum was the first design, and the tests killed
it. A straight arm's own links sit a fixed distance apart -- **0.22 m** on the
example, between the upper arm and the wrist -- so the overall minimum reports
0.22 no matter where an obstacle is, until the obstacle comes nearer than the
arm is to itself. An obstacle 30 cm away is then completely invisible to
somebody asking about obstacles.

That is not a bug in the arithmetic; the overall minimum was exactly the overall
minimum. It is a question answered so literally that it stopped being useful.
The two numbers cost one extra comparison each and answer the two questions
people actually ask.

## Decision 4: distances are signed

Negative means overlapping, by the depth of the overlap, rather than clamped at
zero. "Touching" and "driven 40 mm through" call for different responses, and a
caller who only wants a yes or no compares against zero.

When there is genuinely nothing to measure -- no obstacles, every link pair
adjacent or excluded -- the answer is `kNothingNear`, a large finite number.
Infinity is the honest answer and a terrible one to hand back, because it
survives arithmetic that should have failed and reappears as a NaN much later,
somewhere else.

## Consequences

- A clearance query costs **432 ns** for six links, three obstacles and nine
  self pairs -- twenty-seven capsule distances, 0.043% of a millisecond cycle,
  allocating nothing. It can therefore run as a gate on every setpoint rather
  than as a planning-time check, which is the difference between catching a bad
  move and catching a bad *command*.
- The model is per-configuration. Nothing here checks the swept volume between
  two configurations, so a step large enough to pass through a thin obstacle
  passes through it undetected. Following a plan means sampling it densely
  enough, and a test does exactly that along a Cartesian move.
- Six links of capsules is a coarse robot. It will refuse poses a real machine
  could hold. That is stated rather than hidden, and the fix is more capsules.
- Nothing is wired into `CartesianPlan` automatically. Planning does not take
  obstacles and does not report clearance, because the obstacle set changes far
  more often than the arm does and baking it into the planner would mean
  replanning to answer a question about a fence that moved.

## Alternatives considered

**Spheres only.** Simpler still -- distance between two spheres is one
subtraction -- and a link needs five or six of them to be covered as well as one
capsule covers it. The segment-segment distance is thirty lines and pays for
itself immediately.

**A full mesh with a broad phase.** What a production cell uses, and correct in
a way capsules never are. It is also a bounding-volume hierarchy, a mesh loader
and a format decision, none of which this library has any business owning.

**Reporting every colliding pair rather than the closest.** A caller deciding
whether to move needs one number. A caller diagnosing why gets the pair that
produced it, which in practice is the one being argued about anyway.

**Checking swept volumes between configurations.** The honest fix for the gap
named above, and it needs a continuous-collision formulation -- conservative
advancement, or a bound on how far the geometry moves per unit of path. Both are
larger than this whole file.
