# ADR-0016: Spending the path parameter by difficulty, and where that stops working

- **Status**: Accepted
- **Date**: 2026-09-10
- **Deciders**: Onur Can Urhan

## Context

[ADR-0012](0012-cartesian-moves-are-paced-by-one-speed.md) accepted a known
cost: a Cartesian move is driven by one jerk-limited profile on one parameter,
so its speed limit is whatever the *worst* knot on the path demands. An arm that
must crawl past one awkward stretch crawls for the entire move, however short
that stretch is.

[ADR-0015](0015-collision-checking-in-capsules.md) then explained why the
textbook fix was not taken. Full time-optimal path parameterisation optimises in
the phase plane of the path parameter and its rate, subject to velocity and
acceleration, and has **no jerk limit** -- which this library treats as the
limit that decides whether the mechanics ring.

There is a third option, and it is what this ADR is about. The conservatism
above comes from the parameter being spread evenly in *distance*, so that a
short difficult stretch and a long easy one get the same share of it. Spread the
parameter by **difficulty** instead and the single limit describes the path as a
whole. The profile stays one jerk-limited S-curve on one scalar, so nothing
about ADR-0006 is given up.

## Decision 1: reparameterise the path, do not replace the profile

The knots are solved twice. The first pass sits them evenly in `s` and exists
only to find out where the difficulty is: at each knot, the path speed the joint
limits permit. The reciprocal of that is a cost per unit of path, and its
normalised integral is the new schedule. The second pass solves the knots at the
places that answer chose.

Duration becomes the integral of `ds / smax(s)` in place of `1 / min(smax)`, and
the first can only be the smaller of the two.

Solving twice rather than interpolating the first pass keeps every knot an exact
inverse solution, which is what `chord_deviation` is measured against. It costs:
planning goes from **78 us to 149 us**.

## Decision 2: cornering is part of difficulty, and leaving it out is worse than doing nothing

The first implementation measured difficulty by the velocity limit alone. It
made every case it was tried on *slower*, including ones it should have helped.

The reason is that equalising one constraint promotes the other. With the
velocity term flat along the path, acceleration becomes the binding constraint
everywhere it was not already, and the schedule had done nothing to help it.
Measured on a blended staircase: 1.44 s evenly spread, **1.80 s** with a
velocity-only schedule. Adding the cornering term to the difficulty measure is
not a refinement; without it the whole thing is a pessimisation.

## Decision 3: it is off by default, because it is not a universal win

On a single straight move it is a large gain, and the more awkward the move the
larger:

    50 mm through a near-singular wrist    8.15 s  ->  3.27 s
    ordinary 250 mm move                   3.52 s  ->  2.23 s

On a route with blended corners it is a **loss**:

    three-legged staircase                 1.44 s  ->  2.31 s

That result is asserted by a test, so that it stays known and so that anyone
tempted to make this the default fails on the way past.

## Why blending defeats it

Worth setting out, because the mechanism is not obvious and the result looks
like a bug.

The schedule is computed from the curvature of the joint path, and then it
**changes that curvature**. Concentrating the parameter where the arm is
struggling means the parameter now varies quickly with respect to distance
there, and that variation is itself curvature -- in the new parameter, which is
the one the pacing then measures. One pass is a single step of a fixed-point
iteration that has not been shown to converge.

On a smooth path the step is small and the answer improves. At a blended corner
it is not small. Blending has already arranged for the difficulty to sit at the
corners; concentrating the parameter there as well overshoots, and manufactures
more curvature than it removed.

Damping was tried -- pulling the schedule only part of the way from even towards
difficulty. It moves the answer smoothly between the two extremes and does not
fix anything: at every setting tried, the blended route was still slower than
spreading the parameter evenly. The knob would have been one more number with no
principled value, so it is not in the interface.

## Decision 4: the cost in path accuracy is reported, not hidden

Knots serve two masters. They are where the inverse solve is exact, and they are
the resolution at which the pacing sees the path. Spending them where the arm
struggles takes them from where it does not, and the easy stretches are then
followed less exactly.

    chord_deviation, evenly spread        5.5e-05 m
    chord_deviation, by difficulty        2.6e-04 m
    the same, at 65 knots instead of 33   8.8e-05 m

So the accuracy is buyable back, at the price of more inverse solves. Reporting
the figure is what makes that a decision rather than a surprise, and it is the
same argument ADR-0014 makes for keeping `corner_deviation` and
`chord_deviation` apart.

## Consequences

- `CartesianOptions::pace_by_difficulty` defaults to false, so every existing
  measurement in this repository is unchanged by this commit. That was checked
  rather than assumed.
- `CartesianPlan::knotAt` exposes where each knot landed. A plan that spends its
  parameter unevenly is hard to reason about without being able to see where it
  spent it.
- The profile parameter is no longer the path fraction. `poseAtTime` reads it
  back through the schedule; anything else that assumed the two were the same
  would now be wrong, and the only such place was that function.
- This is **not** time-optimal path parameterisation and does not claim to be.
  TOPP is bang-bang in the phase plane and would be faster again; it also has no
  jerk limit. What is here keeps the jerk limit and takes most of the gain on
  the paths where the gain exists.
- CUDA batch inverse kinematics remains undone, for the reason ADR-0015 gives:
  GitHub's runners have no GPU, and nothing else in this library rests on a
  claim that CI cannot check.

## Alternatives considered

**Iterating the schedule to a fixed point.** The principled answer to the
convergence problem above, at one inverse-kinematics pass per iteration. It was
not taken because there is no proof it converges, the cost is linear in
iterations on an operation already at 149 us, and the failure mode of a
non-converging iteration is a plan that is worse than not trying -- which is
exactly what this ADR is documenting.

**Keeping the knots evenly spaced and reparameterising only the profile.** This
would preserve path accuracy, since the geometry would still be sampled evenly.
It needs the derivatives with respect to the new parameter to be assembled from
the even knots and the schedule together, rather than read off the knots
directly, which is a larger change to the pacing than the pacing is. Worth
revisiting if the accuracy cost turns out to matter more than the planning cost.

**Making the default depend on whether the route has corners.** It would pick
the better option in both measured cases, and it would be a default that changes
under the caller for reasons they have to go and read about. An option they set
once is easier to hold in the head than a rule that decides for them.
