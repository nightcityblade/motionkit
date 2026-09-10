# ADR-0014: Blending cuts the corner, and says how much

- **Status**: Accepted
- **Date**: 2026-09-10
- **Deciders**: Onur Can Urhan

## Context

A sequence of Cartesian moves planned one at a time stops at every waypoint,
because each one ends at rest. On a real machine that is the difference between
a cycle that flows and a cycle that stutters, and it is visible from across the
room.

## A correction to ADR-0012 and ADR-0013

Both said blending was waiting on a profile that could start from a non-zero
state, and that is **not right**. `ReachProfile` was worth building for its own
reasons -- replanning when a target changes mid-move -- but it is not what a
known sequence of waypoints needs.

What a known sequence needs is to be planned as **one path with one profile**.
The profile still starts and ends at rest, because the whole sequence does; it
simply has no reason to stop in the middle. Nothing about that requires a
non-zero start. The earlier claim confused two different problems that happen to
share the word "blend": joining moves that are already known, and appending a
move to one already running.

## Decision 1: the corner is what gives

A tool cannot turn a sharp corner at speed. Its velocity would have to change
direction instantaneously, which is unbounded acceleration however gently the
path is paced. Something has to give, and the choice is only ever between
stopping at the waypoint and not passing exactly through it.

Each interior waypoint is replaced by a quadratic Bezier leaving the incoming
line `blend_radius` before the corner and rejoining the outgoing line the same
distance after. The waypoint is missed, and by how much is reported rather than
buried: on a route of 80 mm legs meeting at right angles, the measured miss is
**0.0141 m** and the reported figure is **0.0141 m**.

The trade is stated in both directions, and both are measured. Rounding the
corners of a three-legged staircase takes **1.44 s**; asking for
`blend_radius = 0` makes the path pass exactly through every waypoint and takes
**3.02 s**. Hitting the corners exactly costs slightly more than double.
Against planning the same waypoints as three separate moves -- **1.83 s** -- the
blended version saves 1.27x while cutting 14 mm off each corner.

Which of those a caller wants is not something this library can decide, so it
does not. Radius zero is a supported answer, not a degenerate one.

## Decision 2: a blend may take no more than half of either segment it touches

`blend_radius` is reduced at every corner to at most half the shorter of its two
adjacent segments. Two adjacent blends therefore can never reach past each other
or run through a neighbouring waypoint, however large a radius is asked for.

Without the clamp a generous radius on a short segment produces a path that
folds back on itself -- geometry that is not obviously wrong until it is drawn,
and that would be planned, paced and executed without complaint. A test asks for
a 10 m radius on 80 mm legs and checks the result is finite and still bounded by
half a leg.

## Decision 3: two deviations, reported separately

`CartesianReport` now carries `corner_deviation` beside `chord_deviation`, and
collapsing them into one number would have been a mistake.

`chord_deviation` is discretisation error. The path is solved at knots and
interpolated between them, so the tool leaves the true path a little; adding
knots shrinks it, and it is nobody's intention.

`corner_deviation` is the opposite. It is deliberate, it is the thing the caller
asked for when they set a radius, and **it does not shrink with more knots**.
Reporting one figure would let a caller add knots, watch the number fall, and
conclude the corners were being hit more closely than they were.

## Decision 4: the path parameter measures distance, except when there is none

The parameter runs on arc length, which is the natural measure and the one that
makes a blended route traverse evenly. A pure reorientation -- same position,
different orientation -- has no length at all, and a length-based parameter
would leave the tool motionless while claiming to have finished.

Such a path is measured in even shares per segment instead. That is kept in a
separate member from the geometric length and never mixed with it, because the
tempting alternative -- adding some multiple of an angle to a distance -- needs
a scale factor nobody can justify. `SE3::isApprox` refuses the same trade for
the same reason, and takes two tolerances rather than inventing one.

A waypoint repeated in the *middle* of a route is refused outright: there is no
direction to leave it by, so there is no corner to round and no honest way to
guess one.

## Consequences

- `CartesianPlan::plan` now delegates to `planThrough` with a single waypoint,
  so a straight move and a blended route share one knot solve, one pacing pass
  and one profile. A sequence that paced itself differently from a single move
  would be a second set of limits to be wrong about.
- `CartesianPlan::path()` returns a `BlendedPath` rather than a `CartesianPath`.
  `CartesianPath` remains, because a straight line is worth naming, and a route
  of two waypoints is exactly one.
- The whole sequence is still paced by its worst point, exactly as a single move
  is. A route that passes near a singularity anywhere crawls throughout. That is
  the same limitation ADR-0012 accepted and the same fix would lift it.
- Routes are capped at `kMaxWaypoints`, because the plan holds them in a fixed
  array and sampling must not allocate.
- Chosen for the tests over an L-shaped route in the horizontal plane, which
  drives the example arm's base joint to its stop. That is a correct refusal to
  a different question, and it is `JointLimitAlongPath` saying so.

## Alternatives considered

**Circular arc blends instead of Bezier.** The traditional choice, and it gives
constant curvature through the corner, which is easier to reason about. It also
needs the arc centre, the tangent points and the sweep angle computed per
corner, each with its own degenerate case as the turn approaches straight or
doubles back. A quadratic Bezier is three points and one formula, degrades
gracefully to a straight line when the turn vanishes, and the curvature it does
not control is bounded by the pacing anyway.

**Blending in joint space rather than in Cartesian space.** Cheaper, and it
avoids the geometry entirely -- but the deviation would then be unbounded in the
frame anybody cares about. "The tool passes within 14 mm of the waypoint" is a
statement a process engineer can accept or reject. "The joints pass within
0.03 rad of the waypoint configuration" is not.

**Exact arc length for the Bezier pieces.** No closed form, so it would need
numerical quadrature per corner. The control polygon bounds the arc and the
chord is bounded by it, so their average is within a fraction of a percent for
the shallow turns a blend actually makes -- and the path parameter only has to
be monotone and smooth, not metrically exact.
