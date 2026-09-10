# ADR-0013: A move from a moving axis, found by bisection rather than by cases

- **Status**: Accepted
- **Date**: 2026-09-10
- **Deciders**: Onur Can Urhan

## Context

`ScurveProfile` starts at rest and ends at rest. That is the right shape for a
move planned in advance and the wrong one for everything else: an axis already
running cannot use it, because to follow a new target it must first stop, and
stopping in order to start again is exactly what a machine should not do between
two segments of one job.

It is also the reason nothing in this library can blend. Consecutive Cartesian
moves stop at every waypoint ([ADR-0012](0012-cartesian-moves-are-paced-by-one-speed.md)),
and the missing piece was never the blending arithmetic -- it was a profile that
can be planned from the sample the executor is holding right now, mid-move, at
speed, mid-acceleration.

`ReachProfile` is that profile: any `MotionState` to a position, arriving at
rest, with every limit respected throughout.

## Decision 1: bisection over a monotone function, not a closed-form case analysis

The move decomposes into three parts: a jerk-limited change from the starting
velocity to some cruise velocity, time spent at that cruise velocity, and a
jerk-limited change from cruise to rest. Only the cruise velocity is unknown.

The classical approach solves for it in closed form. That solution exists, and
it is roughly a dozen cases: whether the acceleration peak clips its limit on
each of the two legs independently, whether the starting acceleration is helping
or hurting, whether the move reverses, whether it reverses twice. Every one of
them is a sign convention somebody has to get right, and the ones that are wrong
are wrong only for inputs nobody happened to try.

Here the distance covered as a function of cruise velocity is **monotonically
increasing** -- a faster middle covers more ground on both legs -- so the cruise
velocity is found by bisecting that function against the distance required.
Sixty halvings reach the limit of what a double can represent.

It costs. Measured on the example limits:

    long move, closed-form branch     120 ns
    short move, bisected               1.32 us

Eleven times more, and still 0.13% of a millisecond cycle, which is the number
that makes the trade acceptable rather than merely tidy. What is bought is that
the search cannot be wrong in a case nobody thought of. There is exactly one
piece of algebra in the file -- the single velocity change -- and everything
else is that function called twice and a bracket that narrows.

## Decision 2: a state already past the acceleration limit is accepted

The same acceptance `StopProfile` makes, for the same reason: refusing to plan
for a machine that is already misbehaving has the logic backwards. What the
profile promises is not that the limits held before it was asked, but that it
never makes things worse than the state it inherited.

That promise needed a correction to hold. The velocity a ramp delivers on its
own has a closed form that assumes the acceleration peak lies **beyond** the
starting acceleration, which is true for every ordinary input and false exactly
when the axis arrives accelerating harder than the limit allows -- because then
the peak clamps to the limit and lands *below* where the axis started. The sign
of one term flips, and the formula silently returns the wrong number for the one
case the profile is documented to accept.

Found by a property sweep, not by reading: starting from velocity -1.988 with
acceleration 8.25 against a limit of 6.0, the planned move ended **1.206 units**
short of its goal. Written for any arrangement of peak and start rather than for
the usual one, the worst terminal error over two hundred starting states is
**5.2e-15**.

## Decision 3: a move that turns round is reported, not refused

An axis travelling at 2 m/s towards a goal 10 mm away cannot arrive at rest
without overshooting and coming back. One travelling away from its goal must
reverse first. Both are ordinary, both are correct, and neither is an error.

`reversed()` reports which happened, because a move that turns round is worth
noticing even when it is right -- it usually means the target changed later than
it should have. The detection is one comparison: the velocity crosses zero
exactly when the middle of the move runs against the way the axis was already
going, which covers both cases without distinguishing them.

## Decision 4: the endpoint is promised, and that makes one test worthless

Summing seven segments lands within rounding of the goal rather than on it, so
`sample()` substitutes the exact goal at and after `duration()`. An axis told to
stop a few picometres short of its target is an axis that never quite arrives,
and the substitution is the right call.

It also creates a trap, and the first version of the test file fell into it. A
test that plans a move, samples at `duration()`, and asserts the axis is at the
goal is **asking the profile what it was told to say**. It passes unconditionally.
It passed while the profile from Decision 2 was finishing 1.2 units short --
because the substitution turned that error into a position discontinuity at the
very last instant, where nothing was looking.

Two changes fix it. The arrival check now samples just *before* the end, which is
the last value the segments actually produce. And the property sweep
differentiates position and compares the result against the velocity the profile
reports -- a metric that has no opinion about endpoints and went to **673** while
the bug was present, against **3.4e-06** now.

The general lesson is the one this repository keeps relearning: a test whose
subject is allowed to define the answer is not a test. It belongs next to the
allocation counter's positive control in
[docs/review-checklist.md](../review-checklist.md).

## Consequences

- Blending is now possible. It is not yet done -- `CartesianPlan` still plans
  each move to rest, so a sequence of them still stops at every waypoint -- but
  the piece that was missing is no longer missing.
- Carrying speed through beats stopping and restarting, which is the point.
  From 1.0 m/s to a goal 3 m away: **1.82 s** against **2.22 s** for a stop
  followed by a fresh rest-to-rest move.
- From rest the profile reproduces `ScurveProfile` sample for sample --
  durations agreeing to **1.1e-16 s** and positions to **1.3e-15**. Two
  constructions that share no code and reach the same answer are better evidence
  than any hand-written expectation, and it means the general path is not paying
  for its generality with accuracy in the common case.
- The velocity limit can be exceeded, and legitimately. An axis at the velocity
  limit while still accelerating must overshoot it, because acceleration cannot
  step to zero and a further `a0^2 / 2j` of speed is already committed. The
  property test measures against what the starting state arrives owing rather
  than against the limit alone, and the worst ratio over two hundred starts is
  **1.0000**.
- Planning allocates nothing, so a controller may replan from the state it is
  holding without leaving the cyclic path.

## Alternatives considered

**Stop, then plan a fresh rest-to-rest move.** Two existing well-tested pieces,
composed in five lines, always feasible. Rejected because it is precisely the
behaviour that makes a machine look broken, and because the measurement above
puts a number on what it costs.

**Newton's method instead of bisection.** Faster convergence, and it needs the
derivative of a piecewise function whose pieces change under it. Bisection needs
only monotonicity, which is a property that can be argued in one sentence.

**Returning a best effort when the state is past the acceleration limit.**
Considered and rejected for the same reason `StopProfile` rejects it: the
profile that a machine in trouble needs is the one it gets least often.
