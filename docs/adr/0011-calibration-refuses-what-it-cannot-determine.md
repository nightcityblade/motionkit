# ADR-0011: Calibration solves rotations as one linear system, and refuses what it cannot determine

- **Status**: Accepted
- **Date**: 2026-09-09
- **Deciders**: Onur Can Urhan

## Context

WP-06 needs the two calibrations every cell needs before it can do anything
useful: where the tool tip sits relative to the flange, and where a
flange-mounted camera sits relative to the flange.

Both are least-squares problems with a known closed form, and both have a
failure mode that matters more than the arithmetic. **The data can fail to
determine the answer while looking perfectly reasonable.** Six poses that touch
a point from one orientation; six stations whose every motion turns about the
same axis. Nothing about those inputs looks wrong, no solver crashes, and the
number that comes out is whatever the rounding left behind. Somebody then types
it into a controller.

This ADR is mostly about that.

## Decision 1: the hand-eye rotation is one linear system, not an average

`A X = X B` is usually split: solve the rotation, then the translation. The
rotation half is where implementations go quietly wrong, because the obvious
approach -- solve each motion pair for a rotation, then average -- requires
averaging rotations, and there is no way to do that which is both simple and
correct. Componentwise averaging of matrices leaves SO(3). Averaging Euler
angles is worse. Quaternion averaging needs an eigenvector anyway.

Written with quaternions the equation is **linear**:

    q_A * q_X = q_X * q_B    becomes    (L(q_A) - R(q_B)) q_X = 0

where `L` and `R` are the 4x4 matrices of left and right quaternion
multiplication. Every motion contributes one 4x4 block to a single homogeneous
system, and `q_X` is its null space -- a least-squares fit over all motions at
once, with no averaging step anywhere.

The null vector is the eigenvector of the smallest eigenvalue of the 4x4 normal
matrix, found by power iteration on `c I - N`, which turns the smallest
eigenvalue into the largest. `c` comes from the Gershgorin bound rather than the
trace: any upper bound works, but a loose one flattens the gap between shifted
eigenvalues and slows convergence badly.

This choice falls out of [ADR-0001](0001-quaternion-storage-for-so3.md). The
library already stores rotations as canonical unit quaternions, so the linear
form is available without converting anything, and the answer arrives already in
the representation the rest of the library uses.

## Decision 2: degenerate geometry is refused, and reported separately

`DegenerateGeometry` is a distinct error from `NotEnoughSamples`, and the
distinction is the whole point: **more measurements of the same useless kind do
not help.** Telling an operator "not enough samples" when the real problem is
that every pose had the same orientation sends them to collect twenty more
identical poses.

For the tool point the test is free. The normal-equations matrix loses rank
exactly when the orientations do not vary, so Cholesky hits a non-positive pivot
and the solve returns false. Degeneracy detection is not an extra check; it is
the factorisation refusing to proceed.

For hand-eye the test is explicit, because the classical requirement is
specific: at least two motions whose **rotation axes are not parallel**. If
every motion turns about one axis, the camera's position along that axis never
changes what it sees, and no quantity of data recovers it. `axis_spread` reports
the largest angle found, so "solvable but barely" and "not solvable" are
distinguishable -- they call for different responses from whoever is holding the
teach pendant.

Each axis is compared against the first rather than against every other. That is
not an approximation of the test that matters: the axes are all parallel exactly
when they are all parallel to the first. It decides degeneracy correctly in one
pass, and -- the reason it is worth mentioning -- without a pose history whose
length would then cap how many stations a caller may supply.

## Decision 3: residuals are part of the answer, and there are two of them

A tool tip known to a tenth of a millimetre and one known to five millimetres
are the same struct. Only the residual separates them, so a calibration that
returns a pose and nothing else is returning half a result.

Both an RMS **and** a worst case, because they answer different questions. One
bad touch among six barely moves the average and is exactly the thing worth
noticing. Measured, with a single touch displaced by 1 mm:

    residual_rms    3.633e-04 m
    residual_worst  7.951e-04 m

The RMS alone would read as "sub-millimetre, fine".

`ToolPointCalibration` also returns the touched point, which is a nuisance
parameter nobody asked for. It is solved for simultaneously and costs nothing to
return, and when it comes out somewhere impossible the fault is the fixture
rather than the tool -- a distinction that otherwise takes an afternoon.

## Decision 4: the working set does not grow with the measurements

Both solvers accumulate into fixed-size normal equations: 6x6 for the tool
point, 4x4 and 3x3 for hand-eye. Thirty stations cost the same memory as three,
and neither function allocates, which a test asserts.

This is not about deadlines -- calibration is a setup procedure and nobody runs
it in a control loop. It is about the accumulate-then-solve shape being the
right one. A solver that builds an `n`-row design matrix would have the same
answer and a working set proportional to how patient the operator was.

## Consequences

- Noise-free synthetic data is recovered essentially exactly: the tool point to
  **1.2e-16 m**, the camera mounting to **2.6e-15 rad** and **3.4e-16 m**. Those
  figures are evidence the algebra is right and evidence of nothing else.
- The test that matters more adds 0.2 mm and 0.2 mrad of deterministic wobble to
  every observation. The mounting comes back off by **3.2e-04 rad** and
  **2.7e-04 m** -- degrading in proportion rather than collapsing -- and the
  reported residuals, **2.5e-04 rad** and **3.2e-04 m**, are the same order as
  the error they are reporting. A residual that is always 1e-16 has only ever
  been shown exact data.
- There is **no nonlinear refinement**. Both solvers are single-shot linear
  least squares; neither iterates to a maximum-likelihood estimate, and neither
  models the camera. For a hand-eye result to be better than its geometry
  allows, the refinement would have to include the camera intrinsics and the
  target, which is a bundle adjustment and a different library.
- Base-frame calibration -- where the robot sits relative to the cell -- is not
  included, despite being named in the work package. It is the same shape of
  problem as hand-eye and would mostly duplicate it.
- `solveSymmetric` is a second Cholesky, alongside the one in `kinematics.cpp`.
  Duplication, and accepted: it is a textbook algorithm rather than a
  convention, so two copies cannot disagree about anything a reader must
  reconcile. That is precisely the distinction ADR-0008 draws when it refuses
  two spellings of a frame convention.

## Alternatives considered

**Tsai-Lenz for the rotation.** The classical method, and it avoids eigenvectors
entirely by solving a 3x3 linear system in a modified Rodrigues vector. Rejected
because that parameterisation is singular at 180 degrees and needs its own care,
while the quaternion form is linear everywhere and reuses a representation the
library already maintains as canonical and unit.

**Returning a best effort on degenerate data, with a warning flag.** The common
choice. It relies on every caller checking a flag on a struct that already
contains a plausible-looking pose, and the failure when they do not is a robot
confidently reaching for the wrong place.

**Requiring the caller to supply motion pairs rather than stations.** Simpler to
implement, and it pushes the pairing -- a genuine source of sign and inversion
errors -- onto the caller. What a caller actually has is a list of stations.
