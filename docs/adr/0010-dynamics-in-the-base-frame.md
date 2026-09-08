# ADR-0010: Dynamics in the base frame, and two algorithms that check each other

- **Status**: Accepted
- **Date**: 2026-09-09
- **Deciders**: Onur Can Urhan

## Context

WP-04 needs the equations of motion for a serial chain: the torque a motion
demands, and the mass matrix relating acceleration to torque. Both are standard
and both have a textbook formulation that this ADR departs from.

The library already had a claim to answer for. The README and the CMake
`DESCRIPTION` have said "kinematics, **dynamics**, trajectory generation and
calibration" since the first commit, and dynamics did not exist. That was
defensible while the work was deferred behind a deadline; without the deadline
it is a repository describing itself inaccurately, so the choice was to build it
or to amend the sentence.

## Decision 1: every quantity stays in the base frame

The classical recursive Newton-Euler formulation rotates each link's velocity,
acceleration and force into that link's own frame, propagating with a rotation
matrix at every step. It exists because it saves arithmetic: quantities stay
small and many products collapse.

Here everything — angular velocity, angular acceleration, centre-of-mass
acceleration, forces, moments and inertia tensors — is expressed in the base
frame throughout. It costs a rotation of each inertia tensor per call,
`R I R'`, which is two 3x3 products per link.

The reason is that **an intermediate value can be checked against a drawing.**
In the link-frame formulation, `omega[3]` is the angular velocity of link 3 in
link 3's frame, and a reader who wants to know whether it is right must first
reconstruct which frame that is. In this formulation it is the angular velocity
of link 3 in the frame the robot is bolted to, which is the frame the CAD model,
the measurement and the operator's intuition all already use. Debugging dynamics
is mostly checking intermediate values, and this trades arithmetic the machine
does not notice for a property the reader does.

It also follows the frame convention already set in
[ADR-0008](0008-kinematics-by-screws-and-a-damped-inverse.md): joints are
described by an axis and a point in the base frame at zero, because per-link
frames require a convention nobody can measure. Inertias are described the same
way — mass, centre of mass and inertia tensor in the base frame at the zero
configuration — which is exactly the form a CAD package reports for an assembly.

## Decision 2: gravity enters as a base acceleration

The base is given an acceleration of `-gravity` at the top of the recursion, and
no link carries a weight term.

This is not a trick to save typing; it is the equivalence principle, and it
removes a whole class of sign error. The alternative adds `m_i * g` to every
link's force balance, which is one more place per link to get a sign or a frame
wrong, and those errors produce torques that look plausible and are wrong only
in configurations nobody tested. One line at the top replaces `n` lines spread
through the loop.

It also makes an unusual case free rather than special. `setGravity` accepts any
vector, so an arm on a wall or hanging from a ceiling works without a code path
of its own — and a test asserts that reversing the field reverses every torque.
Mounting orientation is normally discovered on site, and a library that silently
assumed a floor would be wrong in a way nobody thinks to test.

## Decision 3: both CRBA and RNEA are implemented, and neither is derived from the other

The mass matrix could be obtained from the recursion already written: column `j`
is the torque of a unit acceleration on joint `j` with no gravity and no motion.
That is `n` calls to `inverseDynamics`, O(n^2), the same complexity as the
composite-rigid-body algorithm, and it would have been perhaps twenty lines.

The composite-rigid-body algorithm was written separately anyway, propagating
the momentum of each frozen distal subtree. The two share the link frames and
nothing else — one recurses on forces, the other on momentum.

**That is the entire point.** Two independent derivations of the same matrix
make each a test of the other, and the test is not a tolerance somebody chose:

    CRBA vs RNEA, worst element: 8.882e-16 kg m^2

which is about one unit in the last place. A hand-written expected matrix would
have demonstrated only that the author and the implementation multiplied the
same way. This is the same argument that put the Jacobian against numerical
differentiation in ADR-0008, and it is the strongest form of evidence available
for numerical code: agreement between two derivations that could not have made
the same mistake.

The gravity torque gets the same treatment from a third direction. It is the
gradient of the potential energy, and the potential energy is a one-line sum
over link heights sharing no code with the recursion; numerical differentiation
of one reproduces the other to 9.05e-09 N m, which is the truncation floor of a
central difference on a 1e-6 step.

## Decision 4: there is no Coriolis matrix

`inverseDynamics` evaluates `M(q) qdd + C(q, qd) qd + g(q)` in one O(n) pass
without ever forming `C`.

`C` is not unique — many matrices satisfy the equation, differing by terms that
vanish against `qd` — so a function returning "the" Coriolis matrix would have
to pick one and document which. It also costs more to build than the answer it
would help compute. It appears in textbooks because the matrix form is how the
equation is *analysed*, not how it is *evaluated*. Anyone who genuinely needs
`C qd` alone can call `inverseDynamics` with zero acceleration and subtract the
gravity torque, which is two calls and no new concept.

## Decision 5: an implausible inertia is refused, not accepted quietly

`build` rejects a body whose inertia tensor is asymmetric, has a non-positive
principal moment, or violates the triangle inequality — no principal moment may
exceed the sum of the other two.

The first two catch ordinary mistakes: a tensor filled in on one triangle only,
or built about the wrong point. The third catches the ones that pass both other
tests and still describe nothing physical, because no distribution of mass makes
one axis harder to spin than the other two together. Checking it requires the
eigenvalues, so there is a closed-form symmetric 3x3 eigensolver in the
implementation that exists solely for this.

That is real complexity for a validation check, and it is worth it because **an
implausible inertia produces plausible torques**. Nothing downstream fails. The
arm simply needs numbers that no real machine would need, in configurations
nobody happens to test, and the error is attributed to the controller. A value
that is merely wrong is worse than one that is refused.

The trade is that a legitimate idealisation is also refused. A point mass has
zero inertia about its own centre and cannot be expressed; the test fixture uses
1e-9 instead. That is the right way round — the validator cannot distinguish a
deliberate idealisation from a forgotten field, and only one of those is common.

## Consequences

- Dynamics is allocation-free and non-throwing, so it is callable from the
  cyclic task. Measured on the six-axis example: inverse dynamics **416 ns**,
  gravity torque **400 ns**, mass matrix **400 ns** — each under 0.05% of a
  1 kHz cycle. The O(n^2) mass matrix costs the same as the O(n) recursion at
  six joints, because at that size the constant factors decide.
- The base-frame formulation costs two 3x3 products per link per call to rotate
  the inertia tensor. At the measured figures this is not a cost worth
  optimising away, and the decision should be revisited only with a benchmark
  showing it matters.
- Forward dynamics — given torques, what motion follows — is **not** provided.
  It needs the mass matrix factorised and an integrator, which is a simulator's
  job rather than a controller's. `massMatrix` is the half of it that belongs
  here.
- `RigidBody` describes only rigid links. No joint friction, no motor rotor
  inertia, no gearbox, no link flexibility. On a real machine rotor inertia
  reflected through a high gear ratio is often comparable to the link inertia
  itself, so anyone using this for actual torque prediction needs to add it.
  Naming the omission is the honest alternative to a model that silently
  under-predicts.
- The base is not a body. It never accelerates, so it exerts no torque any
  joint can feel, and including it would only invite someone to give it a mass.

## Alternatives considered

**Spatial vector algebra (Featherstone).** The elegant formulation: 6D motion
and force vectors, one composite operator per joint, and the whole of RNEA in a
handful of lines. Rejected because it requires the reader to learn a second
algebra before they can check anything, and this library's existing vocabulary
is `Vec3`, `Mat3` and `SO3`. The 3D form is longer and reads as physics.

**Deriving the mass matrix from RNEA.** Twenty lines instead of sixty, and it
would have been correct. It would also have left the mass matrix with no
independent check at all — the property tests would confirm symmetry, which is
guaranteed by construction rather than by correctness.

**Accepting any inertia tensor and documenting the requirements.** The usual
choice, and it moves the failure from `build` to a torque that is quietly wrong.
