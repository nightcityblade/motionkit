# Review checklist

Not a list to tick through in order. It is the set of questions that have
actually caught something in this repository, kept so they get asked again.

Generic checklists ("is the code readable?") produce generic reviews. Every
item below names a specific way this codebase has been wrong.

## Numerical

- **Is the singular case tested?** Gimbal lock at pitch = ±π/2, a rotation of
  exactly π where the axis flips sign, a zero-length vector, a chain at a
  kinematic singularity. Nearly every interesting bug here has lived at one of
  these. The example 6R arm's *zero configuration* is singular, which is a
  useful default seed for such a test.
- **Does a tolerance have units, and are they the caller's?** `isApprox` on a
  pose takes two tolerances, one linear and one angular, because adding a
  distance to an angle requires a length scale that belongs to the robot and
  not to the comparison. Any single-number pose metric has picked one silently.
- **Is a difference of large numbers hiding a cancellation?** ADR-0003 exists
  because the obvious rotation metric loses seven orders of magnitude near
  identity, which is exactly where accuracy matters.
- **Is the result compared against something independent?** The Jacobian is
  checked against central differences of `forward()`. A test that compares an
  implementation to itself confirms determinism, not correctness.

## Realtime

- **Does anything on the cyclic path allocate, lock or throw?** If a new
  function is callable from the 1 kHz task, it needs an entry in
  `tests/test_no_allocation.cpp`.
- **Could this test pass while inert?** The allocation counter has a positive
  control (`TheAllocationCounterItselfWorks`) because a counter that never
  increments passes silently. Any test asserting an absence needs one.
- **Is unbounded work hiding behind a bound?** An iterative solve has
  `max_iterations`; a loop over a container has whatever the caller put in it.

## Interfaces

- **Can the absence of information be read as permission?** This is the
  recurring one. `MotionLimits` defaults to zero and `validate()` rejects zero,
  so an unconfigured limit set means "may not move". `RevoluteJoint` limits
  default *wide*, because a joint range of zero would mean an unposable chain,
  and that failure is not safer. Both are deliberate and opposite; a new
  default needs the same argument made explicitly.
- **What does a failure return, and can it be mistaken for a result?**
  `Expected` default-constructs its value on failure rather than leaving it
  indeterminate. A partially filled result is the failure mode the type exists
  to prevent.
- **Does a returned pose say which frame it is in?** If the name does not
  carry it (`base_T_tool`), the doc comment must.
- **Is a new public symbol a promise you want to keep?** Anything outside
  `detail/` is depended on. Moving it later is a breaking change.

## Documentation

- **Does the comment bind to the entity you think it does?** A single comment
  above two members documents the first and leaves the second blank. This has
  happened twice here — `RevoluteJoint::upper` and
  `IkReport::orientation_error` — and neither is visible when reading the
  header. The `docs` CI job is what catches it.
- **Does the commit message say why?** The diff says what. A future reader
  needs the alternative that was rejected, and the reason.
- **Does this decision need an ADR?** If someone would otherwise have to
  reverse-engineer the reasoning, yes.

## Build and CI

- **Does a new gate get shown biting?** A check that has never failed on
  purpose is a check nobody has confirmed works. ADR-0009's gate was tested by
  adding an undocumented entity and watching the build fail, on both the CI
  Doxygen version and a much newer one.
- **Does a new CI job depend on a repository setting?** If so, it should be
  inert until that setting exists rather than red on `main`. See `PUBLISH_DOCS`.
- **Does this work from a fresh clone?** An empty directory is untracked, so
  it is absent for everyone else — which is how the formatter job once failed
  while every file was correctly formatted.
