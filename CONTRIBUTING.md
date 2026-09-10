# Contributing

## Building

Requires CMake 3.24+, Ninja, and a C++20 compiler (GCC 14 or Clang 18 upward).
Dependencies are fetched by CMake; there is nothing to install first.

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
```

Configure and build presets: `debug`, `release`, `asan`, `tsan`, `tidy`. Test
presets: `debug`, `release`, `asan`, `tsan` — `tidy` has none, because it
analyses while compiling rather than at test time. Each matches a CI job, so a
preset passing locally means that job passes.

Two caveats worth knowing before you spend an afternoon on them:

- **TSan excludes the allocation tests.** They replace global `operator new`,
  which is exactly what TSan's runtime also does. `MOTIONKIT_BUILD_ALLOCATION_TESTS`
  is off under that preset, so the count is 210 rather than 227.
- **`ScurveProfile` is rest-to-rest; `ReachProfile` is not.** Use the latter
  when the axis is already moving. `StopProfile` starts from an arbitrary state
  and has no position target, which is a third problem again.
- **A route through waypoints is one plan, not several.** `planThrough` does not
  stop in between; planning each leg with `plan` does, because each leg ends at
  rest. The cost of not stopping is that interior waypoints are missed by
  `corner_deviation`.

## Before opening a pull request

```bash
bash scripts/format.sh          # or --check to only report
cmake --preset tidy && cmake --build --preset tidy
```

The `tidy` preset sets `CMAKE_CXX_CLANG_TIDY`, so the analysis runs as part of
compiling; there is no separate test step for it.

CI runs GCC and Clang in Debug and Release, ASan/UBSan, TSan, clang-tidy,
clang-format, an installed-package consumer build, and the documentation gate.
All are required.

## The documentation gate

Every public entity needs a doc comment. Adding a public function without one
fails CI — see [ADR-0009](docs/adr/0009-the-api-reference-is-a-gate.md).

```bash
cmake -S . -B build-docs -DMOTIONKIT_BUILD_DOCS=ON
cmake --build build-docs --target docs
```

Doxygen is not needed for an ordinary build; the option defaults to `OFF`.

What the gate asks for is a **sentence saying what the entity is for**. It does
not require `@param` and `@return` on everything, deliberately: `@return The
size.` is not documentation. Use `@param` where the parameter carries a unit, a
frame, or an ownership transfer.

One trap the gate exists to catch: a comment binds to **one** entity. This
documents `lower` and leaves `upper` blank in the published reference, even
though it reads as covering both:

```cpp
/// Travel limits in radians.
Scalar lower{-6.28};
Scalar upper{6.28};
```

## Style

`.clang-format` (Google, 90 columns) and `.clang-tidy` are authoritative — run
them rather than reading this section. Conventions they cannot express:

- **Frames are named `A_T_B`**, read as "the pose of B expressed in A", so that
  `A_T_B * B_T_C` visibly cancels. A composition that does not cancel is a bug
  the reader can see.
- **Units are SI** — metres, radians, seconds — and are never converted
  silently. Say the unit in the doc comment where a number has one.
- **Failures a control loop can expect are `Expected<T, E>` values, not
  exceptions.** Exceptions are for a caller who has already broken a
  precondition, such as normalising a zero vector.
- **Anything callable from the cyclic task allocates nothing and does not
  throw**, and there is a test in `tests/test_no_allocation.cpp` asserting it.
  If you add such a function, add the test too.
- **Dependencies point down.** See the component diagram and the rules in
  [docs/architecture.md](docs/architecture.md). Siblings must not include each
  other.

## Tests

Property tests over generated inputs are preferred to hand-picked values where
the property is the real claim — that `inverse()` undoes `forward()`, that
`matrix()` is always in SO(3). Assert the property, not one case of it.

Two specific habits:

- **Test the singular case explicitly.** Most of the interesting failures in
  this library live at a singularity, a wrap-around, or a zero-length input.
- **Give a negative control to any test that could pass by being inert.** The
  allocation counter has one — `TheAllocationCounterItselfWorks` — because a
  counter that never increments passes every test silently.

## Commits and ADRs

Commit messages explain **why**, in prose. The diff already shows what changed.

A decision that a future reader would otherwise have to reverse-engineer goes
in an ADR under `docs/adr/`, numbered sequentially. Rejected alternatives are
part of the record: the value is in knowing that an option was considered and
why it lost, which is the question that gets asked eighteen months later.

Add an entry to [CHANGELOG.md](CHANGELOG.md) under `Unreleased` for anything
that changes the public API or observable behaviour.
