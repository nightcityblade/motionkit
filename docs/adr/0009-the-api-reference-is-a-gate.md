# ADR-0009: The API reference is a gate, not an artifact

- **Status**: Accepted
- **Date**: 2026-09-09
- **Deciders**: Onur Can Urhan

## Context

WP-15 calls for a published API reference. The obvious implementation is a
Doxygen run wired to GitHub Pages, and that is what most projects do.

It is also what most projects have that nobody reads. The failure mode is well
known and does not look like failure: the reference builds, deploys and is
never wrong, because it never says anything. Entities appear with their
signature and no prose. New code lands undocumented and the site absorbs it in
silence. Six months later the reference is a searchable rendering of the header
files, which the header files already were.

The question this ADR settles is not "should there be a reference" but "what
does the build do when somebody adds a public function and writes nothing about
it".

## Decision 1: undocumented public API fails the build

`WARN_AS_ERROR = FAIL_ON_WARNINGS` with `WARN_IF_UNDOCUMENTED = YES`, run as a
required CI job on every pull request.

The setting that gives this teeth is not either of those — it is
**`EXTRACT_ALL = NO`**. With `EXTRACT_ALL = YES`, the default in most
hand-copied Doxyfiles, Doxygen emits a page for every entity whether or not
anyone documented it, and `WARN_IF_UNDOCUMENTED` goes permanently silent. The
combination produces exactly the reference described above: complete-looking
and empty. A project can have warnings-as-errors enabled and still be in that
state, which is why it is worth naming.

Turning the gate on found 86 undocumented public entities in code that had been
through eight work packages and was, by any casual reading, well commented. The
comments were there; they were on the interesting things. What was missing was
the ordinary surface — `size()`, `empty()`, `translation()`, the arithmetic
operators on `Vec3` — the parts a caller meets first.

Two of those 86 were not omissions but bugs:

- `RevoluteJoint::upper` had no comment because a single comment above `lower`
  was describing both. Doxygen attaches a comment to one entity, so the
  published reference documented the lower limit and left the upper one blank.
- `IkReport::orientation_error` had the same problem, sharing a comment with
  `position_error` that read "position error in metres, and orientation error
  in radians". The rendered page said that about `position_error` alone.

Neither is visible when reading the header, where the comment plainly covers
both lines. Both are visible in the output, and only the gate looks at the
output.

## Decision 2: require a sentence, not a form

`WARN_NO_PARAMDOC` and `WARN_IF_INCOMPLETE_DOC` are **off**.

They demand an `@param` for every parameter and an `@return` for every
non-void function, including `size()`, `empty()` and `valid()`. The only text
that satisfies that rule for `size()` is `@return The size.`, and a rule whose
compliant output is that teaches contributors that documentation is a form to
fill in. It also inflates the diff of every change, which makes reviewers skim
the documentation hunks — the opposite of the intended effect.

So the gate asks one thing: every public entity has a sentence saying what it
is for. Where a parameter carries a unit, a frame, or an ownership transfer,
`@param` earns its place and is used. Where it does not, prose is enough.

This is a weaker gate than the maximal one, and it is weaker on purpose. It is
the strongest rule that cannot be satisfied by writing nothing of value.

## Decision 3: `detail/` is excluded, and that is an interface decision

`EXCLUDE_PATTERNS = */detail/*` keeps `detail/jerk_segments.hpp` out of the
published reference.

The point is not tidiness. A published symbol is one somebody may reasonably
depend on, and the reference is where that promise is made. Excluding `detail/`
states that its contents may change without notice; including it would quietly
extend the compatibility surface to every helper. The exclusion also means the
documentation gate does not apply there, which is the correct trade — internal
helpers are read by people who can read the implementation next to them.

## Decision 4: publication is opt-in; the gate is not

The `docs` job runs on every pull request. The `publish-docs` job runs only on
`main`, and only when the repository variable `PUBLISH_DOCS` is set to `true`.

Deploying to Pages requires the Pages source to be set to "GitHub Actions", and
on a private repository it requires a plan that permits Pages at all. Neither
fact is discoverable from inside a workflow. A publish job that simply ran
would fail on a repository where nobody had made that decision, and it would
fail on `main` — a red cross on the default branch caused by a setting, not by
a commit. Requiring an explicit variable makes enabling publication a decision
somebody takes, rather than a default somebody has to notice and undo.

The gate does not wait for that variable. Documentation quality is enforced
from the first pull request; publication is a separate question about hosting.

## Consequences

- Adding a public function without a doc comment fails CI. This is the point,
  and it will be irritating on the day it first happens to someone.
- Documentation review moves into code review, because the diff now contains
  it. That is the change most likely to matter in the long run.
- The gate is tied to a Doxygen version: a new release may add a check and turn
  a green branch red for a reason unrelated to the commit. The `docs` job
  prints `doxygen --version` before building so the two causes can be told
  apart. Both the current runner version (1.9.8) and a much newer one (1.15.0)
  were verified to build this configuration clean and to fail on an
  undocumented entity, so the setup is not depending on one version's quirks.
- Doxygen is **not** required to build, test or consume the library.
  `MOTIONKIT_BUILD_DOCS` defaults to `OFF`. A contributor who never runs it is
  told by CI rather than blocked locally, and a consumer needs nothing extra.
- The reference does not include `detail/`, so a reader looking for the
  seven-segment jerk construction must read the header. That is intended.

## Alternatives considered

**A lint script that greps for `///` above public declarations.** No Doxygen
dependency, and it would have caught most of the 86. It would not have caught
either of the two shared-comment bugs, because in the source both lines *are*
preceded by a comment. Only something that parses the way Doxygen parses sees
that the comment binds to one of them.

**Documentation coverage as a reported percentage rather than a gate.** Honest,
and it makes the trend visible. It also makes every individual omission
somebody else's problem, and a number that drifts from 100% to 94% never
identifies the pull request that moved it.

**`EXTRACT_ALL = YES` with a promise to write docs.** This is the status quo
ante with extra steps, and it is what the eight preceding work packages
effectively had.
