## What this changes, and why

<!-- The diff shows what. Explain why, and what you considered instead.
     If this implements a decision worth keeping, it probably wants an ADR. -->

## Evidence

<!-- Numbers, not adjectives. "Sampling is allocation-free" is a claim;
     "asserted by TrajectoryRealtime.ProfileSamplingDoesNotAllocate over 1000
     calls with a positive control" is evidence.

     If you changed something with a measurable cost, give the measurement. -->

## Risk

<!-- What breaks if this is wrong, and how would anyone find out?
     "Nothing, it is a doc change" is a fine answer. -->

---

Confirm before requesting review:

- [ ] `bash scripts/format.sh --check` passes
- [ ] Tests added for the behaviour this changes — including the singular,
      empty or wrap-around case, which is where this library's bugs live
- [ ] Public API has doc comments; `--target docs` builds clean
- [ ] `CHANGELOG.md` updated under `Unreleased`, if behaviour or API changed
- [ ] New realtime-callable functions have an allocation test
- [ ] ADR added or updated, if a decision here would otherwise have to be
      reverse-engineered later
