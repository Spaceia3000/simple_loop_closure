# C1 external completeness certification gate — 2026-10-18

## Purpose

`scripts/run_slc_transport_diagnostic.py` retains its previous diagnostic
behavior unless a new certification option is selected. The C1 external gate
is intentionally stricter than backend completion: successful native publish
accounting does not prove delivery to either independent DDS reader.

Use either:

```bash
--require-external-stamp-integrity
```

to require external stamp integrity only, or:

```bash
--certify-all
```

to require both external stamp integrity and the existing native backend
audits, plus the per-bag full synchronized-source contract described below.
`--require-backend-audits` remains available and retains its prior
meaning. The standalone `audit` command and partial audits do not opt into the
full-source assertion.

There is no dataset-specific frame-count constant. `--certify-all` derives the
required sequence and count from the two original source topics read directly
from the source bag's SQLite rows. An independent expected count can be added
explicitly with `--expected-source-count POSITIVE_INTEGER`; it is omitted by
default. A mismatch fails before replay.

## Exact acceptance contract

The report reads each artifact independently and compares header-stamp
sequences, preserving duplicate multiplicity. C1 external integrity is `PASS`
only when every comparison below has an identical ordered sequence, an
identical multiset, zero missing stamps, and zero extra stamps:

| Stage | Streams |
|---|---|
| source SQLite → witness | cloud and odometry |
| source SQLite → recorded SQLite | cloud and odometry |
| PGO witness → recorded SQLite | `/pgo_odom` |

For a completed `--certify-all` replay, three additional comparisons and a
validated source inventory are mandatory:

| Full-source stage | Required result |
|---|---|
| source cloud → source odometry | identical ordered sequence and duplicate multiset |
| source cloud → PGO witness | identical ordered sequence and duplicate multiset |
| source cloud → recorded PGO | identical ordered sequence and duplicate multiset |
| source cloud and odometry inventory | each is nonempty |
| source cloud → source odometry | identical nonempty count, order, multiplicity, and timestamps |
| source uniqueness | every source stamp is unique on both topics |
| optional independent expected count | both source topics equal `--expected-source-count` |
| verifiable rosbag metadata | each per-topic metadata count equals its SQLite row count |

This supports every full synchronized source bag, including LIO and LIVO bags
with different frame counts. The selected cloud and odometry topics must be
one-to-one, unique, and have the same ordered header-stamp sequence. Under that
precondition, every full replay must produce and externally expose exactly the
per-bag count derived from SQLite. Do not use the strict direct source-to-PGO
requirement for asynchronous, filtered, partial, or otherwise non-one-to-one
source datasets; use the existing audit behavior instead.

The source→recorded comparisons do not use the witness as an intermediate
baseline. Consequently, independent reader losses cannot cancel or be hidden
by equal counts. Reordered stamps, changed stamps at equal count, and duplicate
multiplicity changes all fail.

The PGO witness and recorded PGO are each compared independently with the
original SQLite source sequence. The direct source-to-PGO comparisons do not derive their expected sequence
from the two witness input streams. Therefore, if both witness inputs omit the
same source frame and both PGO readers omit it too, certification still fails.
PGO witness-to-recorder comparison remains as an independent additional stage.
An equal-count replacement, any duplicate multiplicity difference, or any
order difference also fails.

The additive `external_stamp_integrity` object in `loss_report.json` contains
the labeled stages, the full-source precondition state, a `PASS`/`FAIL` status,
and explicit discrepancies.
Existing loss-report keys remain present. If the gate is requested and any
comparison fails, `run_status.completed=false` and both run-status and report
audit evidence set `pass_eligible=false`; the artifact audit remains
`scope=complete` when all artifacts were readable. An aborted replay retains
partial-audit semantics. An unreadable/malformed witness or SQLite artifact
cannot create a passing verdict. JSON evidence replacement is atomic, so an
I/O failure while promoting a successful certification retains the previous
fail-closed report rather than a partially written PASS.

Comparison evidence is validated structurally (counts, stamp lists, booleans,
and accounting consistency). Missing or malformed fields produce `FAIL`
evidence; production validation does not depend on Python `assert`, so running
with `python -O` cannot remove these checks.

The manifest records the diagnostic schema version, implementation git commit
when available, selected witness QoS profile, and the existing publisher and
recorder QoS selections.

## Historical CBD03 LIO evidence

The three read-only historical `loss_report.json` witnesses were used as
regression evidence:

| Run | Witness cloud | Recorder cloud | PGO witness/recorder | External result |
|---|---:|---:|---:|---|
| `slc_s2_reader100_cbd03_lio_full_20261018` | 2660 | 2660 | 2660 / 2660 exact | PASS |
| `slc_s2_reader100_cbd03_lio_full_repeat2_20261018` | 2660 | 2660 | 2660 / 2660 exact | PASS |
| `slc_s2_reader100_cbd03_lio_full_repeat3_20261018` | 2637 | 2612 | 2660 / 2660 exact | FAIL |

The third replay remains backend-complete (2660 ingress/sync/publish and
passing native audits) but is not C1 externally complete. Its prior
`pass_eligible=true` describes the older opt-in policy and must not be treated
as a current V2 C1 certification.

These historical counts are evidence for those particular CBD03 LIO runs, not
defaults or acceptance constants. Unit coverage also certifies complete
synthetic source inventories of 2377 and 1351 frames, representing different
per-bag LIVO sizes.

No ROS replay is required for these deterministic tests, and the change does
not alter SLC C++, FAST, ROS middleware, or system configuration.
