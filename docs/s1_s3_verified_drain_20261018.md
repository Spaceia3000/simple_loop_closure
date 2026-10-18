# S1/S3 bounded verified drain — 2026-10-18

## Scope

`scripts/run_slc_transport_diagnostic.py run` now has an optional native
completion gate. The default remains the existing fixed `--drain-sec` sleep.
No backend, frontend, DDS, kernel, sysctl, or stable-branch setting is changed.

The motivating 301-pair fixture was incomplete after a fixed 5-second drain
(280 native ingress/sync/publish records) and complete after 25 seconds. That
observation does **not** establish 25 seconds as a universal drain duration.
The verified gate does not encode 301, 5 seconds, or 25 seconds.

## Invocation

The stream audit must be enabled when the backend starts (`simple_stream_audit:
true` in the selected backend parameter file), so that the node provides the
`~/dump_stream_audit` Trigger service. With the default node name, its fully
qualified service is `/simple_loop_closure/dump_stream_audit`.

Use the gate together with the full C1 certification policy:

```bash
python3 scripts/run_slc_transport_diagnostic.py run \
  --bag /absolute/path/to/source_bag \
  --output-dir /absolute/path/to/new_output \
  --domain-id 166 \
  --subscriber-qos-profile reliable \
  --playback-rate 1.0 \
  --playback-timeout-sec 360 \
  --verified-drain \
  --verified-drain-timeout-sec 60 \
  --certify-all
```

`--verified-drain-timeout-sec` is mandatory with `--verified-drain`, must be
finite and positive, and has no implicit value. Supplying the timeout without
the gate is rejected. A namespaced/custom node can select another service with
`--stream-audit-service`.

The existing default launch configuration leaves `simple_stream_audit` false.
The runner does not silently change that parameter. If the service is absent,
disabled, blocked by a single-threaded callback, or otherwise unavailable, the
bounded gate ends in `TIMEOUT` and the run fails. It never converts service
unavailability into a fixed sleep or a passing result.

## Exact acceptance rule

Before replay, the runner reads the selected cloud and odometry rows from the
source bag's SQLite database. Verified drain requires both ordered header-stamp
sequences to be nonempty, unique, and exactly equal. It computes the same
64-bit FNV-1a digest used by `stream_audit.hpp`, including signed-int64
two's-complement bytes in little-endian order, for:

- the complete cloud sequence;
- the complete odometry sequence; and
- the ordered `(cloud, odometry)` pair sequence.

After successful playback, the runner calls the native
`std_srvs/srv/Trigger` service in separately owned, bounded `ros2 service call`
subprocesses. A snapshot is complete only when all of these invariants hold:

- cloud and odometry ingress counts equal the source cardinality and their
  ordered digests equal the corresponding source digests;
- synchronized callback, synchronized cloud, synchronized odometry, accepted
  pair, accepted cloud, and accepted odometry cardinalities all equal the
  complete source cardinality;
- synchronized and accepted pair/stream digests exactly equal their complete
  source digests;
- rejected pair count, pair stamp mismatches, and failed PGO publishes are
  zero;
- PGO publish attempted and completed counts both equal the source cardinality
  and both ordered digests equal the source odometry digest;
- the native sync and PGO count/digest accounting booleans are true; and
- the native snapshot status is `PASS`.

Any counter above the source cardinality, any nonzero rejection/failure, or an
exact-count digest mismatch is an immediate fail-closed error. Incomplete
counts remain pending only until the explicit deadline. Two consecutive,
identical complete snapshot payloads are required, preventing a single
transient snapshot from opening the cleanup gate.

Once the gate passes, the original recorder, witness, playback, and backend
cleanup order is retained. The shutdown stream audit is then checked again
against the exact verified-drain count and digest contract. `--verified-drain`
therefore also enables the existing required native final-audit validation.
For full C1 certification, `--certify-all` remains necessary because only the
post-cleanup artifact audit can prove exact source-to-witness and
source-to-recorder DDS delivery.

## Failure and evidence

The gate owns each service-call subprocess. A call that exceeds its remaining
per-call bound is killed and reaped; repeated unavailability cannot wait beyond
the global verified-drain deadline. If a managed witness, recorder, or backend
process exits while waiting, the gate fails immediately. The normal bounded
cleanup path still runs after every gate outcome.

`manifest.json` records the selected drain mode, service, and timeout.
`verified_drain.json` and `run_status.json` record `PASS`, `FAIL`, or `TIMEOUT`,
attempt counts, and the last failure observation. Timeout is always a failed
run; it is not pass-eligible even if partial recorder artifacts are readable.

No full rosbag replay was run while implementing or testing this option. The
validation used fast mocked Python tests only, with external pytest plugin
autoload disabled.
