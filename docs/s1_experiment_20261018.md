# S1 native ExactTime experiment — 2026-10-18

## Scope and baseline

- Worktree: `slc_s1_codex_20261018`, branch `feat/slc-s1-exact-sync-gate-20261018`.
- Baseline and merge base: `c389bb3ea5ce8b9d403b2f8634bdd146858875f0`.
- Inspected the node source/header, FAST-LIVO2 contract, launch/configuration contract, CMake/package dependencies, and all existing tests before editing.
- Inspected `slc_fsync_exact128_20261018` and `slc_fsync_approx128_20261018` read-only. Reused the native `message_filters` test-source pattern and the default-50/configured-128 distinction. The minimum remains 1; the independently documented 1024 ceiling is a memory-safety guard, not a tuning claim.
- No FAST-LIVO2, kernel, system QoS, other worktree, remote, replay, or GUI changes. No commit, push, or merge.

## File diff

- `include/simple_loop_closure/simple_loop_closure_node.hpp`: replace `ApproximateTime` with native `ExactTime`; add the queue-size member.
- `src/simple_loop_closure_node.cpp`: declare/read `sync_queue_size` (code default 50), validate before conversion, construct `ExactTime`, and remove the approximate-policy interval setter. The synchronized callback body is byte-identical to `c389bb3` (SHA-256 `f11db4babdf63c6e8dcaf7d7a20e1c45a779d5493d5db9119b80944560ca5adb`).
- `include/simple_loop_closure/math_safety.hpp`: accept queue sizes in `[1, 1024]` and reject all other values before constructing the synchronizer. The upper bound is an operational memory-safety guard, not an empirically optimal matching depth.
- `config/simple_loop_closure.yaml`: set 128 as an explicitly experimental runtime-arrival buffer. This is not a claim that 128 is optimal.
- `test/test_exact_time_sync.cpp`: native-policy tests for exact pairing, out-of-order arrivals, unmatched stamps, bounded-queue eviction, same-stamp pending replacement, purge of older pending tuples after a newer completion, recreated older-pair reverse emission, repeated complete same-stamp emission, and the explicitly ordered delayed-stream boundary `L > Q` versus `Q >= L`.
- `test/test_keyframe_gate.cpp`: queue validation boundary tests.
- `test/test_launch_contract.py`: launch/config/source contract for ExactTime, validation-before-construction, default 50, experimental 128, and absence of approximate slop.
- `CMakeLists.txt`: build the new native-policy test.

## Correctness conditions

1. A callback is possible only when cloud and odometry `header.stamp` values are exactly equal at nanosecond resolution. No timestamp is shifted, rounded, synthesized, or tolerance-matched.
2. `sync_queue_size` must be in `[1, 1024]` before conversion to the ROS 2 Humble `message_filters::sync_policies::ExactTime` constructor type. Native ExactTime treats `Q = 0` as unbounded, but the S1 validator rejects zero. The validated upper bound `kMaxSyncQueueSize = 1024` prevents type-valid but operationally dangerous allocations.
3. The in-code default remains 50. YAML value 128 is only the S1 experiment setting and requires runtime evidence before any tuning conclusion; it is not a blindly chosen certified setting.
4. Native Humble ExactTime stores incomplete tuples in a map keyed by exact timestamp. When capacity is exceeded it evicts the smallest unmatched timestamp. A same-topic duplicate replaces the message in that pending tuple slot. After a complete tuple at `T` invokes the callback, the policy erases `T` and purges every then-pending tuple with stamp `<= T`.
5. That purge does not establish a permanent lower timestamp bound: a complete older tuple recreated after `T` can still invoke the callback, producing a reversed timestamp sequence. Likewise, after a complete tuple is erased, another fully supplied tuple at the same stamp can invoke the callback again. The native policy holds its ExactTime mutex while invoking the user callback.
6. For the test's fully specified stream order, every cloud and odometry stamp `1..N` is injected exactly once, cloud `s` arrives immediately before odometry `s+L`, and the final clouds are drained without newer odometry. Under that model, `Q < L` loses pairs through oldest-tuple eviction, whereas `Q >= L` pairs all inputs. This is a result for that controlled arrival sequence, not a general queue-sizing claim.
7. Existing FAST-LIVO2 frame/schema/pose/exact-stamp checks remain as a fail-closed second gate. Existing keyframe selection, `/pgo_odom` publication, audit stages, loop scheduling, and PGO code are unchanged.
8. The legacy `time_stamp_tolerance` parameter remains declared and validated to preserve the existing F0/F2 parameter contract, but it no longer affects synchronization under ExactTime.

The supplied CBD03 SQLite premise (2660 unique, monotonically increasing cloud stamps exactly matching 2660 odometry stamps) satisfies the timestamp condition. S1 addresses only the runtime arrival buffer and pairing policy; it does not independently re-read or alter the dataset.

## Bounded isolated validation

All artifacts are under `.codex_native/s1/`; no shared build/install/log directory was used.

- Build (600 s bound): 1 package finished in 1m00s; node plus 6 native test executables built successfully. Log: `.codex_native/s1/log/build_2026-10-18_13-32-54/`.
- Test (300 s bound): 10/10 CTest targets passed in 31.6 s; `colcon test-result` reports 95 tests, 0 errors, 0 failures, 0 skipped. The ExactTime target passed 5/5 cases; launch contract passed 12/12. Log: `.codex_native/s1/test_log/test_2026-10-18_13-34-00/`.
- Supervisor safety-cap follow-up: incremental node, `test_keyframe_gate`, and `test_exact_time_sync` targets built successfully. Focused tests passed 33/33: keyframe/math 16/16 (including the single queue-validation case with all 7 requested boundaries: 0, 1, 50, 128, 1024, 1025, and `UINT32_MAX`), ExactTime 5/5, and launch contract 12/12.
- Supervisor ExactTime-semantics follow-up: isolated incremental build completed successfully in 1m00s; only `test_exact_time_sync.cpp` recompiled and all other native targets were up to date. The focused ExactTime executable passed 9/9 tests. The subsequent full package run passed 10/10 CTest targets in 1.52 s of CTest time; the result files contain 89 framework test cases, and `colcon test-result` reports 99 aggregate records when its 10 CTest wrapper records are included, with 0 errors, 0 failures, and 0 skipped. After a comment-only correction, the final ExactTime target rebuilt and all 10/10 CTest targets passed again in 1.32 s. Build log: `.codex_native/s1/log/build_2026-10-18_13-42-45/`. Test log: `.codex_native/s1/test_log/test_2026-10-18_13-43-55/`.
- `git diff --check`: clean.
- Non-functional environment notes: `clang-format` was unavailable; manual diff inspection and the configured `-Wall -Wextra -Wpedantic` build found no compile warning. Colcon's optional desktop notification could not access the session D-Bus after successful build/test completion.

## Risks and next evidence

- Queue capacity trades memory for tolerated cross-topic arrival skew. At the measured 1.099 MiB per cloud, 1024 pending clouds account for about 1.1 GiB and 128 for about 141 MiB, before odometry, container, allocator, and other process overhead. Therefore 1024 is only a fail-fast memory-safety ceiling. Likewise, 128 is a bounded experimental choice, not a blindly chosen certified or empirically optimal matching depth; replay evidence would be needed to relate depth to observed cross-topic arrival skew. If one counterpart arrives after its incomplete tuple has been evicted, ExactTime cannot recover that pair. The native policy also clears incomplete timestamps not newer than the last emitted pair, so per-topic timestamp reordering would remain lossy.
- Same-stamp duplicates are not FIFO: a duplicate replaces the pending message in that topic slot, while two separately completed same-stamp tuples each emit. CBD03's stated stamp uniqueness avoids both ambiguities.
- ExactTime itself does not guarantee monotonic callback order. The current S1 candidate would accept and process a fully recreated older exact pair after a newer pair, so an adversarial or reordered live stream could drive `/pgo_odom` and PGO inputs backward in timestamp. A monotonic fail-closed gate is explicitly required before treating such input as safe, but production logic remains unchanged until end-to-end evidence defines the correct placement, accounting, and operational response.
- The synchronized node callback runs while the native ExactTime mutex is held. Callback latency therefore blocks further additions to that synchronizer; the tests document policy semantics only and make no throughput or concurrency guarantee.
- Values above 1024, including `UINT32_MAX`, are rejected before synchronizer allocation to prevent obvious host-memory exhaustion. Deployments must still budget the memory allowed by the cap.
- No rosbag replay was performed by instruction. A later supervised, bounded CBD03 replay should compare ingress and synchronized stamp counts/digests and queue-loss evidence before accepting 128 or choosing another value.

Fatal blockers: none.
