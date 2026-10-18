# S3 `/pgo_odom` output delivery experiment — 2026-10-18

## Scope and baseline

This isolated S3 change starts from validated S1 ExactTime commit `8e175b1` and changes only the ROS 2 publisher history depth for `/pgo_odom`, plus the validation, audit reporting, tests, and documentation needed to evaluate it. It does not change FAST-LIVO2, input DDS, ExactTime synchronization, input contracts, keyframe selection, geometry, GTSAM, visualization publishers, or subscriptions.

The new integer parameter is `pgo_output_depth`. Its code and checked-in configuration default is `1`, which preserves the baseline reliable `KEEP_LAST(1)` publisher. Values outside `[1, 1024]` fail during initialization before the `/pgo_odom` publisher is created. The upper bound is an operational resource cap, not a recommended value or evidence of delivery correctness.

The configured value is passed directly to `rclcpp::QoS` for the `/pgo_odom` publisher. No other publisher or subscription uses it. F0 stream-audit version 7 obtains the publisher's actual QoS from `get_actual_qos()` and reports `pgo_publisher_qos=<reliability>_<history>_<depth>` for `KEEP_LAST`, for example `reliable_keep_last_1` or `reliable_keep_last_100`. This replaces the former hardcoded `reliable_keep_last_1` label.

## Correctness invariants

1. With `pgo_output_depth=1`, `/pgo_odom` retains the S1 reliable `KEEP_LAST(1)` QoS behavior.
2. `pgo_output_depth` is read as signed 64-bit, validated in `[1, 1024]`, and only then converted to the unsigned depth accepted by `rclcpp::QoS`; zero, negatives, and values above the cap cannot wrap into an unintended depth.
3. The parameter affects exactly one endpoint: the `nav_msgs/msg/Odometry` publisher on `/pgo_odom`.
4. The PGO message payload, input header and exact stamp, publication site, and number of publish calls are unchanged.
5. S1 native ExactTime construction and its validated `sync_queue_size` are unchanged. All input subscriptions, audit ingress taps, contract gates, keyframe logic, loop scheduling, PCL processing, and GTSAM processing are unchanged.
6. F0 `pgo_publish_attempted`, `pgo_publish_completed`, and `pgo_publish_failed` retain their original meaning. In particular, `completed` means only that the local `rclcpp::Publisher::publish` call returned normally; it does not prove DDS transmission, subscriber receipt, recorder persistence, or rosbag drain completion.
7. An F0 report states the QoS obtained from the live publisher rather than assuming depth 1. `dds_delivery_status=INCONCLUSIVE` remains mandatory because internal publisher-call accounting cannot establish end-to-end delivery.
8. Depths `10` and `100` are controlled experimental treatments. Neither is a production default or a certified optimum without the A/B evidence below.

## Proof obligations

The implementation is acceptable only if all of these obligations hold:

- Boundary tests accept `1`, `10`, `100`, and `1024`, and reject `0`, `1025`, and the signed 64-bit maximum.
- Launch/config contract tests prove the default is `1`, validation precedes publisher construction, the validated depth occurs in exactly one QoS construction, and the audit queries `get_actual_qos()`.
- Audit unit tests prove the report renders a non-default observed QoS value rather than a hardcoded depth-1 label.
- The isolated package builds and its complete test suite passes with separate build, install, and log directories.
- A source review confirms no changes to S1 synchronized callback behavior, message content, visualization publisher QoS, or input subscriptions.
- The supervisor's controlled replay demonstrates whether any treatment changes end-to-end receipt and persistence. Local tests cannot discharge this final obligation.

## Controlled depth A/B protocol

Run three treatments, `pgo_output_depth=1`, `10`, and `100`, against the same fixed 30-second bag, on the same host and ROS/RMW configuration, with the same playback command, rates, recorder command, topic selection, process startup ordering, and shutdown procedure. Use fresh output bags and logs. Do not tune any other parameter between treatments. Repeated trials are preferable; treatment order should be recorded and, if practical, rotated to expose warm-cache or resource-order effects.

For every treatment, retain:

- source cloud and odometry counts and their exact ordered timestamp sequences;
- SLC cloud/odom ingress, ExactTime callback, contract-accepted, PGO publish-attempted, publish-completed, and publish-failed counts, first/last stamps, reversals, duplicates, and FNV-64 ordered digests;
- actual `pgo_publisher_qos` from the F0 shutdown audit;
- `/pgo_odom` counts and exact ordered stamp sequences from an online witness and from the completed rosbag;
- an additional independent reader of the finalized bag, rather than relying only on recorder console output;
- wall-clock times for playback start, last input observed, last PGO witness observation, SLC audit shutdown completion, recorder stop request, recorder process exit, and independent read completion;
- middleware, recorder, storage plugin, host-load, and command-line details sufficient to reproduce the run.

Tail completion must be explicit. After the final source stamp, wait for the SLC witness/audit to reach the expected final PGO stamp and for the chosen quiet/drain interval before requesting recorder shutdown. Then wait for normal recorder exit and storage finalization before reading the bag. Apply the identical drain rule to all treatments. If a witness sees a sample that the finalized bag omits, classify that separately from a sample absent at both witnesses; do not attribute either result to publisher history without further evidence.

The primary comparison is equality of count and exact ordered stamp sequence at each boundary, not merely a terminal count. A successful `publish()` return is evidence only for the local call boundary. Missing tail messages can arise from DDS queues, subscriber execution, witness behavior, recorder queues, shutdown ordering, or storage drain, so depth alone cannot identify the cause.

## Interpretation

Promoting a non-default depth requires repeatable improvement with no sequence corruption and a documented resource tradeoff under the controlled protocol. Equal results do not prove depth irrelevance outside the tested load. Better results at depth 10 or 100 do not by themselves prove DDS delivery or justify a production default: witness-versus-finalized-bag evidence and shutdown timing must distinguish transport/subscriber loss from recorder drain loss.
