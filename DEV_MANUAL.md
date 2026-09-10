# SysNav Development Manual

## Bug log

### 2026-09-09 — semantic object merge index overflow

**Symptom**

`semantic_mapping_node` exited while optimizing persistent objects:

```text
IndexError: list index out of range
single_obj = self.single_obj_list[i]
```

Once the node exited, YOLO detections could no longer become persistent object candidates, so target VLM verification and planner takeover were not reached.

**Root cause**

The same-class merge loop removed `target_obj_same` and then decremented `i`. When a merge happened at `i == 0`, `i` became negative. Python accepted the negative index temporarily, but the index-based self check no longer recognized the current object. This allowed self-merges and repeated list shrinkage until the negative index exceeded the remaining list length.

**Fix**

- Keep `i` unchanged after removing a merged object, so the loop continues from a valid non-negative position.
- Exclude the current object by identity (`curr_obj is single_obj`) instead of comparing its list index.
- Object association thresholds and merge criteria were not changed.

**Validation**

Focused regression:

- Four overlapping dummy objects merged into one object.
- No self-merge occurred.
- No `IndexError` occurred.

Simulation episode `semantic-merge-fix-episode-20260909-02`:

- Fixed start: `(0.0, 0.0, 0.0)`.
- Target: `toilet`; absent from the initial rendered panorama.
- Duration: `300.035 s`; autonomous travel: `25.493 m`.
- YOLO first detected the target at `82.614 s`.
- The observer received `461` `ObjectNodeList` messages.
- At least one same-class merge executed in the live semantic mapper.
- `semantic_mapping_node` stayed alive until controlled shutdown.
- No negative-index `IndexError` or duplicate-ID self-merge was observed.

After the episode completed, the controlled `SIGINT` shutdown reported exit code 1 because
`rclpy.shutdown()` was called after the ROS context had already shut down. This happened only
during process teardown and is separate from the runtime merge-index bug.

Evidence is stored locally in:

- `recordings/semantic-merge-fix-episode-20260909-02/objnav.jsonl`
- `recordings/semantic-merge-fix-episode-20260909-02/metrics.json`
- `recordings/semantic-merge-fix-episode-20260909-02/visibility.json`
- `recordings/semantic-merge-fix-episode-20260909-02/debug.log`

The episode did not submit the detected toilet to target-object VLM verification and timed out without planner takeover. That is a separate candidate-promotion issue; it is not treated as part of this merge-index fix.
