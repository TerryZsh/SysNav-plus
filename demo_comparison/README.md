# SysNav baseline/current demo comparison

This directory contains standalone copies of the three baseline demos and the
three current demos. The original episode directories under `recordings/` are
left unchanged.

## Layout

```text
demo_comparison/
├── baseline/
│   ├── chair-in-living-room/
│   ├── bed-in-bedroom/
│   └── toilet/
├── current/
│   ├── chair-in-living-room/
│   ├── bed-in-bedroom/
│   └── toilet/
└── manifest.json
```

Every leaf directory contains `demo.mp4`, `demo-preview.jpg`, `metrics.json`,
`visibility.json`, and `objnav.jsonl`. Current/refined directories additionally
contain `planner-path.json`, `run-config.json`, and `validation.json`.

The three demos under `current/` use the refined 2560x1440 live-compositor
layout: YOLO/mask panorama, RViz spatial-memory growth, Unity third-person
view, pipeline stage, VLM input/output panel, and VLM-result captions. They
are copied from the validated episode directories listed in `manifest.json`;
the source recordings remain unchanged under `recordings/`.

The three demos under `baseline/` have also replaced the previous comparison
copies and use the same 2560x1440 panel geometry. Their dashboard telemetry is
reconstructed from the original `objnav.jsonl`; each directory contains a
`transformation.json` provenance record. The baseline toilet recording retains
panorama, RViz, and Unity. Because the baseline chair and bed recordings only
retained panorama pixels, their RViz and Unity panels explicitly report that
the source stream was not recorded.

## Fairness notes

- All six episodes use the same fixed spawn pose: `(x=0, y=0, z=0.75,
  yaw=0)`.
- Target categories match pairwise: chair, bed, and toilet.
- The toilet pair uses the same ground-truth target position. The z coordinate
  differs by only `0.000046134 m` because the baseline value was rounded to
  zero.
- The old chair and bed episodes did not preselect a room-qualified object
  instance (`ground_truth_target_pose` is null). They match the current runs by
  category and room instruction, but the logs cannot prove that the VLM chose
  the exact same physical instance.
- The baseline bed and toilet episodes ended by timeout; their videos are kept
  as baseline failure cases, not presented as successful arrivals.
- The baseline chair/bed videos only preserve the panorama stream, so they
  cannot be losslessly reconstructed into the refined multi-view layout.
- Baseline episode files do not embed a Git commit hash. The chair/bed mapping
  uses the existing `baseline-*` recordings; the toilet mapping uses the
  existing `toilet_original_repo_demo` recording. Exact provenance is recorded
  in `manifest.json` rather than inferred silently.
