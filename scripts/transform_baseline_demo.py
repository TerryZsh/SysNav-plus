#!/usr/bin/env python3
"""Recompose a recorded baseline episode into the refined dashboard layout.

This tool never invents missing streams or events.  A missing RViz/Unity stream
is rendered as an explicit unavailable panel, and telemetry is reconstructed
only from the episode's objnav.jsonl.
"""

import argparse
import json
import math
import subprocess
from collections import Counter
from pathlib import Path

import cv2
import imageio_ffmpeg
import numpy as np


WIDTH = 2560
HEIGHT = 1440
FPS = 10.0

BACKGROUND = (14, 16, 14)
PANEL = (20, 22, 19)
TEXT = (226, 231, 228)
MUTED = (150, 157, 151)
GREEN = (78, 225, 91)
CYAN = (197, 194, 48)
AMBER = (18, 126, 188)
RED = (69, 75, 225)

PANORAMA_REGION = (24, 146, 1816, 605)
UNITY_REGION = (1864, 146, 672, 378)
STAGE_REGION = (1864, 550, 672, 201)
RVIZ_REGION = (24, 834, 1212, 582)
DASHBOARD_REGION = (1260, 834, 1276, 582)
CAPTION_REGION = (24, 770, 2512, 44)

# Clean content regions in the previously recomposed 1920x1080 toilet demo.
LEGACY_PANORAMA = (24, 138, 1296, 429)
LEGACY_UNITY = (1360, 138, 536, 401)
LEGACY_RVIZ = (24, 632, 540, 448)


def draw_text(canvas, value, origin, scale=0.55, color=TEXT, thickness=1):
    cv2.putText(
        canvas,
        str(value),
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def ellipsize(value, limit):
    value = " ".join(str(value).replace("\n", " ").split())
    return value if len(value) <= limit else value[: max(0, limit - 3)] + "..."


def crop(frame, region):
    x, y, width, height = region
    return frame[y : y + height, x : x + width]


def fit(canvas, image, region):
    x, y, width, height = region
    if image is None or image.size == 0:
        unavailable(canvas, region, "SOURCE STREAM NOT RECORDED")
        return
    scale = min(width / image.shape[1], height / image.shape[0])
    resized_width = max(1, round(image.shape[1] * scale))
    resized_height = max(1, round(image.shape[0] * scale))
    interpolation = cv2.INTER_CUBIC if scale > 1 else cv2.INTER_AREA
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=interpolation)
    offset_x = x + (width - resized_width) // 2
    offset_y = y + (height - resized_height) // 2
    canvas[offset_y : offset_y + resized_height, offset_x : offset_x + resized_width] = resized


def unavailable(canvas, region, message):
    x, y, width, height = region
    canvas[y : y + height, x : x + width] = PANEL
    cv2.rectangle(canvas, (x, y), (x + width - 1, y + height - 1), (45, 49, 45), 2)
    for offset in range(-height, width, 80):
        cv2.line(canvas, (x + max(offset, 0), y + max(-offset, 0)),
                 (x + min(width, offset + height), y + min(height, height + offset)),
                 (28, 31, 28), 1)
    draw_text(canvas, message, (x + 30, y + height // 2), 0.68, MUTED, 2)
    draw_text(canvas, "Cannot be recovered from the flattened baseline video",
              (x + 30, y + height // 2 + 38), 0.46, MUTED, 1)


class Timeline:
    def __init__(self, path):
        self.events = []
        for line in path.read_text(errors="replace").splitlines():
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            elapsed = event.get("elapsed_s")
            if isinstance(elapsed, (int, float)):
                self.events.append(event)
        self.events.sort(key=lambda item: item["elapsed_s"])
        start = next((event for event in self.events if event.get("event") == "episode_start"), {})
        self.instruction = start.get("instruction", "Unknown instruction")
        self.target = start.get("target_object", "unknown")

    def through(self, elapsed):
        return [event for event in self.events if event["elapsed_s"] <= elapsed]

    def state(self, elapsed):
        events = self.through(elapsed)
        names = [event.get("event") for event in events]
        stage = "EXPLORING"
        detail = "Searching the mapped environment"
        color = CYAN
        if "yolo_target_detected" in names:
            stage, detail = "TARGET DETECTED", "YOLO observed the requested category"
        if "semantic_projection_ready" in names:
            stage, detail = "SEMANTIC PROJECTION", "Mask and 3D projection available"
        if "object_memory_associated" in names:
            stage, detail = "OBJECT MEMORY", "Detection associated with long-term memory"
        submitted = [event for event in events if event.get("event") == "vlm_submitted"]
        results = [event for event in events if event.get("event") == "vlm_result"]
        if submitted and len(results) < len(submitted):
            stage, detail = "WAITING FOR VLM", "A recorded baseline request is in flight"
        accepted = [event for event in results if event.get("output", {}).get("accepted")]
        if accepted:
            stage, detail, color = "TARGET CONFIRMED", "VLM accepted a target candidate", GREEN
        if "planner_input_sent" in names:
            stage, detail, color = "TARGET NAVIGATION", "Confirmed target sent to planner", GREEN
        ended = [event for event in events if event.get("event") == "episode_end"]
        if ended:
            final = ended[-1]
            if final.get("success"):
                stage, detail, color = "GOAL REACHED", "Baseline episode reported successful arrival", GREEN
            else:
                stage = "EPISODE " + str(final.get("stop_reason", "ENDED")).upper()
                detail = "Baseline outcome preserved from the original log"
                color = RED
        return stage, detail, color, submitted, results, events

    def caption(self, elapsed):
        recent = [
            event for event in self.events
            if event.get("event") == "vlm_result" and 0 <= elapsed - event["elapsed_s"] <= 3.0
        ]
        if not recent:
            return None
        event = recent[-1]
        output = event.get("output", {})
        result = output.get("is_target", output.get("accepted", "unknown"))
        return (
            f"VLM OUTPUT RECEIVED  |  request={event.get('request_id')}  |  "
            f"object={event.get('object_id')}  |  is_target={result}"
        )


def source_views(frame, mode):
    if mode == "panorama-only":
        return frame, None, None
    return crop(frame, LEGACY_PANORAMA), crop(frame, LEGACY_RVIZ), crop(frame, LEGACY_UNITY)


def render_stage(canvas, timeline, elapsed):
    stage, detail, color, submitted, results, events = timeline.state(elapsed)
    x, y, width, height = STAGE_REGION
    canvas[y : y + height, x : x + width] = PANEL
    draw_text(canvas, "CURRENT PIPELINE STAGE", (x + 24, y + 32), 0.48, MUTED)
    cv2.circle(canvas, (x + 31, y + 78), 9, color, -1)
    draw_text(canvas, stage, (x + 51, y + 87), 0.68, color, 2)
    draw_text(canvas, ellipsize(detail, 72), (x + 24, y + 126), 0.46, TEXT)
    unique_objects = {
        event.get("object_id") for event in events
        if event.get("event") == "object_memory_associated" and event.get("object_id") is not None
    }
    draw_text(
        canvas,
        f"{elapsed:06.1f}s  |  target: {timeline.target}  |  target memory nodes: {len(unique_objects)}",
        (x + 24, y + height - 20),
        0.45,
        CYAN,
    )


def render_dashboard(canvas, timeline, elapsed):
    _, _, _, submitted, results, events = timeline.state(elapsed)
    x, y, width, height = DASHBOARD_REGION
    canvas[y : y + height, x : x + width] = PANEL
    draw_text(canvas, "RECONSTRUCTED BASELINE DATA / VLM I-O", (x + 24, y + 38), 0.62, TEXT, 2)
    associations = [event for event in events if event.get("event") == "object_memory_associated"]
    actions = Counter(event.get("association_action", "unknown") for event in associations)
    draw_text(
        canvas,
        f"target associations {len(associations)}  |  VLM {len(submitted)} in / {len(results)} out",
        (x + 24, y + 78),
        0.46,
        CYAN,
    )
    cv2.line(canvas, (x + 24, y + 98), (x + width - 24, y + 98), (55, 60, 55), 1)
    draw_text(canvas, "LATEST RECORDED VLM INPUT", (x + 24, y + 132), 0.48, AMBER, 2)
    if submitted:
        event = submitted[-1]
        inp = event.get("input", {})
        draw_text(canvas, ellipsize(f"request={event.get('request_id')}  object={event.get('object_id')}", 112),
                  (x + 24, y + 164), 0.43)
        draw_text(canvas, ellipsize(f"labels={inp.get('candidate_labels')}  room={inp.get('room_context')}", 120),
                  (x + 24, y + 194), 0.42)
        draw_text(canvas, ellipsize(f"prompt: {inp.get('prompt', 'not retained')}", 126),
                  (x + 24, y + 224), 0.40, MUTED)
    else:
        draw_text(canvas, "No VLM submission was recorded in this baseline episode.",
                  (x + 24, y + 174), 0.46, MUTED)
    cv2.line(canvas, (x + 24, y + 252), (x + width - 24, y + 252), (55, 60, 55), 1)
    draw_text(canvas, "LATEST RECORDED VLM OUTPUT", (x + 24, y + 288), 0.48, GREEN, 2)
    if results:
        event = results[-1]
        output = event.get("output", {})
        draw_text(canvas, ellipsize(f"request={event.get('request_id')}  object={event.get('object_id')}", 112),
                  (x + 24, y + 320), 0.43)
        draw_text(canvas, ellipsize(
            f"label={output.get('final_label')}  is_target={output.get('is_target')}  accepted={output.get('accepted')}",
            120), (x + 24, y + 350), 0.43, GREEN)
        reason = output.get("reason") or output.get("raw_output") or "No reason retained"
        draw_text(canvas, ellipsize(f"reason: {reason}", 126), (x + 24, y + 380), 0.40, MUTED)
    else:
        draw_text(canvas, "No VLM result was recorded in this baseline episode.",
                  (x + 24, y + 330), 0.46, MUTED)
    cv2.line(canvas, (x + 24, y + 426), (x + width - 24, y + 426), (55, 60, 55), 1)
    action_text = "  ".join(f"{key} {value}" for key, value in sorted(actions.items())) or "none"
    draw_text(canvas, "TARGET MEMORY ASSOCIATION", (x + 24, y + 464), 0.45, CYAN)
    draw_text(canvas, ellipsize(action_text, 120), (x + 24, y + 496), 0.43, TEXT)
    draw_text(canvas, "Telemetry is reconstructed from objnav.jsonl; absent data is not inferred.",
              (x + 24, y + height - 24), 0.40, MUTED)


def compose(frame, mode, timeline, elapsed):
    panorama, rviz, unity = source_views(frame, mode)
    canvas = np.full((HEIGHT, WIDTH, 3), BACKGROUND, dtype=np.uint8)
    draw_text(canvas, "SYSNAV  /  BASELINE EPISODE", (24, 42), 0.86, TEXT, 2)
    draw_text(canvas, f"Task:  {timeline.instruction}", (24, 77), 0.48, TEXT)
    draw_text(canvas, f"{elapsed:06.1f}s  |  OFFLINE FORMAT TRANSFORM", (2180, 42), 0.44, CYAN)
    draw_text(canvas, "01  YOLO PANORAMA / INSTANCE MASKS", (24, 129), 0.50, TEXT)
    draw_text(canvas, "02  UNITY / THIRD PERSON", (1864, 129), 0.50, TEXT)
    draw_text(canvas, "03  RVIZ / SPATIAL + OBJECT MEMORY GROWTH", (24, 817), 0.50, TEXT)
    draw_text(canvas, "04  PIPELINE TELEMETRY", (1260, 817), 0.50, TEXT)
    fit(canvas, panorama, PANORAMA_REGION)
    fit(canvas, unity, UNITY_REGION)
    fit(canvas, rviz, RVIZ_REGION)
    render_stage(canvas, timeline, elapsed)
    render_dashboard(canvas, timeline, elapsed)
    caption = timeline.caption(elapsed)
    if caption:
        x, y, width, height = CAPTION_REGION
        canvas[y : y + height, x : x + width] = AMBER
        draw_text(canvas, ellipsize(caption, 170), (x + 20, y + 30), 0.52, (255, 255, 255), 2)
    else:
        x, y, width, height = CAPTION_REGION
        canvas[y : y + height, x : x + width] = (25, 28, 25)
        draw_text(canvas, "BASELINE TRANSFORM  |  missing source streams and events are shown explicitly",
                  (x + 20, y + 29), 0.46, MUTED)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--events", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--preview", required=True, type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--mode", required=True, choices=("panorama-only", "legacy-dashboard"))
    args = parser.parse_args()

    capture = cv2.VideoCapture(str(args.input))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {args.input}")
    source_fps = capture.get(cv2.CAP_PROP_FPS)
    source_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    source_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    source_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if source_fps <= 0 or source_frames <= 0:
        raise RuntimeError("Input video has invalid FPS or frame count")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.preview.parent.mkdir(parents=True, exist_ok=True)
    encoder = subprocess.Popen(
        [
            imageio_ffmpeg.get_ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{WIDTH}x{HEIGHT}",
            "-r", str(FPS), "-i", "pipe:0", "-an", "-c:v", "libx264",
            "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(args.output),
        ],
        stdin=subprocess.PIPE,
    )
    timeline = Timeline(args.events)
    last_output_index = -1
    output_frames = 0
    last = None
    try:
        for source_index in range(source_frames):
            ok, frame = capture.read()
            if not ok:
                break
            elapsed = source_index / source_fps
            output_index = int(math.floor(elapsed * FPS + 1e-9))
            if output_index <= last_output_index:
                continue
            last = compose(frame, args.mode, timeline, elapsed)
            encoder.stdin.write(last.tobytes())
            last_output_index = output_index
            output_frames += 1
    finally:
        capture.release()
        encoder.stdin.close()
        return_code = encoder.wait(timeout=max(60, math.ceil(source_frames / source_fps)))
    if return_code != 0 or not output_frames or last is None:
        raise RuntimeError(f"Encoding failed: return_code={return_code}, frames={output_frames}")
    if not cv2.imwrite(str(args.preview), last):
        raise RuntimeError(f"Cannot write preview {args.preview}")

    metadata = {
        "format": "refined-dashboard-v1",
        "transformation": "offline re-layout from flattened baseline video and objnav.jsonl",
        "source_video": str(args.input),
        "source_events": str(args.events),
        "source_resolution": [source_width, source_height],
        "source_fps": source_fps,
        "output_resolution": [WIDTH, HEIGHT],
        "output_fps": FPS,
        "output_frames": output_frames,
        "panorama_available": True,
        "rviz_available": args.mode == "legacy-dashboard",
        "unity_available": args.mode == "legacy-dashboard",
        "telemetry_reconstructed_from_log": True,
        "missing_data_inferred": False,
    }
    args.metadata.write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata))


if __name__ == "__main__":
    main()
