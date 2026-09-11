#!/usr/bin/env python3
"""Run and directly compose one complete SysNav simulation episode."""

import argparse
from collections import Counter, deque
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time


ROOT = Path('/home/myx/SysNav')
DISPLAY = ':1'
CAPTURE_SIZE = '2560x1440'
CAPTURE_WIDTH = 2560
CAPTURE_HEIGHT = 1440
SOURCE_WINDOW_WIDTH = 1280
SOURCE_WINDOW_HEIGHT = 720
FPS = 10

# Baseline-inspired information layout.  Panorama remains the largest visual,
# RViz gets the full lower-left quadrant, and Unity is intentionally compact.
PANORAMA_REGION = (24, 146, 1816, 605)
UNITY_REGION = (1864, 146, 672, 378)
STAGE_REGION = (1864, 550, 672, 201)
RVIZ_REGION = (24, 834, 1212, 582)
DASHBOARD_REGION = (1260, 834, 1276, 582)
CAPTION_REGION = (24, 770, 2512, 44)

BACKGROUND = (15, 17, 17)
PANEL_BACKGROUND = (22, 26, 26)
TEXT = (232, 236, 236)
MUTED = (145, 155, 155)
CYAN = (190, 210, 48)
GREEN = (120, 225, 68)
AMBER = (35, 188, 246)
RED = (75, 75, 245)


def load_workspace():
    if os.environ.get('SYSNAV_DEMO_ENV_READY') == '1':
        return
    os.execv(
        '/bin/bash',
        [
            'bash',
            '-c',
            'set -e\n'
            'source /opt/ros/jazzy/setup.bash\n'
            'source /home/myx/SysNav/install/setup.bash\n'
            'set -a\n'
            'source /home/myx/SysNav/config/vlm.env\n'
            'set +a\n'
            'export SYSNAV_DEMO_ENV_READY=1\n'
            'exec /home/myx/SysNav/.venv/bin/python "$@"',
            'sysnav-demo',
            str(Path(__file__).resolve()),
            *sys.argv[1:],
        ],
    )


load_workspace()

import cv2
import imageio_ffmpeg
import numpy as np
import rclpy
import sam2
import yaml
from cv_bridge import CvBridge
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tare_planner.msg import ObjectNodeList, TargetObjectInstruction


def process_alive(process):
    return process is not None and process.poll() is None


def terminate_group(process, debug):
    if not process_alive(process):
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    debug.flush()


def make_launch_wrapper(path):
    path.write_text(
        """import importlib.util
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    source = '/home/myx/SysNav/src/base_autonomy/vehicle_simulator/launch/system_simulation_with_exploration_planner.launch.py'
    spec = importlib.util.spec_from_file_location('sysnav_sim', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    for index, action in enumerate(description.entities):
        if isinstance(action, IncludeLaunchDescription) and 'goalX' in dict(action.launch_arguments):
            arguments = dict(action.launch_arguments)
            arguments['autonomyMode'] = 'true'
            description.entities[index] = IncludeLaunchDescription(
                action.launch_description_source,
                launch_arguments=arguments.items(),
            )
    return description
"""
    )


def qos_topic(value):
    return {
        'Depth': 5,
        'Durability Policy': 'Volatile',
        'History Policy': 'Keep Last',
        'Reliability Policy': 'Reliable',
        'Value': value,
    }


def marker_display(name, topic):
    return {
        'Class': 'rviz_default_plugins/MarkerArray',
        'Enabled': True,
        'Name': name,
        'Namespaces': {'Value': True},
        'Topic': qos_topic(topic),
        'Value': True,
    }


def single_marker_display(name, topic):
    return {
        'Class': 'rviz_default_plugins/Marker',
        'Enabled': True,
        'Name': name,
        'Namespaces': {'Value': True},
        'Topic': qos_topic(topic),
        'Value': True,
    }


def make_rviz_config(path):
    source = ROOT / 'src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz'
    config = yaml.safe_load(source.read_text())
    config['Panels'] = []
    displays = config['Visualization Manager']['Displays']
    # Image displays allocate persistent render panes even when disabled.  Do
    # not carry them into the demo config: the RViz stream must be one large
    # top-down map/object-memory view, not a mosaic of camera topics.
    displays[:] = [
        display for display in displays
        if display.get('Class') != 'rviz_default_plugins/Image'
    ]
    for display in displays:
        if display.get('Name') in {'OverallMap', 'Path', 'Waypoint', 'Boundary'}:
            display['Enabled'] = True
            display['Value'] = True
        if display.get('Name') == 'RegScan':
            display['Alpha'] = 0.035
        elif display.get('Name') == 'OverallMap':
            display['Alpha'] = 0.075
    displays.extend([
        {
            'Alpha': 0.8,
            'Autocompute Intensity Bounds': True,
            'Autocompute Value Bounds': {'Value': True},
            'Axis': 'Z',
            'Channel Name': 'rgb',
            'Class': 'rviz_default_plugins/PointCloud2',
            'Color Transformer': 'RGB8',
            'Decay Time': 0,
            'Enabled': True,
            'Invert Rainbow': False,
            'Name': 'Object memory points',
            'Position Transformer': 'XYZ',
            'Selectable': True,
            'Size (Pixels)': 5,
            'Size (m)': 0.05,
            'Style': 'Points',
            'Topic': qos_topic('/obj_points'),
            'Use Fixed Frame': True,
            'Use rainbow': True,
            'Value': True,
        },
        marker_display('Object memory boxes', '/obj_boxes'),
        marker_display('Object memory labels', '/obj_labels'),
        marker_display('Room object nodes', '/object_node_markers'),
        marker_display('Room boundaries', '/room_boundaries'),
        marker_display('Room type labels', '/room_type_vis'),
        marker_display('Viewpoint room IDs', '/viewpoint_room_ids'),
        single_marker_display('Chosen room boundary', '/chosen_room_boundary'),
        {
            'Class': 'rviz_default_plugins/Path',
            'Color': '255; 100; 0',
            'Enabled': True,
            'Line Style': 'Lines',
            'Line Width': 0.08,
            'Name': 'Exploration path',
            'Topic': qos_topic('/global_path'),
            'Value': True,
        },
    ])
    current_view = config['Visualization Manager']['Views']['Current']
    current_view['Distance'] = 30.0
    current_view['Pitch'] = 1.5697963237762451
    # The source RViz profile was last saved with its orbit camera focused far
    # outside this Matterport apartment.  Keep the whole explored floor and
    # object-memory overlay centered in the directly captured RViz stream.
    current_view['Focal Point'] = {'X': 4.0, 'Y': -6.0, 'Z': 0.0}
    # Discard the source QMainWindow state because it restores the removed
    # image docks.  A minimal geometry produces a single maximized render area.
    config['Window Geometry'] = {
        'Hide Left Dock': True,
        'Hide Right Dock': True,
        'X': 0,
        'Y': 0,
        'Width': SOURCE_WINDOW_WIDTH,
        'Height': SOURCE_WINDOW_HEIGHT,
    }
    path.write_text(yaml.safe_dump(config, sort_keys=False))


def window_table():
    result = subprocess.run(
        ['wmctrl', '-l', '-p', '-G'], text=True, capture_output=True, check=False
    )
    rows = []
    for line in result.stdout.splitlines():
        parts = line.split(None, 8)
        if len(parts) == 9:
            rows.append({
                'id': parts[0], 'desktop': parts[1], 'pid': int(parts[2]),
                'x': int(parts[3]), 'y': int(parts[4]), 'width': int(parts[5]),
                'height': int(parts[6]), 'host': parts[7], 'title': parts[8],
            })
    return rows


def find_window(rows, pid=None, title=None):
    candidates = rows
    if pid is not None:
        candidates = [row for row in candidates if row['pid'] == pid]
    if title is not None:
        candidates = [row for row in candidates if title.lower() in row['title'].lower()]
    return candidates[-1] if candidates else None


def move_window(row, x, y, width, height):
    subprocess.run(
        ['wmctrl', '-i', '-r', row['id'], '-b', 'remove,maximized_vert,maximized_horz'],
        check=False,
    )
    subprocess.run(
        ['wmctrl', '-i', '-r', row['id'], '-e', f'0,{x},{y},{width},{height}'],
        check=False,
    )
    subprocess.run(
        ['wmctrl', '-i', '-r', row['id'], '-b', 'add,above'], check=False
    )


def arrange_windows(rviz_pid, unity_pid, deadline):
    found = {}
    while time.monotonic() < deadline:
        rows = window_table()
        found = {
            'rviz': find_window(rows, pid=rviz_pid) or find_window(rows, title='RViz'),
            'unity': find_window(rows, pid=unity_pid),
        }
        if all(found.values()):
            move_window(found['rviz'], 0, 0, SOURCE_WINDOW_WIDTH, SOURCE_WINDOW_HEIGHT)
            move_window(found['unity'], SOURCE_WINDOW_WIDTH, 0,
                        SOURCE_WINDOW_WIDTH, SOURCE_WINDOW_HEIGHT)
            time.sleep(1)
            return found, window_table()
        time.sleep(0.25)
    return found, window_table()


def fit_frame(frame, width, height):
    """Letterbox one source frame without changing its aspect ratio."""
    if frame is None or frame.size == 0:
        return np.zeros((height, width, 3), dtype=np.uint8)
    scale = min(width / frame.shape[1], height / frame.shape[0])
    resized_width = max(1, round(frame.shape[1] * scale))
    resized_height = max(1, round(frame.shape[0] * scale))
    interpolation = cv2.INTER_CUBIC if scale > 1 else cv2.INTER_AREA
    resized = cv2.resize(frame, (resized_width, resized_height),
                         interpolation=interpolation)
    output = np.zeros((height, width, 3), dtype=np.uint8)
    x = (width - resized_width) // 2
    y = (height - resized_height) // 2
    output[y:y + resized_height, x:x + resized_width] = resized
    return output


def place_frame(canvas, frame, region):
    x, y, width, height = region
    canvas[y:y + height, x:x + width] = fit_frame(frame, width, height)


def draw_text(canvas, value, position, scale=0.62, color=TEXT, thickness=1):
    # OpenCV's built-in font is deliberately used for deterministic rendering.
    # Escaping non-ASCII keeps a returned multilingual VLM payload legible as
    # data instead of silently producing missing-glyph boxes.
    value = str(value).encode('ascii', 'backslashreplace').decode('ascii')
    cv2.putText(
        canvas, value, position, cv2.FONT_HERSHEY_SIMPLEX, scale,
        color, thickness, cv2.LINE_AA,
    )


def compact_text(value, limit=240):
    if value is None:
        return '-'
    if isinstance(value, (dict, list, tuple)):
        value = json.dumps(value, ensure_ascii=True, separators=(',', ':'))
    value = re.sub(r'\s+', ' ', str(value)).strip()
    return value if len(value) <= limit else value[:limit - 3] + '...'


def wrap_text(value, width=82, max_lines=3):
    words = compact_text(value, 600).split(' ')
    lines = []
    current = ''
    for word in words:
        candidate = word if not current else f'{current} {word}'
        if len(candidate) <= width:
            current = candidate
        else:
            if current:
                lines.append(current)
            current = word
            if len(lines) >= max_lines:
                break
    if current and len(lines) < max_lines:
        lines.append(current)
    if len(lines) == max_lines and words and not lines[-1].endswith('...'):
        lines[-1] = compact_text(lines[-1], max(4, width - 3)) + '...'
    return lines or ['-']


class DemoTelemetry:
    """Live, episode-local state rendered into the composed demo."""

    def __init__(self, instruction):
        self.instruction = instruction
        self.target_object = '-'
        self.stage = 'SYSTEM STARTUP'
        self.stage_detail = 'Launching simulation, RViz, and perception nodes'
        self.stage_color = AMBER
        self.stage_priority = 0
        self.memory_count = 0
        self.memory_nodes = {}
        self.memory_labels = Counter()
        self.memory_actions = Counter()
        self.candidate_count = 0
        self.vlm_submissions = 0
        self.vlm_outputs = 0
        self.latest_vlm_input = None
        self.latest_vlm_output = None
        self.pending_requests = set()
        self.timeline = deque(maxlen=5)
        self.notice_queue = deque()
        self.active_notice = None
        self.active_notice_until = 0.0
        self.captions_shown = 0
        self.instruction_input_recorded = False
        self.instruction_output_recorded = False

    def set_stage(self, stage, detail='', color=CYAN, priority=1, force=False):
        if not force and priority < self.stage_priority:
            return
        if stage != self.stage or (detail and detail != self.stage_detail):
            self.timeline.appendleft((time.monotonic(), stage, detail))
        self.stage = stage
        self.stage_detail = detail
        self.stage_color = color
        self.stage_priority = priority

    def record_instruction_input(self):
        if self.instruction_input_recorded:
            return
        self.instruction_input_recorded = True
        self.vlm_submissions += 1
        self.pending_requests.add('instruction-parse')
        self.latest_vlm_input = {
            'kind': 'instruction_parse',
            'request_id': 'instruction-parse',
            'object_id': None,
            'target': None,
            'labels': [],
            'room': None,
            'image': None,
            'prompt': f'Instruction: {self.instruction}',
        }
        self.set_stage(
            'TASK PARSING / VLM',
            'Decomposing target, room, attributes, and anchor',
            AMBER, priority=3, force=True,
        )

    def record_instruction_output(self, msg):
        if not self.instruction_input_recorded or self.instruction_output_recorded:
            return
        self.instruction_output_recorded = True
        self.target_object = msg.target_object or '-'
        output = {
            'kind': 'instruction_parse',
            'request_id': 'instruction-parse',
            'object_id': None,
            'accepted': True,
            'is_target': None,
            'label': msg.target_object,
            'reason': compact_text({
                'room': msg.room_condition,
                'spatial': msg.spatial_condition,
                'attribute': msg.attribute_condition,
                'anchor': msg.anchor_object,
            }),
            'latency_s': None,
            'timeout': False,
        }
        self._record_vlm_output(output)
        self.set_stage(
            'EXPLORING', f'Searching for {self.target_object}',
            GREEN, priority=1, force=True,
        )

    def set_memory_nodes(self, nodes):
        # ObjectNodeList is published in incremental room/object batches.  A
        # frame-local len(msg.nodes) therefore drops whenever a smaller batch
        # arrives and does not represent memory growth.  Maintain the same
        # episode-local ID map as the evidence logger so the displayed count is
        # cumulative and deletions/merges are still reflected.
        for node in nodes:
            if not node.object_id:
                continue
            object_id = int(node.object_id[0])
            if node.status:
                self.memory_nodes[object_id] = (
                    node.label or 'unknown'
                ).strip().lower()
            else:
                self.memory_nodes.pop(object_id, None)
        self.memory_count = len(self.memory_nodes)
        self.memory_labels = Counter(self.memory_nodes.values())

    def _record_vlm_output(self, output):
        self.vlm_outputs += 1
        self.latest_vlm_output = output
        request_id = output.get('request_id') or f'vlm-output-{self.vlm_outputs}'
        self.pending_requests.discard(request_id)
        result = output.get('is_target')
        if result is None:
            result = output.get('label') or ('accepted' if output.get('accepted') else 'rejected')
        notice = (
            f'VLM OUTPUT RECEIVED  |  {output.get("kind", "unknown")}  |  '
            f'{request_id}  |  result={result}'
        )
        # The terminal target-confirmation result must be visible immediately:
        # the harness only retains two seconds after arrival, so leaving it
        # behind earlier category-review captions can make a valid episode end
        # before the decisive result is ever shown.  The displaced caption has
        # already appeared for at least one encoded frame and remains counted.
        if output.get('kind') == 'target_confirmation':
            self.notice_queue.appendleft(notice)
            self.active_notice = None
            self.active_notice_until = 0.0
        else:
            self.notice_queue.append(notice)

    def handle_trace(self, payload):
        event = payload.get('event')
        if event == 'yolo_target_detected':
            self.set_stage(
                'TARGET DETECTED',
                f'YOLO track {payload.get("track_id")} -> mask and 3D projection',
                AMBER, priority=2,
            )
        elif event == 'semantic_projection_ready' and payload.get('projection_valid'):
            self.set_stage(
                'SEMANTIC PROJECTION',
                f'{payload.get("point_count", 0)} valid 3D points',
                CYAN, priority=2,
            )
        elif event == 'object_memory_associated':
            action = payload.get('association_action', 'unknown')
            self.memory_actions[action] += 1
            if action in {'created', 'merged_by_geometry'}:
                self.timeline.appendleft((
                    time.monotonic(), 'MEMORY ' + action.upper(),
                    f'object {payload.get("object_id")} / {payload.get("dominant_label")}',
                ))
        elif event == 'vlm_candidate_selected':
            self.candidate_count += 1
            self.set_stage(
                'VLM CANDIDATE QUEUED',
                f'object {payload.get("object_id")} / {payload.get("dominant_label")}',
                AMBER, priority=3,
            )
        elif event == 'vlm_request_started':
            kind = payload.get('request_kind', 'vlm')
            self.set_stage(
                'VLM REVIEW',
                f'{kind} request {payload.get("request_id")} started',
                AMBER, priority=4,
            )
        elif event == 'vlm_submitted':
            request = payload.get('input') or {}
            prompt = request.get('prompt') or '-'
            if isinstance(prompt, dict):
                prompt = prompt.get('user') or prompt.get('system') or prompt
            self.vlm_submissions += 1
            self.pending_requests.add(payload.get('request_id'))
            self.latest_vlm_input = {
                'kind': payload.get('request_kind', 'unknown'),
                'request_id': payload.get('request_id'),
                'object_id': payload.get('object_id'),
                'target': payload.get('target_object'),
                'labels': request.get('candidate_labels') or [],
                'room': request.get('room_context'),
                'image': request.get('image_path'),
                'prompt': compact_text(prompt),
            }
            self.set_stage(
                'VLM API IN FLIGHT',
                f'{payload.get("request_kind", "unknown")} / object {payload.get("object_id")}',
                AMBER, priority=5,
            )
        elif event == 'vlm_result':
            raw = payload.get('output') or {}
            output = {
                'kind': payload.get('request_kind', 'unknown'),
                'request_id': payload.get('request_id'),
                'object_id': payload.get('object_id'),
                'accepted': raw.get('accepted', False),
                'is_target': raw.get('is_target'),
                'label': raw.get('final_label'),
                'reason': raw.get('reason') or raw.get('raw_output') or payload.get('error'),
                'latency_s': payload.get('api_latency_s'),
                'timeout': payload.get('timeout', False),
            }
            self._record_vlm_output(output)
            if output['kind'] == 'target_confirmation':
                if output['is_target']:
                    self.set_stage(
                        'TARGET CONFIRMED', 'VLM accepted the observed candidate',
                        GREEN, priority=8,
                    )
                else:
                    self.set_stage(
                        'EXPLORING', 'VLM rejected the observed candidate',
                        RED, priority=1, force=True,
                    )
            elif output['kind'] == 'object_type':
                if output['is_target']:
                    self.set_stage(
                        'POTENTIAL TARGET', 'Category review passed',
                        GREEN, priority=5,
                    )
                else:
                    self.set_stage(
                        'EXPLORING', 'Category review rejected candidate',
                        RED, priority=1, force=True,
                    )

    def handle_planner_log(self, message):
        if (
            'Potential Target' in message
            and (
                'first observed at' in message
                or 'approaching nearest connected collision-free' in message
            )
        ):
            match = re.search(
                r'Potential Target (\d+) first observed at ([0-9.]+) m', message
            )
            object_match = re.search(r'Potential Target (\d+)', message)
            detail = (
                f'Object {match.group(1)} first observed at {match.group(2)} m; '
                'following the observation approach'
                if match
                else f'Object {object_match.group(1) if object_match else "-"}: '
                'following nearest traversable observation waypoint'
            )
            self.set_stage(
                'APPROACHING CANDIDATE', detail,
                AMBER, priority=6,
            )
        elif 'Reached observation point for Potential Target' in message:
            match = re.search(r'Potential Target (\d+)', message)
            self.set_stage(
                'OBSERVING / WAITING VLM',
                f'Object {match.group(1) if match else "-"}: arrival gates passed; holding position',
                AMBER, priority=7,
            )
        elif 'Holding observation position for Potential Target' in message:
            match = re.search(r'Potential Target (\d+)', message)
            self.set_stage(
                'OBSERVING / WAITING VLM',
                f'Object {match.group(1) if match else "-"}: final VLM review pending',
                AMBER, priority=7,
            )
        elif 'passed category review' in message:
            match = re.search(r'Target object (\d+)', message)
            self.set_stage(
                'APPROACHING CANDIDATE',
                f'Object {match.group(1) if match else "-"}: category review passed',
                AMBER, priority=6,
            )
        elif 'Found target object id' in message:
            match = re.search(r'Found target object id:\s*(\d+)', message)
            self.set_stage(
                'TARGET NAVIGATION',
                f'Planner accepted object {match.group(1) if match else "-"}; executing target path',
                GREEN, priority=8,
            )
        elif 'skipping final target navigation' in message:
            self.set_stage(
                'GOAL REACHED', 'Observation pose already satisfies arrival gates',
                GREEN, priority=9,
            )
        elif 'Found the target object, waiting for next action' in message:
            self.set_stage(
                'GOAL REACHED', 'Target confirmed; retaining final two seconds',
                GREEN, priority=9,
            )
        elif 'observation approach ended' in message and 'rejected' in message:
            self.set_stage(
                'EXPLORING', 'Candidate rejected; exploration resumed',
                RED, priority=1, force=True,
            )

    def caption(self, now):
        if self.active_notice is None or now >= self.active_notice_until:
            if self.notice_queue:
                self.active_notice = self.notice_queue.popleft()
                self.active_notice_until = now + 1.0
                self.captions_shown += 1
            else:
                self.active_notice = None
        return self.active_notice


def render_stage_card(telemetry, width, height, elapsed_s):
    panel = np.full((height, width, 3), PANEL_BACKGROUND, dtype=np.uint8)
    draw_text(panel, 'CURRENT PIPELINE STAGE', (24, 31), 0.55, MUTED)
    cv2.circle(panel, (31, 72), 8, telemetry.stage_color, -1, cv2.LINE_AA)
    draw_text(panel, telemetry.stage, (52, 81), 0.78, telemetry.stage_color, 2)
    for index, line in enumerate(wrap_text(telemetry.stage_detail, 55, 2)):
        draw_text(panel, line, (24, 117 + index * 24), 0.48, TEXT)
    draw_text(
        panel,
        f'{elapsed_s:06.1f}s  |  target: {telemetry.target_object}  |  '
        f'memory: {telemetry.memory_count}',
        (24, height - 19), 0.50, CYAN,
    )
    return panel


def render_dashboard(telemetry, width, height, started):
    panel = np.full((height, width, 3), PANEL_BACKGROUND, dtype=np.uint8)
    draw_text(panel, 'LIVE DATA / VLM I-O', (24, 34), 0.66, TEXT, 2)
    draw_text(
        panel,
        f'memory {telemetry.memory_count}  |  labels {len(telemetry.memory_labels)}  |  '
        f'candidates {telemetry.candidate_count}  |  VLM {telemetry.vlm_submissions} in / '
        f'{telemetry.vlm_outputs} out / {len(telemetry.pending_requests)} API pending',
        (24, 70), 0.48, CYAN,
    )
    cv2.line(panel, (24, 90), (width - 24, 90), (55, 65, 65), 1)

    draw_text(panel, 'LATEST VLM INPUT', (24, 122), 0.54, AMBER, 2)
    request = telemetry.latest_vlm_input
    if request:
        draw_text(
            panel,
            f'{request["kind"]}  |  request {request["request_id"]}  |  object {request["object_id"]}',
            (24, 153), 0.47, TEXT,
        )
        draw_text(
            panel,
            f'target={request["target"] or telemetry.target_object}  labels={compact_text(request["labels"], 72)}  '
            f'room={request["room"] or "-"}',
            (24, 180), 0.45, TEXT,
        )
        draw_text(
            panel,
            f'image={Path(request["image"]).name if request["image"] else "none (text-only request)"}',
            (24, 205), 0.43, MUTED,
        )
        for index, line in enumerate(wrap_text(request['prompt'], 102, 3)):
            draw_text(panel, ('prompt: ' if index == 0 else '        ') + line,
                      (24, 229 + index * 22), 0.41, MUTED)
    else:
        draw_text(panel, 'Waiting for a VLM request...', (24, 158), 0.50, MUTED)

    cv2.line(panel, (24, 292), (width - 24, 292), (55, 65, 65), 1)
    draw_text(panel, 'LATEST VLM OUTPUT', (24, 325), 0.54, GREEN, 2)
    output = telemetry.latest_vlm_output
    if output:
        result_color = (
            RED if (output.get('timeout') or output.get('accepted') is False
                    or output.get('is_target') is False) else GREEN
        )
        draw_text(
            panel,
            f'{output["kind"]}  |  request {output["request_id"]}  |  object {output["object_id"]}',
            (24, 356), 0.47, TEXT,
        )
        latency = output.get('latency_s')
        latency_text = '-' if latency is None else f'{latency:.2f}s'
        draw_text(
            panel,
            f'label={output.get("label") or "-"}  is_target={output.get("is_target")}  '
            f'accepted={output.get("accepted")}  latency={latency_text}  timeout={output.get("timeout")}',
            (24, 383), 0.45, result_color,
        )
        for index, line in enumerate(wrap_text(output.get('reason'), 102, 3)):
            draw_text(panel, ('reason: ' if index == 0 else '        ') + line,
                      (24, 412 + index * 24), 0.43, TEXT)
    else:
        draw_text(panel, 'Waiting for a VLM response...', (24, 360), 0.50, MUTED)

    cv2.line(panel, (24, 493), (width - 24, 493), (55, 65, 65), 1)
    top_labels = ', '.join(
        f'{label}:{count}' for label, count in telemetry.memory_labels.most_common(6)
    ) or '-'
    draw_text(panel, f'MEMORY LABELS  {top_labels}', (24, 524), 0.44, CYAN)
    actions = telemetry.memory_actions
    draw_text(
        panel,
        f'TARGET TRACE  created {actions["created"]}  updated {actions["updated_by_track_id"]}  '
        f'merged {actions["merged_by_geometry"]}  rejected {actions["rejected"]}',
        (24, 553), 0.44, MUTED,
    )
    return panel


def compose_demo_frame(panorama, rviz_frame, unity_frame, telemetry, started):
    """Build one synchronized, information-rich frame without desktop capture."""
    canvas = np.full((CAPTURE_HEIGHT, CAPTURE_WIDTH, 3), BACKGROUND, dtype=np.uint8)
    elapsed_s = time.monotonic() - started
    draw_text(canvas, 'SYSNAV  /  ONLINE SEMANTIC MEMORY', (24, 42), 0.86, TEXT, 2)
    draw_text(canvas, f'Task: {telemetry.instruction}', (24, 78), 0.58, TEXT)
    draw_text(canvas, f'{elapsed_s:06.1f}s  |  LIVE SIMULATION', (2180, 42), 0.54, CYAN)

    draw_text(canvas, '01  YOLOE PANORAMA / INSTANCE MASKS', (24, 130), 0.58, TEXT)
    draw_text(canvas, '02  UNITY / THIRD PERSON', (1864, 130), 0.58, TEXT)
    place_frame(canvas, panorama, PANORAMA_REGION)
    place_frame(canvas, unity_frame, UNITY_REGION)
    place_frame(
        canvas,
        render_stage_card(telemetry, STAGE_REGION[2], STAGE_REGION[3], elapsed_s),
        STAGE_REGION,
    )

    notice = telemetry.caption(time.monotonic())
    if notice:
        x, y, width, height = CAPTION_REGION
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (31, 121, 179), -1)
        draw_text(canvas, notice, (x + 20, y + 30), 0.62, (255, 255, 255), 2)

    draw_text(canvas, '03  RVIZ / SPATIAL + OBJECT MEMORY GROWTH', (24, 818), 0.58, TEXT)
    draw_text(canvas, '04  PIPELINE TELEMETRY', (1260, 818), 0.58, TEXT)
    place_frame(canvas, rviz_frame, RVIZ_REGION)
    place_frame(
        canvas,
        render_dashboard(telemetry, DASHBOARD_REGION[2], DASHBOARD_REGION[3], started),
        DASHBOARD_REGION,
    )
    return canvas


class WindowVideoStream:
    """Decode one application window into an in-memory BGR video stream."""

    def __init__(self, ffmpeg, row, width, height, env, debug):
        self.width = width
        self.height = height
        self.frame_size = width * height * 3
        self.frame_count = 0
        self.latest_frame = None
        self.lock = threading.Lock()
        window_id = int(row['id'], 16)
        # Fill the panel instead of letterboxing a 16:9 application window in
        # the wider companion slot.  The symmetric vertical crop also removes
        # most RViz menu/status chrome while retaining its central render view.
        vf = (
            f'scale={width}:{height}:force_original_aspect_ratio=increase,'
            f'crop={width}:{height}'
        )
        self.process = subprocess.Popen(
            [
                ffmpeg, '-hide_banner', '-loglevel', 'warning',
                '-f', 'x11grab', '-draw_mouse', '0',
                '-framerate', str(FPS), '-window_id', str(window_id),
                '-i', DISPLAY, '-vf', vf, '-an', '-pix_fmt', 'bgr24',
                '-f', 'rawvideo', 'pipe:1',
            ],
            cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=debug,
            start_new_session=True, bufsize=0,
        )
        self.thread = threading.Thread(target=self._read_frames, daemon=True)
        self.thread.start()

    def _read_frames(self):
        while True:
            payload = bytearray()
            while len(payload) < self.frame_size:
                chunk = self.process.stdout.read(self.frame_size - len(payload))
                if not chunk:
                    return
                payload.extend(chunk)
            frame = np.frombuffer(payload, dtype=np.uint8).reshape(
                self.height, self.width, 3
            ).copy()
            with self.lock:
                self.latest_frame = frame
                self.frame_count += 1

    def latest(self):
        with self.lock:
            return None if self.latest_frame is None else self.latest_frame.copy()


def extract_preview(video, preview):
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        return False
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    # Leave the compositor's close-window fade out of the preview while keeping
    # the preview within the final retained second of the real recording.
    if count > FPS:
        capture.set(cv2.CAP_PROP_POS_FRAMES, count - FPS)
    ok, frame = capture.read()
    capture.release()
    return bool(ok and cv2.imwrite(str(preview), frame))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-name', required=True)
    parser.add_argument('--instruction', required=True)
    parser.add_argument('--seconds', type=float, default=300.0)
    parser.add_argument('--linger-seconds', type=float, default=2.0)
    parser.add_argument('--target-x', type=float, required=True)
    parser.add_argument('--target-y', type=float, required=True)
    parser.add_argument('--target-z', type=float, default=0.0)
    args = parser.parse_args()

    run = ROOT / 'recordings' / args.run_name
    run.mkdir(exist_ok=False)
    evidence = run / 'evidence'
    evidence.mkdir()
    (evidence / 'src').symlink_to(ROOT / 'src', target_is_directory=True)
    mobileclip = ROOT / 'mobileclip2_b.ts'
    if not mobileclip.exists():
        raise FileNotFoundError(f'Missing local YOLOE text encoder: {mobileclip}')
    (evidence / mobileclip.name).symlink_to(mobileclip)
    env = os.environ.copy()
    env.update({
        'DISPLAY': DISPLAY,
        'OBJNAV_RUN_DIR': str(run),
        'ROS_LOG_DIR': str(evidence / 'ros'),
        'ROS_DOMAIN_ID': '91',
        'SYSNAV_PYTHON': sys.executable,
        'PYTHONPATH': os.pathsep.join([
            str(Path(sam2.__file__).resolve().parents[1]),
            *(str(Path(item).resolve()) for item in sys.path if item),
        ]),
    })
    os.environ.update(env)

    launch_file = evidence / 'sysnav-episode.launch.py'
    rviz_file = evidence / 'episode.rviz'
    make_launch_wrapper(launch_file)
    make_rviz_config(rviz_file)
    shutil.copy2(Path(__file__), evidence / Path(__file__).name)
    shutil.copy2(
        ROOT / 'src/semantic_mapping/semantic_mapping/detection_node.py',
        evidence / 'detection_node.py',
    )
    shutil.copy2(
        ROOT / 'src/semantic_mapping/semantic_mapping/semantic_map_new.py',
        evidence / 'semantic_map_new.py',
    )
    shutil.copy2(
        ROOT / 'src/semantic_mapping/semantic_mapping/single_object_new.py',
        evidence / 'single_object_new.py',
    )
    shutil.copy2(
        ROOT / 'src/semantic_mapping/semantic_mapping/semantic_mapping_node.py',
        evidence / 'semantic_mapping_node.py',
    )
    shutil.copy2(
        ROOT / 'src/semantic_mapping/semantic_mapping/config/objects.yaml',
        evidence / 'objects.yaml',
    )
    (evidence / 'run-config.json').write_text(json.dumps({
        'instruction': args.instruction,
        'max_seconds': args.seconds,
        'post_arrival_linger_seconds': args.linger_seconds,
        'ros_domain_id': 91,
        'video_source': (
            'direct synchronized compositor: /annotated_image + RViz window '
            'stream + Unity window stream; no desktop capture'
        ),
        'video_resolution': CAPTURE_SIZE,
        'desktop_capture': False,
        'layout': {
            'panorama': list(PANORAMA_REGION),
            'rviz_memory': list(RVIZ_REGION),
            'unity_third_person': list(UNITY_REGION),
            'pipeline_stage': list(STAGE_REGION),
            'vlm_io_dashboard': list(DASHBOARD_REGION),
            'vlm_result_caption': list(CAPTION_REGION),
        },
        'panorama_topic': '/annotated_image',
        'ground_truth_target_pose': [args.target_x, args.target_y, args.target_z],
    }, indent=2))

    debug = (run / 'debug.log').open('w', buffering=1)
    processes = {}
    bridge = CvBridge()
    last_panorama = None
    panorama_frames = 0
    target_instruction_received = False
    prompt_attempts = 0
    last_prompt_at = None
    reached_at = None
    failure = None
    windows = {}
    window_snapshot = []
    rviz_stream = None
    unity_stream = None
    encoded_frames = 0
    telemetry = DemoTelemetry(args.instruction)
    node = None

    def start(name, command, **kwargs):
        process = subprocess.Popen(
            command, cwd=evidence, env=env, stdout=debug,
            stderr=subprocess.STDOUT, start_new_session=True, **kwargs
        )
        processes[name] = process
        return process

    try:
        launch = start('simulation', ['ros2', 'launch', str(launch_file)])
        time.sleep(3)
        unity = start('unity', [
            str(ROOT / 'src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64'),
            '-screen-width', str(SOURCE_WINDOW_WIDTH),
            '-screen-height', str(SOURCE_WINDOW_HEIGHT),
            '-window-mode', 'borderless', '-logFile', str(evidence / 'unity.log'),
        ])
        logger = start('logger', [
            sys.executable, str(ROOT / 'scripts/objnav_episode_logger.py'),
            '--root', str(ROOT), '--run-dir', str(run),
            '--seconds', str(args.seconds + args.linger_seconds + 30),
            '--instruction', args.instruction,
            '--target-x', str(args.target_x), '--target-y', str(args.target_y),
            '--target-z', str(args.target_z),
        ])
        rviz_env = env.copy()
        rviz_env.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
        rviz_env.pop('QT_QPA_FONTDIR', None)
        rviz = subprocess.Popen(
            ['rviz2', '-d', str(rviz_file)], cwd=evidence, env=rviz_env,
            stdout=debug, stderr=subprocess.STDOUT, start_new_session=True,
        )
        processes['rviz'] = rviz

        rclpy.init()
        node = rclpy.create_node('sysnav_demo_episode_runner')

        def receive_panorama(msg):
            nonlocal last_panorama, panorama_frames
            last_panorama = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            panorama_frames += 1

        def receive_instruction(msg):
            nonlocal target_instruction_received
            target_instruction_received = bool(msg.target_object.strip())
            telemetry.record_instruction_output(msg)

        def receive_objects(msg):
            telemetry.set_memory_nodes(msg.nodes)

        def receive_log(msg):
            nonlocal reached_at
            marker = 'OBJNAV_TRACE '
            if marker in msg.msg:
                try:
                    telemetry.handle_trace(json.loads(msg.msg.split(marker, 1)[1]))
                except json.JSONDecodeError:
                    pass
            if msg.name == 'tare_planner_node':
                telemetry.handle_planner_log(msg.msg)
            if (msg.name == 'tare_planner_node'
                    and 'Found the target object, waiting for next action' in msg.msg
                    and reached_at is None):
                reached_at = time.monotonic()

        node.create_subscription(Image, '/annotated_image', receive_panorama, 10)
        node.create_subscription(TargetObjectInstruction, '/target_object_instruction', receive_instruction, 10)
        node.create_subscription(ObjectNodeList, '/object_nodes_list', receive_objects, 50)
        node.create_subscription(Log, '/rosout', receive_log, 1000)
        prompt_pub = node.create_publisher(String, '/keyboard_input', 10)

        placeholder = cv2.imread(str(ROOT / 'src/base_autonomy/vehicle_simulator/mesh/unity/render.jpg'))
        placeholder = cv2.resize(placeholder, (1440, 480))
        cv2.putText(placeholder, 'Waiting for YOLO + SAM2 panorama...', (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 255), 3, cv2.LINE_AA)
        last_panorama = placeholder

        startup_deadline = time.monotonic() + 35
        windows, window_snapshot = arrange_windows(rviz.pid, unity.pid, startup_deadline)
        time.sleep(3)
        windows, window_snapshot = arrange_windows(rviz.pid, unity.pid, time.monotonic() + 5)
        if not process_alive(rviz):
            raise RuntimeError('rviz_exited_during_startup')
        if not windows.get('rviz'):
            raise RuntimeError('rviz_window_not_found')
        if not process_alive(unity) or not windows.get('unity'):
            raise RuntimeError('unity_window_not_found')

        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        rviz_stream = WindowVideoStream(
            ffmpeg, windows['rviz'], RVIZ_REGION[2], RVIZ_REGION[3],
            env, debug,
        )
        unity_stream = WindowVideoStream(
            ffmpeg, windows['unity'], UNITY_REGION[2], UNITY_REGION[3],
            env, debug,
        )
        processes['rviz_stream'] = rviz_stream.process
        processes['unity_stream'] = unity_stream.process
        recorder = start('recorder', [
            ffmpeg, '-hide_banner', '-loglevel', 'warning', '-y',
            '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-video_size', CAPTURE_SIZE,
            '-framerate', str(FPS), '-i', 'pipe:0',
            '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
            str(run / 'demo.mp4'),
        ], stdin=subprocess.PIPE)

        started = time.monotonic()
        next_frame_at = started
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.03)
            now = time.monotonic()
            if now >= next_frame_at:
                frame = compose_demo_frame(
                    last_panorama, rviz_stream.latest(), unity_stream.latest(),
                    telemetry, started,
                )
                recorder.stdin.write(frame.tobytes())
                encoded_frames += 1
                next_frame_at += 1.0 / FPS
                if next_frame_at < now - 1.0 / FPS:
                    next_frame_at = now + 1.0 / FPS

            if (last_panorama is not None and not target_instruction_received
                    and (last_prompt_at is None or now - last_prompt_at >= 5.0)
                    and 'vlm_node' in set(node.get_node_names()) and prompt_attempts < 3):
                prompt = String()
                prompt.data = args.instruction
                telemetry.record_instruction_input()
                prompt_pub.publish(prompt)
                prompt_attempts += 1
                last_prompt_at = now

            required_processes = [
                'simulation', 'unity', 'rviz', 'rviz_stream',
                'unity_stream', 'recorder'
            ]
            for name in required_processes:
                if not process_alive(processes[name]):
                    raise RuntimeError(f'{name}_exited_early')
            # The passive logger closes as soon as it records the planner's
            # arrival event.  Its clean exit must not stop the harness before
            # the required post-arrival video has been retained.
            if reached_at is None and not process_alive(processes['logger']):
                metrics_path = run / 'metrics.json'
                if metrics_path.exists():
                    logger_metrics = json.loads(metrics_path.read_text())
                    if logger_metrics.get('success'):
                        reached_at = now
                    else:
                        raise RuntimeError('logger_exited_early')
                else:
                    raise RuntimeError('logger_exited_without_metrics')
            if reached_at is not None and now - reached_at >= args.linger_seconds:
                break

        if reached_at is None:
            failure = 'timeout'
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or 'interrupted'
    finally:
        if failure == 'timeout' and process_alive(processes.get('logger')):
            os.killpg(processes['logger'].pid, signal.SIGUSR1)
            try:
                processes['logger'].wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        recorder_process = processes.get('recorder')
        if process_alive(recorder_process) and recorder_process.stdin is not None:
            try:
                recorder_process.stdin.close()
                recorder_process.wait(timeout=15)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                pass
        for name in ('recorder', 'rviz_stream', 'unity_stream', 'logger',
                     'rviz', 'unity', 'simulation'):
            if name in processes:
                terminate_group(processes[name], debug)
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        debug.close()

    video = run / 'demo.mp4'
    preview_ok = video.exists() and video.stat().st_size > 0 and extract_preview(
        video, run / 'demo-preview.jpg'
    )
    captions_complete = (
        telemetry.vlm_outputs > 0
        and telemetry.captions_shown == telemetry.vlm_outputs
        and not telemetry.notice_queue
    )
    required = {
        'panorama_recorded': panorama_frames > 0 and video.exists(),
        'rviz_memory_view_recorded': (
            rviz_stream is not None and rviz_stream.frame_count > 0 and video.exists()
        ),
        'unity_third_person_recorded': (
            unity_stream is not None and unity_stream.frame_count > 0 and video.exists()
        ),
        'data_panel_recorded': (
            encoded_frames > 0 and telemetry.instruction_input_recorded
            and telemetry.vlm_outputs > 0 and captions_complete and video.exists()
        ),
    }
    validation = {
        **required,
        'rviz_started': bool(windows.get('rviz')),
        'rviz_process_started': 'rviz' in processes,
        'unity_process_started': 'unity' in processes,
        'panorama_frames_received': panorama_frames,
        'rviz_stream_frames_received': (
            0 if rviz_stream is None else rviz_stream.frame_count
        ),
        'unity_stream_frames_received': (
            0 if unity_stream is None else unity_stream.frame_count
        ),
        'composed_frames_encoded': encoded_frames,
        'vlm_submissions_observed': telemetry.vlm_submissions,
        'vlm_outputs_received': telemetry.vlm_outputs,
        'vlm_result_captions_shown': telemetry.captions_shown,
        'vlm_result_captions_complete': captions_complete,
        'vlm_caption_queue_remaining': len(telemetry.notice_queue),
        'memory_objects_at_end': telemetry.memory_count,
        'memory_labels_at_end': dict(telemetry.memory_labels),
        'memory_association_actions': dict(telemetry.memory_actions),
        'final_pipeline_stage': telemetry.stage,
        'desktop_capture': False,
        'composition_layout': {
            'panorama': list(PANORAMA_REGION),
            'rviz_memory': list(RVIZ_REGION),
            'unity_third_person': list(UNITY_REGION),
            'pipeline_stage': list(STAGE_REGION),
            'vlm_io_dashboard': list(DASHBOARD_REGION),
            'vlm_result_caption': list(CAPTION_REGION),
        },
        'preview_extracted_from_demo': preview_ok,
        'windows_before_recording': window_snapshot,
        'failure': failure,
    }
    (evidence / 'validation.json').write_text(json.dumps(validation, indent=2))

    metrics_path = run / 'metrics.json'
    metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
    navigation_success = bool(metrics.get('success'))
    capture_success = all(required.values()) and preview_ok
    metrics.update({
        **required,
        'rviz_started': bool(windows.get('rviz')),
        'demo_recorded': capture_success,
        'desktop_capture': False,
        'composition_layout': validation['composition_layout'],
        'vlm_submissions_observed': telemetry.vlm_submissions,
        'vlm_outputs_received': telemetry.vlm_outputs,
        'vlm_result_captions_shown': telemetry.captions_shown,
        'vlm_result_captions_complete': captions_complete,
        'memory_objects_at_end': telemetry.memory_count,
        'final_pipeline_stage': telemetry.stage,
        'demo_failure_reason': None if capture_success else (failure or 'required_view_validation_failed'),
        'navigation_success': navigation_success,
        'success': navigation_success and capture_success,
        'post_arrival_linger_seconds': args.linger_seconds,
    })
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2))
    print(run, flush=True)
    if failure and failure != 'timeout':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
