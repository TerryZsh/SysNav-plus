#!/usr/bin/env python3
import argparse
from collections import Counter, deque
import csv
import json
import math
from pathlib import Path
import re
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rcl_interfaces.msg import Log
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tare_planner.msg import (
    DetectionResult,
    ObjectNodeList,
    TargetObject,
    TargetObjectInstruction,
)


ALLOWED_EVENTS = {
    'episode_start',
    'yolo_target_detected',
    'semantic_projection_ready',
    'object_memory_associated',
    'vlm_candidate_selected',
    'vlm_candidate_rejected',
    'vlm_request_started',
    'vlm_submitted',
    'vlm_result',
    'planner_input_sent',
    'planner_input_received',
    'planner_path_generated',
    'controller_path_received',
    'motion_started',
    'navigation_stalled',
    'episode_end',
}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def full_pose(pose):
    return {
        'x': float(pose.position.x),
        'y': float(pose.position.y),
        'z': float(pose.position.z),
        'qx': float(pose.orientation.x),
        'qy': float(pose.orientation.y),
        'qz': float(pose.orientation.z),
        'qw': float(pose.orientation.w),
    }


def point_pose(point):
    return {
        'x': float(point.x),
        'y': float(point.y),
        'z': float(point.z),
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0.0,
        'qw': 1.0,
    }


def path_length(path):
    return sum(
        math.dist(
            (first['x'], first['y'], first['z']),
            (second['x'], second['y'], second['z']),
        )
        for first, second in zip(path, path[1:])
    )


class EpisodeLogger:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.run_dir = args.run_dir
        self.run_dir.mkdir(parents=True, exist_ok=True)
        (self.run_dir / 'evidence').mkdir()
        self.output = (self.run_dir / 'objnav.jsonl').open('x', buffering=1)
        self.started_steady_ns = time.monotonic_ns()
        self.started_ros = self.ros_now()
        self.trace_id = f'{self.run_dir.name}/{args.target}-1'
        self.events = []
        self.counts = Counter()
        self.poses = deque(maxlen=12000)
        self.trajectory = []
        self.first_pose = None
        self.last_pose = None
        self.last_speed = 0.0
        self.last_sample_steady = 0
        self.detection_pose = None
        self.takeover_pose = None
        self.final_pose = None
        self.target_detected = False
        self.track_to_object = {}
        self.objects = {}
        self.pending_goal = None
        self.goal_id = None
        self.path_id = None
        self.paths = []
        self.last_path_signature = None
        self.controller_received = False
        self.first_cmd = None
        self.motion_started = False
        self.last_progress_pose = None
        self.last_progress_steady = time.monotonic()
        self.last_stall_steady = 0.0
        self.last_cmd = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        self.terrain_xy = []
        self.last_terrain_sample = 0.0
        self.semantic_samples = []
        self.target_colors = self.load_target_colors()
        self.target_instruction_received = False
        self.prompt_attempts = 0
        self.last_prompt_at = None
        self.done = False
        self.goal_reached = False
        self.stop_reason = 'timeout'
        self.stop_source = 'timeout'

        node.create_subscription(Odometry, '/state_estimation', self.on_odom, qos_profile_sensor_data)
        node.create_subscription(Image, '/camera/semantic_image', self.on_semantic, qos_profile_sensor_data)
        node.create_subscription(DetectionResult, '/detection_result', self.on_detection, 50)
        node.create_subscription(ObjectNodeList, '/object_nodes_list', self.on_objects, 200)
        node.create_subscription(TargetObjectInstruction, '/target_object_instruction', self.on_instruction, 10)
        node.create_subscription(TargetObject, '/target_object_answer', self.on_target_answer, 50)
        node.create_subscription(NavPath, '/global_path', self.on_path, 20)
        node.create_subscription(PointStamped, '/way_point', self.on_waypoint, 20)
        node.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 50)
        node.create_subscription(PointCloud2, '/terrain_map_ext', self.on_terrain, qos_profile_sensor_data)
        node.create_subscription(Log, '/rosout', self.on_log, 1000)
        self.prompt_pub = node.create_publisher(String, '/keyboard_input', 10)
        node.create_timer(0.25, self.tick)

        yaw_half = args.start_yaw / 2.0
        start_pose = {
            'x': args.start_x,
            'y': args.start_y,
            'z': args.start_z,
            'qx': 0.0,
            'qy': 0.0,
            'qz': math.sin(yaw_half),
            'qw': math.cos(yaw_half),
        }
        self.event(
            'episode_start',
            instruction=args.instruction,
            target_object=args.target,
            start_pose=start_pose,
            configured_goal_pose=point_pose(args.target_point),
            status='completed',
        )

    def ros_now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return (time.monotonic_ns() - self.started_steady_ns) / 1e9

    def common(self, event, fields):
        object_id = fields.get('object_id')
        track_id = fields.get('track_id')
        if object_id is None and track_id is not None:
            object_id = self.track_to_object.get(int(track_id))
        if track_id is None and object_id is not None:
            candidates = [track for track, obj in self.track_to_object.items() if obj == int(object_id)]
            track_id = candidates[-1] if candidates else None
        return {
            'event': event,
            'timestamp': float(fields.pop('timestamp', self.ros_now())),
            'steady_time_ns': int(fields.pop('steady_time_ns', time.monotonic_ns())),
            'elapsed_s': round(self.elapsed(), 6),
            'frame_id': 'map',
            'trace_id': self.trace_id,
            'image_sequence_id': fields.pop('image_sequence_id', None),
            'track_id': int(track_id) if track_id is not None else None,
            'object_id': int(object_id) if object_id is not None else None,
            'request_id': fields.pop('request_id', None),
            'goal_id': fields.pop('goal_id', self.goal_id),
            'path_id': fields.pop('path_id', self.path_id),
            'status': fields.pop('status', 'completed'),
            **fields,
        }

    def event(self, event, **fields):
        if event not in ALLOWED_EVENTS:
            raise ValueError(f'event not allowed: {event}')
        item = self.common(event, fields)
        self.events.append(item)
        self.output.write(json.dumps(item, ensure_ascii=False, separators=(',', ':')) + '\n')
        print(json.dumps(item, ensure_ascii=False), flush=True)

    def load_target_colors(self):
        scene = self.args.root / 'src/base_autonomy/vehicle_simulator/mesh/unity/environment'
        with (scene / 'Categories.csv').open(encoding='utf-8-sig') as stream:
            categories = {row['name']: row['nyu40class'].strip() for row in csv.DictReader(stream)}
        with (scene / 'AssetList.csv').open(encoding='utf-8-sig') as stream:
            return [
                np.array([int(row['b']), int(row['g']), int(row['r'])], dtype=np.int16)
                for row in csv.DictReader(stream)
                if categories.get(row['name']) == self.args.target
            ]

    def nearest_pose(self, timestamp):
        if not self.poses:
            return None
        return min(self.poses, key=lambda item: abs(item[0] - timestamp))[1]

    def on_odom(self, msg):
        self.counts['state_estimation'] += 1
        pose = full_pose(msg.pose.pose)
        timestamp = stamp_seconds(msg.header.stamp)
        self.poses.append((timestamp, pose))
        self.last_pose = pose
        self.final_pose = pose
        if self.first_pose is None:
            self.first_pose = pose
        velocity = msg.twist.twist.linear
        self.last_speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        now_steady = time.monotonic_ns()
        if now_steady - self.last_sample_steady >= 200_000_000:
            self.trajectory.append({
                'timestamp': timestamp,
                'elapsed_s': round(self.elapsed(), 3),
                **pose,
                'speed_mps': self.last_speed,
            })
            self.last_sample_steady = now_steady
        if self.pending_goal is not None:
            if self.last_progress_pose is None:
                self.last_progress_pose = pose
                self.last_progress_steady = time.monotonic()
            elif math.hypot(
                pose['x'] - self.last_progress_pose['x'],
                pose['y'] - self.last_progress_pose['y'],
            ) >= 0.10:
                if self.first_cmd is not None and not self.motion_started:
                    self.motion_started = True
                    self.event(
                        'motion_started',
                        first_cmd_timestamp=self.first_cmd['timestamp'],
                        first_motion_timestamp=timestamp,
                        first_cmd_vel=self.first_cmd['velocity'],
                        robot_pose=pose,
                        planner_to_cmd_latency_s=(
                            self.first_cmd['steady'] - self.pending_goal['received_steady']
                        ),
                        cmd_to_motion_latency_s=(
                            time.monotonic() - self.first_cmd['steady']
                        ),
                        motion_distance_threshold_m=0.10,
                    )
                self.last_progress_pose = pose
                self.last_progress_steady = time.monotonic()

    def on_semantic(self, msg):
        self.counts['semantic_image'] += 1
        if self.target_instruction_received or len(self.semantic_samples) >= 3:
            return
        if msg.encoding not in ('bgr8', 'rgb8'):
            return
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = image[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            image = image[:, :, ::-1]
        image16 = image.astype(np.int16)
        pixels = [
            int(np.count_nonzero(np.max(np.abs(image16 - color), axis=2) <= 8))
            for color in self.target_colors
        ]
        self.semantic_samples.append({
            'frame_timestamp': stamp_seconds(msg.header.stamp),
            'target_pixels': pixels,
        })

    def on_instruction(self, msg):
        self.counts['target_instruction'] += 1
        if msg.target_object.strip().lower() == self.args.target:
            self.target_instruction_received = True

    def on_detection(self, msg):
        self.counts['detection_result'] += 1
        if not self.target_instruction_received or self.target_detected:
            return
        for index, label in enumerate(msg.label):
            if label.strip().lower() != self.args.target:
                continue
            self.target_detected = True
            frame_timestamp = stamp_seconds(msg.header.stamp)
            self.detection_pose = self.nearest_pose(frame_timestamp)
            self.event(
                'yolo_target_detected',
                timestamp=self.ros_now(),
                target_object=self.args.target,
                track_id=int(msg.track_id[index]),
                confidence=float(msg.confidence[index]),
                bbox=[
                    float(msg.x1[index]),
                    float(msg.y1[index]),
                    float(msg.x2[index]),
                    float(msg.y2[index]),
                ],
                frame_timestamp=frame_timestamp,
                detected_timestamp=self.ros_now(),
                robot_pose=self.detection_pose,
                image_id=f'detection-{msg.header.stamp.sec}-{msg.header.stamp.nanosec}',
                timing={
                    'source_timestamp': frame_timestamp,
                    'queued_timestamp': None,
                    'started_timestamp': None,
                    'finished_timestamp': self.ros_now(),
                    'published_timestamp': stamp_seconds(msg.header.stamp),
                    'downstream_received_timestamp': None,
                    'queue_wait_s': None,
                    'processing_s': None,
                    'transport_s': None,
                    'end_to_end_s': self.ros_now() - frame_timestamp,
                },
            )
            return

    def on_objects(self, msg):
        self.counts['object_nodes_list'] += 1
        for obj in msg.nodes:
            if not obj.status:
                continue
            object_id = int(obj.object_id[0]) if obj.object_id else None
            record = {
                'pose': point_pose(obj.position),
                'label': obj.label,
                'track_ids': [int(value) for value in obj.object_id],
                'image_path': obj.img_path,
                'bbox3d': [
                    {'x': float(point.x), 'y': float(point.y), 'z': float(point.z)}
                    for point in obj.bbox3d
                ],
            }
            if object_id is not None:
                self.objects[object_id] = record
            for track_id in obj.object_id:
                self.track_to_object[int(track_id)] = object_id

    def on_target_answer(self, msg):
        self.counts['target_object_answer'] += 1
        object_id = int(msg.object_id)
        obj = self.objects.get(object_id, {})
        goal_id = f'goal-{object_id}-{msg.header.stamp.sec}-{msg.header.stamp.nanosec}'
        self.goal_id = goal_id
        self.pending_goal = {
            'object_id': object_id,
            'target_pose': obj.get('pose'),
            'sent_steady': time.monotonic(),
            'received_steady': None,
            'is_target': bool(msg.is_target),
        }
        self.event(
            'planner_input_sent',
            timestamp=stamp_seconds(msg.header.stamp),
            goal_id=goal_id,
            object_id=object_id,
            target_object=self.args.target,
            input={
                'robot_pose': self.last_pose,
                'target_pose': obj.get('pose'),
                'target_bbox3d': obj.get('bbox3d'),
                'target_point_cloud_id': object_id,
                'is_target': bool(msg.is_target),
            },
            sent_timestamp=stamp_seconds(msg.header.stamp),
        )

    def on_path(self, msg):
        self.counts['global_path'] += 1
        path = [full_pose(pose.pose) for pose in msg.poses]
        if not path:
            return
        signature = tuple((round(p['x'], 2), round(p['y'], 2)) for p in path)
        if self.pending_goal is None or self.pending_goal['received_steady'] is None:
            return
        if signature == self.last_path_signature:
            return
        self.last_path_signature = signature
        self.path_id = f'path-{len(self.paths) + 1}'
        record = {
            'path_id': self.path_id,
            'timestamp': stamp_seconds(msg.header.stamp),
            'path': path,
            'path_length_m': path_length(path),
        }
        self.paths.append(record)
        self.event(
            'planner_path_generated',
            timestamp=record['timestamp'],
            path_id=self.path_id,
            object_id=self.pending_goal['object_id'],
            generated_timestamp=record['timestamp'],
            start_pose=self.takeover_pose,
            target_pose=self.pending_goal['target_pose'],
            path=path,
            path_length_m=record['path_length_m'],
            visualization_path='planner-path.png',
        )

    def on_waypoint(self, msg):
        self.counts['way_point'] += 1
        if self.pending_goal is None or self.pending_goal['received_steady'] is None:
            return
        if self.controller_received:
            return
        self.controller_received = True
        received = self.ros_now()
        self.event(
            'controller_path_received',
            timestamp=received,
            object_id=self.pending_goal['object_id'],
            path_received_timestamp=received,
            accepted=True,
            reject_reason=None,
            path_point_count=1,
            waypoint=point_pose(msg.point),
            planner_to_controller_latency_s=(
                time.monotonic() - self.pending_goal['received_steady']
            ),
        )

    def on_cmd(self, msg):
        self.counts['cmd_vel'] += 1
        velocity = {
            'linear_x': float(msg.twist.linear.x),
            'linear_y': float(msg.twist.linear.y),
            'angular_z': float(msg.twist.angular.z),
        }
        self.last_cmd = velocity
        magnitude = math.hypot(velocity['linear_x'], velocity['linear_y'])
        if self.pending_goal is not None and self.first_cmd is None and magnitude >= 0.05:
            self.first_cmd = {
                'timestamp': stamp_seconds(msg.header.stamp),
                'steady': time.monotonic(),
                'velocity': velocity,
            }

    def on_terrain(self, msg):
        now = time.monotonic()
        if now - self.last_terrain_sample < 5.0 or len(self.terrain_xy) >= 50000:
            return
        try:
            points = list(point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True))
            step = max(1, len(points) // 5000)
            self.terrain_xy.extend(
                (float(point[0]), float(point[1])) for point in points[::step]
            )
            self.last_terrain_sample = now
        except Exception:
            return

    def on_log(self, msg):
        marker = 'OBJNAV_TRACE '
        if marker in msg.msg:
            try:
                payload = json.loads(msg.msg.split(marker, 1)[1])
            except json.JSONDecodeError:
                return
            event = payload.pop('event', None)
            if event in ALLOWED_EVENTS:
                if event == 'object_memory_associated':
                    track_id = payload.get('track_id')
                    object_id = payload.get('object_id')
                    if track_id is not None and object_id is not None:
                        self.track_to_object[int(track_id)] = int(object_id)
                self.event(event, **payload)
            return
        if msg.name != 'tare_planner_node' or self.pending_goal is None:
            return
        found = re.search(r'(?:Found target object id|Update to a closer target object id):\s*(\d+)', msg.msg)
        rejected = re.search(r'Target object id\s+(\d+)\s+is out of bounds', msg.msg)
        if found:
            object_id = int(found.group(1))
            if self.pending_goal['received_steady'] is None:
                self.pending_goal['received_steady'] = time.monotonic()
                self.takeover_pose = dict(self.last_pose or {})
                self.event(
                    'planner_input_received',
                    object_id=object_id,
                    received_timestamp=self.ros_now(),
                    accepted=True,
                    reject_reason=None,
                    planner_start_pose=self.takeover_pose,
                    target_pose=self.pending_goal['target_pose'],
                    input_latency_s=(
                        self.pending_goal['received_steady'] - self.pending_goal['sent_steady']
                    ),
                )
        elif rejected:
            self.event(
                'planner_input_received',
                object_id=int(rejected.group(1)),
                received_timestamp=self.ros_now(),
                accepted=False,
                reject_reason='object_id_out_of_bounds',
                planner_start_pose=self.last_pose,
                target_pose=self.pending_goal['target_pose'],
                input_latency_s=time.monotonic() - self.pending_goal['sent_steady'],
                status='rejected',
            )
        elif 'Found the target object, waiting for next action' in msg.msg:
            self.goal_reached = True
            self.stop_reason = 'goal_reached'
            self.stop_source = 'planner'
            self.done = True

    def tick(self):
        now = time.monotonic()
        nodes = set(self.node.get_node_names())
        retry_due = self.last_prompt_at is None or now - self.last_prompt_at >= 5.0
        if (
            self.semantic_samples
            and not self.target_instruction_received
            and retry_due
            and 'vlm_node' in nodes
            and self.prompt_attempts < 3
        ):
            msg = String()
            msg.data = self.args.instruction
            self.prompt_pub.publish(msg)
            self.prompt_attempts += 1
            self.last_prompt_at = now
        if (
            self.pending_goal is not None
            and self.last_progress_pose is not None
            and math.hypot(self.last_cmd['linear_x'], self.last_cmd['linear_y']) >= 0.05
            and now - self.last_progress_steady >= self.args.stall_seconds
            and now - self.last_stall_steady >= self.args.stall_seconds
        ):
            self.event(
                'navigation_stalled',
                object_id=self.pending_goal['object_id'],
                robot_pose=self.last_pose,
                commanded_velocity=self.last_cmd,
                actual_velocity={'speed_mps': self.last_speed},
                no_progress_duration_s=now - self.last_progress_steady,
                remaining_path_m=(
                    self.paths[-1]['path_length_m'] if self.paths else None
                ),
                stall_threshold_s=self.args.stall_seconds,
                reason='vehicle_not_progressing_with_nonzero_command',
                status='failed',
            )
            self.last_stall_steady = now
        if self.elapsed() >= self.args.seconds:
            self.done = True

    def write_visibility(self):
        start_error = None
        if self.first_pose is not None:
            start_error = math.hypot(
                self.first_pose['x'] - self.args.start_x,
                self.first_pose['y'] - self.args.start_y,
            )
        visible = bool(
            self.semantic_samples
            and any(self.semantic_samples[0]['target_pixels'])
        )
        report = {
            'target_object': self.args.target,
            'fixed_start': {
                'x': self.args.start_x,
                'y': self.args.start_y,
                'z': self.args.start_z,
                'yaw': self.args.start_yaw,
            },
            'observed_initial_pose': self.first_pose,
            'start_xy_error_m': start_error,
            'ground_truth_target_pose': point_pose(self.args.target_point),
            'initial_target_visible': visible,
            'passed': bool(start_error is not None and start_error <= 0.05 and not visible),
            'semantic_frames': self.semantic_samples,
        }
        (self.run_dir / 'visibility.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2)
        )

    def write_path_outputs(self):
        path_file = {
            'path_generated': bool(self.paths),
            'reason': None if self.paths else 'planner_did_not_generate_target_path',
            'paths': self.paths,
            'executed_trajectory': self.trajectory,
        }
        (self.run_dir / 'planner-path.json').write_text(
            json.dumps(path_file, ensure_ascii=False, indent=2)
        )
        if not self.paths:
            return
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        figure, axis = plt.subplots(figsize=(10, 8))
        if self.terrain_xy:
            terrain = np.asarray(self.terrain_xy)
            axis.scatter(terrain[:, 0], terrain[:, 1], s=0.3, c='#b8b8b8', label='terrain/obstacle')
        for index, record in enumerate(self.paths):
            path = record['path']
            axis.plot(
                [point['x'] for point in path],
                [point['y'] for point in path],
                linewidth=2,
                label=f"planned path {record['path_id']}",
            )
        if self.trajectory:
            axis.plot(
                [point['x'] for point in self.trajectory],
                [point['y'] for point in self.trajectory],
                'k--',
                linewidth=1.2,
                label='executed trajectory',
            )
        markers = [
            ('start', self.first_pose, 'o', 'green'),
            ('YOLO detection', self.detection_pose, 's', 'orange'),
            ('planner takeover', self.takeover_pose, '^', 'blue'),
            ('goal', self.pending_goal['target_pose'] if self.pending_goal else None, '*', 'red'),
            ('final', self.final_pose, 'X', 'black'),
        ]
        for label, pose, marker, color in markers:
            if pose is not None:
                axis.scatter(pose['x'], pose['y'], marker=marker, s=90, c=color, label=label)
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
        axis.set_title(f'{self.run_dir.name} planner path')
        axis.axis('equal')
        axis.grid(True, alpha=0.3)
        axis.legend(loc='best')
        figure.tight_layout()
        figure.savefig(self.run_dir / 'planner-path.png', dpi=160)
        plt.close(figure)

    def close(self):
        if self.goal_reached:
            stop_reason = 'goal_reached'
            stop_source = 'planner'
        else:
            stop_reason = self.stop_reason
            stop_source = self.stop_source
        target_pose = self.pending_goal['target_pose'] if self.pending_goal else None
        distance = None
        if target_pose is not None and self.final_pose is not None:
            distance = math.dist(
                (target_pose['x'], target_pose['y'], target_pose['z']),
                (self.final_pose['x'], self.final_pose['y'], self.final_pose['z']),
            )
        completed = {event['event'] for event in self.events}
        expected = [
            'yolo_target_detected',
            'semantic_projection_ready',
            'object_memory_associated',
            'vlm_candidate_selected',
            'vlm_request_started',
            'vlm_submitted',
            'vlm_result',
            'planner_input_sent',
            'planner_input_received',
            'planner_path_generated',
            'controller_path_received',
            'motion_started',
        ]
        last_completed = self.events[-1]['event'] if self.events else 'episode_start'
        self.event(
            'episode_end',
            goal_pose=target_pose,
            final_pose=self.final_pose,
            goal_reached=self.goal_reached,
            distance_to_goal_m=distance,
            active_stop=True,
            stop_source=stop_source,
            stop_reason=stop_reason,
            success=self.goal_reached,
            last_completed_stage=last_completed,
            missing_stages=[name for name in expected if name not in completed],
            status='completed' if self.goal_reached else 'timeout',
        )
        self.output.close()
        self.write_visibility()
        self.write_path_outputs()
        event_times = {}
        for event in self.events:
            event_times.setdefault(event['event'], event['elapsed_s'])
        metrics = {
            'run_name': self.run_dir.name,
            'success': self.goal_reached,
            'stop_reason': stop_reason,
            'duration_s': round(self.elapsed(), 3),
            'event_times_s': event_times,
            'event_counts': dict(Counter(event['event'] for event in self.events)),
            'topic_counts': dict(self.counts),
            'travel_distance_m': round(sum(
                math.hypot(second['x'] - first['x'], second['y'] - first['y'])
                for first, second in zip(self.trajectory, self.trajectory[1:])
            ), 3),
            'demo_recorded': (self.run_dir / 'demo.mp4').exists(),
            'planner_path_generated': bool(self.paths),
        }
        (self.run_dir / 'metrics.json').write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2)
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--instruction', default='Find a toilet.')
    parser.add_argument('--target', default='toilet')
    parser.add_argument('--start-x', type=float, default=0.0)
    parser.add_argument('--start-y', type=float, default=0.0)
    parser.add_argument('--start-z', type=float, default=0.75)
    parser.add_argument('--start-yaw', type=float, default=0.0)
    parser.add_argument('--target-x', type=float, default=2.9565)
    parser.add_argument('--target-y', type=float, default=-12.301)
    parser.add_argument('--target-z', type=float, default=0.0)
    parser.add_argument('--seconds', type=float, default=300.0)
    parser.add_argument('--stall-seconds', type=float, default=8.0)
    args = parser.parse_args()
    args.target_point = type(
        'Point',
        (),
        {'x': args.target_x, 'y': args.target_y, 'z': args.target_z},
    )()
    rclpy.init()
    node = rclpy.create_node('objnav_episode_logger')
    logger = EpisodeLogger(node, args)
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True
        logger.stop_reason = 'manual_stop'
        logger.stop_source = 'test_harness'

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopped and not logger.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        logger.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
