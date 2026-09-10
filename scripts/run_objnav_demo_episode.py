#!/usr/bin/env python3
"""Run and screen-record one complete SysNav simulation episode."""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


ROOT = Path('/home/myx/SysNav')
PANORAMA_TITLE = 'SysNav YOLO + SAM2 Panorama'
DISPLAY = ':1'
CAPTURE_SIZE = '2560x1440'
FPS = 10


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
import rclpy
import sam2
import yaml
from cv_bridge import CvBridge
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tare_planner.msg import TargetObjectInstruction


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


def make_rviz_config(path):
    source = ROOT / 'src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz'
    config = yaml.safe_load(source.read_text())
    config['Panels'] = []
    displays = config['Visualization Manager']['Displays']
    for display in displays:
        if display.get('Class') == 'rviz_default_plugins/Image':
            display['Enabled'] = False
            display['Value'] = False
        if display.get('Name') in {'OverallMap', 'Path', 'Waypoint', 'Boundary'}:
            display['Enabled'] = True
            display['Value'] = True
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
    geometry = config.setdefault('Window Geometry', {})
    geometry['Hide Left Dock'] = True
    geometry['Hide Right Dock'] = True
    geometry['X'] = 0
    geometry['Y'] = 854
    geometry['Width'] = 1280
    geometry['Height'] = 586
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
            'panorama': find_window(rows, pid=os.getpid(), title=PANORAMA_TITLE),
            'rviz': find_window(rows, pid=rviz_pid) or find_window(rows, title='RViz'),
            'unity': find_window(rows, pid=unity_pid),
        }
        if all(found.values()):
            move_window(found['panorama'], 0, 0, 2560, 854)
            move_window(found['rviz'], 0, 854, 1280, 586)
            move_window(found['unity'], 1280, 854, 1280, 586)
            time.sleep(1)
            return found, window_table()
        time.sleep(0.25)
    return found, window_table()


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
        'video_source': 'X11 synchronized dashboard plus /annotated_image',
        'video_resolution': CAPTURE_SIZE,
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
            '-screen-width', '1280', '-screen-height', '586',
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

        def receive_log(msg):
            nonlocal reached_at
            if (msg.name == 'tare_planner_node'
                    and 'Found the target object, waiting for next action' in msg.msg
                    and reached_at is None):
                reached_at = time.monotonic()

        node.create_subscription(Image, '/annotated_image', receive_panorama, 10)
        node.create_subscription(TargetObjectInstruction, '/target_object_instruction', receive_instruction, 10)
        node.create_subscription(Log, '/rosout', receive_log, 1000)
        prompt_pub = node.create_publisher(String, '/keyboard_input', 10)

        cv2.namedWindow(PANORAMA_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(PANORAMA_TITLE, 2560, 854)
        placeholder = cv2.imread(str(ROOT / 'src/base_autonomy/vehicle_simulator/mesh/unity/render.jpg'))
        placeholder = cv2.resize(placeholder, (1440, 480))
        cv2.putText(placeholder, 'Waiting for YOLO + SAM2 panorama...', (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 255), 3, cv2.LINE_AA)
        cv2.imshow(PANORAMA_TITLE, placeholder)
        cv2.waitKey(1)

        startup_deadline = time.monotonic() + 35
        windows, window_snapshot = arrange_windows(rviz.pid, unity.pid, startup_deadline)
        cv2.moveWindow(PANORAMA_TITLE, 0, 0)
        time.sleep(3)
        windows, window_snapshot = arrange_windows(rviz.pid, unity.pid, time.monotonic() + 5)
        cv2.moveWindow(PANORAMA_TITLE, 0, 0)
        if not process_alive(rviz):
            raise RuntimeError('rviz_exited_during_startup')
        if not windows.get('rviz'):
            raise RuntimeError('rviz_window_not_found')
        if not process_alive(unity) or not windows.get('unity'):
            raise RuntimeError('unity_window_not_found')
        if not windows.get('panorama'):
            raise RuntimeError('panorama_window_not_found')

        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        recorder = start('recorder', [
            ffmpeg, '-hide_banner', '-loglevel', 'warning', '-y',
            '-f', 'x11grab', '-draw_mouse', '0', '-video_size', CAPTURE_SIZE,
            '-framerate', str(FPS), '-i', f'{DISPLAY}+0,0',
            '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
            '-pix_fmt', 'yuv420p', str(run / 'demo.mp4'),
        ])

        started = time.monotonic()
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.03)
            now = time.monotonic()
            if last_panorama is not None:
                cv2.imshow(PANORAMA_TITLE, last_panorama)
            cv2.waitKey(1)

            if (last_panorama is not None and not target_instruction_received
                    and (last_prompt_at is None or now - last_prompt_at >= 5.0)
                    and 'vlm_node' in set(node.get_node_names()) and prompt_attempts < 3):
                prompt = String()
                prompt.data = args.instruction
                prompt_pub.publish(prompt)
                prompt_attempts += 1
                last_prompt_at = now

            required_processes = ['simulation', 'unity', 'rviz', 'recorder']
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
        cv2.destroyAllWindows()
        if failure == 'timeout' and process_alive(processes.get('logger')):
            os.killpg(processes['logger'].pid, signal.SIGUSR1)
            try:
                processes['logger'].wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        for name in ('recorder', 'logger', 'rviz', 'unity', 'simulation'):
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
    required = {
        'panorama_recorded': panorama_frames > 0 and video.exists(),
        'rviz_memory_view_recorded': bool(windows.get('rviz')) and video.exists(),
        'unity_third_person_recorded': bool(windows.get('unity')) and video.exists(),
    }
    validation = {
        **required,
        'rviz_started': bool(windows.get('rviz')),
        'rviz_process_started': 'rviz' in processes,
        'unity_process_started': 'unity' in processes,
        'panorama_frames_received': panorama_frames,
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
