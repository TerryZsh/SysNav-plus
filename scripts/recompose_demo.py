#!/usr/bin/env python3
"""Recompose a 1920x1080 SysNav demo with YOLO panorama as the main view."""

import argparse
import math
from pathlib import Path
import subprocess

import cv2
import imageio_ffmpeg
import numpy as np


WIDTH = 1920
HEIGHT = 1080
BACKGROUND = (18, 16, 14)
TEXT = (230, 235, 240)
CYAN = (225, 225, 55)

# Regions produced by the original three-panel recorder.
HEADER = (0, 0, 1920, 90)
MAP = (24, 134, 1080, 896)
UNITY = (1238, 134, 564, 423)
YOLO = (1144, 644, 752, 249)
TASK_STATUS = (1138, 557, 770, 35)
RUN_STATUS = (1138, 920, 770, 120)


def crop(frame, region):
    x, y, width, height = region
    return frame[y:y + height, x:x + width]


def fit(canvas, image, region):
    x, y, width, height = region
    scale = min(width / image.shape[1], height / image.shape[0])
    resized_width = max(1, round(image.shape[1] * scale))
    resized_height = max(1, round(image.shape[0] * scale))
    interpolation = cv2.INTER_CUBIC if scale > 1 else cv2.INTER_AREA
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=interpolation)
    offset_x = x + (width - resized_width) // 2
    offset_y = y + (height - resized_height) // 2
    canvas[offset_y:offset_y + resized_height, offset_x:offset_x + resized_width] = resized


def label(canvas, value, position, scale=0.58, color=TEXT):
    cv2.putText(
        canvas,
        value,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        1,
        cv2.LINE_AA,
    )


def compose(frame):
    if frame.shape[:2] != (HEIGHT, WIDTH):
        raise ValueError(f'Expected 1920x1080 input, got {frame.shape[1]}x{frame.shape[0]}')

    canvas = np.full((HEIGHT, WIDTH, 3), BACKGROUND, dtype=np.uint8)
    canvas[0:90, 0:1920] = crop(frame, HEADER)

    label(canvas, '01  YOLOE PANORAMA / INSTANCE MASKS', (24, 122), 0.62)
    fit(canvas, crop(frame, YOLO), (24, 138, 1296, 429))

    label(canvas, '02  UNITY SIMULATION', (1360, 122), 0.58)
    fit(canvas, crop(frame, UNITY), (1360, 138, 536, 401))

    label(canvas, '03  SPATIAL + OBJECT MEMORY', (24, 618), 0.58)
    fit(canvas, crop(frame, MAP), (24, 632, 540, 448))

    label(canvas, 'PIPELINE STATE', (624, 650), 0.58)
    fit(canvas, crop(frame, TASK_STATUS), (624, 674, 770, 35))
    fit(canvas, crop(frame, RUN_STATUS), (624, 746, 770, 120))
    label(canvas, 'YOLO panorama is shown at native 3:1 aspect ratio.', (624, 920), 0.48, CYAN)
    label(canvas, 'Unity and online memory remain synchronized companion views.', (624, 952), 0.48)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--preview', type=Path)
    args = parser.parse_args()

    if args.output.exists():
        parser.error(f'Output already exists: {args.output}')

    capture = cv2.VideoCapture(str(args.input))
    if not capture.isOpened():
        raise RuntimeError(f'Cannot open {args.input}')

    fps = capture.get(cv2.CAP_PROP_FPS)
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if fps <= 0 or frame_count <= 0:
        raise RuntimeError('Input video has invalid FPS or frame count')

    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoder = subprocess.Popen(
        [
            imageio_ffmpeg.get_ffmpeg_exe(),
            '-hide_banner',
            '-loglevel',
            'error',
            '-n',
            '-f',
            'rawvideo',
            '-pix_fmt',
            'bgr24',
            '-s',
            f'{WIDTH}x{HEIGHT}',
            '-r',
            str(fps),
            '-i',
            'pipe:0',
            '-an',
            '-c:v',
            'libx264',
            '-preset',
            'veryfast',
            '-crf',
            '20',
            '-pix_fmt',
            'yuv420p',
            '-movflags',
            '+faststart',
            str(args.output),
        ],
        stdin=subprocess.PIPE,
    )

    last = None
    processed = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            last = compose(frame)
            encoder.stdin.write(last.tobytes())
            processed += 1
    finally:
        capture.release()
        encoder.stdin.close()
        return_code = encoder.wait(timeout=max(30, math.ceil(frame_count / fps)))

    if return_code != 0 or processed != frame_count:
        raise RuntimeError(
            f'Encoding failed: return_code={return_code}, frames={processed}/{frame_count}'
        )

    preview = args.preview or args.output.with_suffix('.preview.jpg')
    cv2.imwrite(str(preview), last)
    print(f'Saved {args.output} ({processed} frames at {fps:g} FPS)')
    print(f'Saved {preview}')


if __name__ == '__main__':
    main()
