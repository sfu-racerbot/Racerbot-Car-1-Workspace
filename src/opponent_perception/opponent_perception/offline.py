"""Generate an explicit synthetic example or replay timestamped images, without ROS."""

import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np
import yaml

from .configuration import load_config
from .detector import Camera, tag_corners
from .geometry import rigid_transform
from .pipeline import Pipeline


def read_json(line):
    if len(line.encode('utf-8')) > 65536:
        raise ValueError('manifest record exceeds 64 KiB')

    def reject_constant(value):
        raise ValueError(f'non-finite JSON constant {value}')
    try:
        value = json.loads(line, parse_constant=reject_constant)
    except json.JSONDecodeError as error:
        raise ValueError('invalid manifest JSON') from error
    if not isinstance(value, dict) or set(value) != {
            'stamp_sec', 'image', 'tracking_from_camera'}:
        raise ValueError('manifest record has missing or unknown keys')
    stamp = value['stamp_sec']
    if (type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < 0
            or not isinstance(value['image'], str) or not value['image']):
        raise ValueError('manifest requires nonnegative stamp and image path')
    rigid_transform(value['tracking_from_camera'])
    return value


def replay(config_path, camera_path, manifest_path, output_path):
    config = load_config(config_path)
    camera_data = yaml.safe_load(Path(camera_path).read_text(encoding='utf-8'))
    if not isinstance(camera_data, dict) or set(camera_data) != {
            'width', 'height', 'matrix', 'distortion'}:
        raise ValueError('camera YAML requires width, height, matrix, distortion')
    camera = Camera(**camera_data)
    pipeline = Pipeline(config)
    manifest = Path(manifest_path)
    # Never overwrite any input with replay output.
    output = Path(output_path)
    if output.resolve() in {Path(p).resolve() for p in (config_path, camera_path, manifest_path)}:
        raise ValueError('output must differ from replay inputs')
    records = []
    with manifest.open(encoding='utf-8', errors='strict') as source:
        for line in source:
            record = read_json(line)
            image_path = (manifest.parent / record['image']).resolve()
            if image_path == output.resolve():
                raise ValueError('output must differ from input images')
            image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f'cannot read image {image_path}')
            records.append(pipeline.process(image, camera, record['stamp_sec'],
                                            record['tracking_from_camera']))
    # Validate/process all inputs before opening output, avoiding partial-looking runs.
    with output.open('x', encoding='utf-8') as destination:
        for record in records:
            destination.write(json.dumps(record, allow_nan=False) + '\n')
    return records


def make_demo(directory):
    """Synthetic camera moving past a stationary opponent; no car calibration."""
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=False)
    camera = Camera(640, 480, np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]]),
                    np.zeros(5))
    opponent_from_tag = np.eye(4)
    opponent_from_tag[:3, :3] = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
    opponent_from_tag[:3, 3] = [0, 0, 0.3]
    config = {'schema_version': 1, 'calibrated': True, 'tracking_frame': 'odom',
              'max_error_px': 2.0, 'max_range_m': 8.0, 'tracking': {},
              'tags': [{'id': 7, 'vehicle_id': 'synthetic_opponent', 'size_m': 0.2,
                        'opponent_from_tag': opponent_from_tag.tolist()}]}
    (root / 'synthetic-config.yaml').write_text(yaml.safe_dump(config), encoding='utf-8')
    camera_data = {'width': camera.width, 'height': camera.height,
                   'matrix': camera.matrix.tolist(), 'distortion': camera.distortion.tolist()}
    (root / 'camera.yaml').write_text(yaml.safe_dump(camera_data), encoding='utf-8')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(dictionary, 7, 200)
    else:
        marker = cv2.aruco.drawMarker(dictionary, 7, 200)
    world_from_car = np.eye(4)
    world_from_car[0, 3] = 3.0
    optical_rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    manifest = []
    for index in range(11):
        angle = -0.05 + index * 0.01
        c, s = math.cos(angle), math.sin(angle)
        world_from_camera = np.eye(4)
        world_from_camera[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ optical_rotation
        world_from_camera[:3, 3] = [index * 0.04, index * 0.002, 0.3]
        camera_from_tag = np.linalg.inv(world_from_camera) @ world_from_car @ opponent_from_tag
        rotation = cv2.Rodrigues(camera_from_tag[:3, :3])[0]
        pixels = cv2.projectPoints(tag_corners(0.2), rotation, camera_from_tag[:3, 3],
                                   camera.matrix, camera.distortion)[0].reshape(4, 2)
        warp = cv2.getPerspectiveTransform(
            np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32),
            pixels.astype(np.float32))
        image = cv2.warpPerspective(marker, warp, (640, 480), flags=cv2.INTER_NEAREST,
                                    borderValue=255)
        name = f'frame-{index:03d}.png'
        if not cv2.imwrite(str(root / name), image):
            raise OSError('could not write demo image')
        manifest.append({'stamp_sec': 1.0 + index * 0.1, 'image': name,
                         'tracking_from_camera': world_from_camera.tolist()})
    (root / 'frames.jsonl').write_text(
        ''.join(json.dumps(row, allow_nan=False) + '\n' for row in manifest), encoding='utf-8')
    records = replay(root / 'synthetic-config.yaml', root / 'camera.yaml',
                     root / 'frames.jsonl', root / 'tracks.jsonl')
    positions = [t['position_m'] for record in records for t in record['tracks']]
    velocities = [t['velocity_mps'] for record in records for t in record['tracks']
                  if t['velocity_valid']]
    summary = {'synthetic_only': True, 'frames': len(records),
               'detected_frames': sum(bool(r['tracks']) for r in records),
               'position_rmse_m': float(np.sqrt(np.mean(np.sum(
                   (np.array(positions) - [3, 0])**2, axis=1)))) if positions else None,
               'stationary_speed_rmse_mps': float(np.sqrt(np.mean(np.sum(
                   np.array(velocities)**2, axis=1)))) if velocities else None}
    (root / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    demo = sub.add_parser('demo', help='render and detect synthetic tags with moving-camera truth')
    demo.add_argument('--output', required=True, help='new directory; never overwritten')
    play = sub.add_parser('replay',
                          help='replay PNG/JPEG frames and timestamped camera transforms')
    for name in ('config', 'camera', 'manifest', 'output'):
        play.add_argument('--' + name, required=True)
    options = parser.parse_args(args)
    try:
        if options.command == 'demo':
            print(json.dumps(make_demo(options.output), indent=2))
        else:
            rows = replay(options.config, options.camera, options.manifest, options.output)
            print(f'Processed {len(rows)} frames -> {options.output}')
    except (ValueError, OSError, UnicodeError, yaml.YAMLError) as error:
        parser.exit(2, f'perception: {error}\n')


if __name__ == '__main__':
    main()
