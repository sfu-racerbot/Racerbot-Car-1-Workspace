#!/usr/bin/env python3
"""Save the last /map in a run's bag as a map_server map (.pgm + .yaml, plus a .png).

    python3 tools/run_analysis/extract_map.py <run_dir>/bag <out_prefix>
    # e.g. ... data/rb1/racerbot_runs/20261026-194514/bag /tmp/live_194514
    #      -> /tmp/live_194514.pgm / .png / .yaml

The live maps in data/rb1/racerbot_runs/20261026-*/live_map/ were made this way
(2026-10-26 session; kept here 2026-10-31). The map is what slam_toolbox last
published live, not re-optimized. Needs ROS sourced (rosbag2_py, rclpy), numpy, Pillow.
"""
import sys

import numpy as np


def grid_to_image(grid):
    """OccupancyGrid values (-1, 0..100) -> uint8 map_server image in grid order:
    254 free (0), 0 occupied (>= 65), 205 unknown or uncertain."""
    g = np.asarray(grid, dtype=np.int16)
    img = np.full(g.shape, 205, np.uint8)
    img[g == 0] = 254
    img[g >= 65] = 0
    return img


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 2:
        print(__doc__.split('\n\n')[1], file=sys.stderr)
        return 2
    import rosbag2_py
    from nav_msgs.msg import OccupancyGrid
    from PIL import Image
    from rclpy.serialization import deserialize_message

    bag, out = argv
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/map']))
    last = None
    while r.has_next():
        _, last, _ = r.read_next()
    if last is None:
        print(f'{bag}: no /map in this bag', file=sys.stderr)
        return 1
    m = deserialize_message(last, OccupancyGrid)
    w, h = m.info.width, m.info.height
    img = np.flipud(grid_to_image(np.array(m.data).reshape(h, w)))  # pgm row 0 = top
    Image.fromarray(img).save(out + '.pgm')
    Image.fromarray(img).save(out + '.png')
    o = m.info.origin.position
    with open(out + '.yaml', 'w') as f:
        f.write(f"image: {out.split('/')[-1]}.pgm\nmode: trinary\nresolution: {m.info.resolution}\n"
                f"origin: [{o.x}, {o.y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f'{out}: {w} x {h} cells at {m.info.resolution} m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
