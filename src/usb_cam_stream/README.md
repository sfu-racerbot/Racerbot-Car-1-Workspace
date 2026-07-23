# `usb_cam_stream`

Serves a camera as a live MJPEG video stream over plain HTTP — open
`http://<car-ip>:9090/` in any browser to watch, no ROS install or plugins
needed on the viewing device. Car 1 uses its USB webcam captured directly over V4L2. The node also supports a generic `sensor_msgs/Image` topic when explicitly configured. Full
write-up (camera recommendations, wire format, security, parameter
reference):
[docs/usb-camera-livestream.md](../../docs/usb-camera-livestream.md).

## Files

| File | What it is |
|---|---|
| [`usb_cam_stream/camera_stream_node.py`](usb_cam_stream/camera_stream_node.py) | The node: frame source (OpenCV/V4L2 capture thread, or an image-topic subscription) + a Tornado web server serving the MJPEG stream. Small enough to keep in one file, same as `gap_follow`. |
| [`config/usb_cam_stream.yaml`](config/usb_cam_stream.yaml) | UVC-webcam variant: device path, resolution, FPS, JPEG quality, host/port. Change behavior here, not in the code. |
| [`launch/usb_cam_stream_launch.py`](launch/usb_cam_stream_launch.py) | Starts the node with the UVC-webcam YAML. |
| [`web/index.html`](web/index.html) | The entire browser side — a single `<img src="/stream">` tag, no JS needed. |
| `resource/usb_cam_stream` | Empty marker file required by `ament_python` — not code. |

## Interface

- **Publishes:** nothing, in either mode.
- **Subscribes:** nothing in the default V4L2 mode (talks to the camera
  directly, no ROS topics at all); only the configured `image_topic`
  (`sensor_msgs/Image`) in topic mode. Either way it never touches
  `/drive` or anything another node reads, so it's safe to run alongside
  any other node in this workspace, at any time.

## Running it

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch usb_cam_stream usb_cam_stream_launch.py    # Car 1 UVC webcam
```
then open `http://<car-ip>:9090/` (port `9090`, not `8080` — that's
already used by `web_dashboard` on this car), or just open the
[web dashboard](../../docs/web-dashboard.md) — its camera panel picks the
stream up automatically.
