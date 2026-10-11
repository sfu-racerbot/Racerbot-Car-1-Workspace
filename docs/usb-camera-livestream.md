# Camera livestream on car 1 (moved)

> **Who this is for:** anyone who followed a link here looking for `usb_cam_stream`, the camera's MJPEG stream.
> **Read first:** nothing.
> **You'll be able to:** find the stream's real doc, and start car 1's camera.

`usb_cam_stream` moved to **[sfu-racerbot/web-dashboards](https://github.com/sfu-racerbot/web-dashboards)** on 2026-09-27, included here as the `src/web_dashboards` submodule. The package name did not change. The full doc — cameras, the two stream tiers, `image_topic` mode, every parameter, the security note — is **[web-dashboards/car/docs/usb-camera-livestream.md](../src/web_dashboards/car/docs/usb-camera-livestream.md)**.

**Car 1** streams a plain UVC USB webcam at `/dev/video0` — it has **no RealSense** (car 2 does, and streams its colour topic instead). Car 1's camera setting (`device`) is in the `usb_cam_stream_node` section of [`src/racerbot_launch/config/web_dashboard_rb1.yaml`](../src/racerbot_launch/config/web_dashboard_rb1.yaml); everything else (1280×720, 30 FPS capture, port 9090, the two stream tiers) is the package's own generic webcam defaults. Find the device path with `v4l2-ctl --list-devices` if it ever enumerates elsewhere.

**Terminal 1, from `~/racerbot-ws`:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_launch webcam_launch.py
```

**Working when:** the camera inset on https://dashboard.sfuracerbot.ca (pick **Car 1**) fills in within a few seconds, or `http://<car-ip>:9090/` shows the picture on the car's own network. With no webcam plugged in it keeps running and logs that it is waiting for the device.

It publishes nothing and cannot move the car. More car-1 detail: [web-dashboard.md](web-dashboard.md) and [hardware-reference.md](hardware-reference.md#camera--uvc-usb-webcam).
