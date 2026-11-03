"""Same perception pipeline for offline image replay and the ROS adapter."""

from .detector import TagDetector
from .geometry import opponent_pose, rigid_transform
from .tracking import Tracker


class Pipeline:
    def __init__(self, config):
        self.config = config
        self.detector = TagDetector({i: t.size_m for i, t in config.tags.items()},
                                    config.max_error_px, config.max_range_m)
        self.tracker = Tracker(config.tracking_frame, config.tracking)

    def process(self, image, camera, stamp, tracking_from_camera):
        transform = rigid_transform(tracking_from_camera)
        detections = self.detector.detect(image, camera)
        observations = []
        for detection in detections:
            tag = self.config.tags[detection.tag_id]
            pose = opponent_pose(transform, detection.camera_from_tag, tag.opponent_from_tag)
            observations.append((tag.vehicle_id, detection.tag_id, pose,
                                 detection.reprojection_error_px))
        self.tracker.observe(stamp, observations)
        return self.tracker.snapshot(stamp)
