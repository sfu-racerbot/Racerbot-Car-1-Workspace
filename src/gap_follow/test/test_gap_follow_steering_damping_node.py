"""
Integration tests for steering damping from the IMU yaw rate, in the real
GapFollowNode (docs/gap-follow.md#steering-damping-gyro).

The term is gap_logic.steering_damping: minus the gain times the gyro's yaw
rate, added to the requested steering before the clip, the arc guard and the
slew limit. The gyro is trusted only while /imu/status says 'ok' and
/imu/data is fresh. Otherwise the term is exactly zero, and the node commands
exactly what it commands with the feature off.

GAIN here is 0.07 s, not the shipped 0.03 s (gap_follow.yaml), so the term is
large enough to measure against the undamped command. Where a test needs the
undamped command it gets it from a feature-off node run on the same scan, so
the oracle is the node's own feature-off behaviour, and the damping itself is
checked against the closed form -GAIN x yaw.

Needs ROS2 sourced and the package built, not a bare pytest:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/gap_follow/test/test_gap_follow_steering_damping_node.py -v
"""
import json
import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import Imu, Joy, LaserScan
from std_msgs.msg import String

from drive_intent import schema
from gap_follow.gap_follow_node import GapFollowNode

# Every node these tests build publishes its drive commands here, never on
# the real /drive (see test_gap_follow_node.py for why). It goes LAST in every
# rclpy.init: test_gap_follow_node.py's test_no_test_here_builds_a_node_on_the_
# real_drive checks every rclpy.init in this directory for it.
TEST_ONLY_DRIVE = ['-p', 'drive_topic:=/test_only/drive']

DEADMAN_BUTTON = 4
GAIN = 0.07           # the test gain; the shipped value is 0.03 (gap_follow.yaml)
MAX_STEERING = 0.26
# The slew limit and the clip are kept out of the way here, so the steering a
# test reads is the requested steering, except where a test says otherwise.
BASE = [
    '-p', 'min_gap_distance:=2.0',
    '-p', 'fallback_min_gap_distance:=0.8',
    '-p', 'min_speed:=0.8',
    '-p', 'max_speed:=2.0',
    '-p', f'max_steering_angle:={MAX_STEERING}',
    '-p', 'max_steering_rate:=100.0',
    # Every scan publishes an intent, so a test can read the reason it gives.
    '-p', 'intent_rate_hz:=0.0',
]
DAMPED = ['-p', 'enable_steering_damping:=true',
          '-p', f'steering_damping_gain:={GAIN}']


@pytest.fixture
def node():
    """A real GapFollowNode with steering damping ON at GAIN, in its own context.

    Nothing is fed yet: each test says what the deadman, odometry and IMU are."""
    rclpy.init(args=['--ros-args', *BASE, *DAMPED, *TEST_ONLY_DRIVE])
    n = GapFollowNode()
    assert n.drive_pub.topic_name == '/test_only/drive'
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _scan(ranges=None, n=541, span=math.pi):
    msg = LaserScan()
    msg.angle_min = -span / 2.0
    msg.angle_increment = span / (n - 1)
    msg.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
    msg.range_min, msg.range_max = 0.05, 12.0
    msg.ranges = [8.0] * n if ranges is None else list(ranges)
    return msg


def _blocked_right(n=541):
    """An obstacle filling the right half: a correct node steers left (positive)."""
    ranges = [8.0] * n
    for i in range(0, 250):
        ranges[i] = 1.0
    return ranges


def _hold_deadman(node, held=True):
    msg = Joy()
    msg.buttons = [0] * (DEADMAN_BUTTON + 1)
    msg.buttons[DEADMAN_BUTTON] = 1 if held else 0
    node.joy_callback(msg)


def _feed_odom(node, speed=0.0):
    msg = Odometry()
    msg.twist.twist.linear.x = float(speed)
    node.odom_callback(msg)


def _ready(node, speed=0.0):
    _hold_deadman(node)
    _feed_odom(node, speed)


def _feed_imu(node, yaw_rate=None, state='ok', age_sec=0.0):
    """What imu_bridge would send: one /imu/status verdict and one /imu/data
    sample. yaw_rate=None sends no sample; state=None sends no verdict.
    age_sec makes the sample that much older than the node's clock."""
    if state is not None:
        node.imu_status_callback(String(data=json.dumps(
            {'state': state, 'reason': None, 'imu': 'bno'})))
    if yaw_rate is not None:
        msg = Imu()
        msg.angular_velocity.z = float(yaw_rate)
        node.imu_callback(msg)
        if age_sec:
            node.last_imu_time = node.get_clock().now() - Duration(seconds=age_sec)


def _capture(node):
    published = []
    original = node.drive_pub.publish
    node.drive_pub.publish = lambda msg: (published.append(msg), original(msg))
    return published


def _capture_intent(node):
    """Record every payload the node puts on /drive_intent, decoded."""
    seen = []
    original = node.intent_pub.publish

    def spy(msg):
        seen.append(schema.decode(msg.data))
        return original(msg)

    node.intent_pub.publish = spy
    return seen


def _drive_once(extra, *, imu=None, ranges=None, speed=0.0):
    """One scan into a fresh node built with `extra` parameters, after LB is
    held and odometry says `speed`. `imu` is a dict of _feed_imu's keyword
    arguments, or None to give the node no IMU at all. Returns the drive
    commands it published and the intent payloads it put out."""
    rclpy.init(args=['--ros-args', *BASE, *extra, *TEST_ONLY_DRIVE])
    node = GapFollowNode()
    try:
        assert node.drive_pub.topic_name == '/test_only/drive'
        published = _capture(node)
        intents = _capture_intent(node)
        _ready(node, speed)
        if imu is not None:
            _feed_imu(node, **imu)
        node.scan_callback(_scan(_blocked_right() if ranges is None else ranges))
        return published, intents
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _steering(published):
    return published[-1].drive.steering_angle


# ============================================================================
# A healthy gyro: the command moves by exactly -GAIN x yaw
# ============================================================================

def test_a_healthy_imu_moves_the_command_by_minus_gain_times_yaw():
    # oracle: closed form on the difference. The feature-off run on the same
    # scan is the undamped command; the term at yaw 1.0 rad/s is -GAIN rad.
    undamped = _steering(_drive_once([])[0])
    published, _intents = _drive_once(DAMPED, imu=dict(yaw_rate=1.0))
    damped = _steering(published)
    assert damped == pytest.approx(undamped - GAIN * 1.0, abs=1e-9)  # rad
    assert damped < undamped, 'a left yaw rate must steer the command right'


def test_the_damped_request_is_clipped_with_the_rest_of_the_request():
    # The term joins the request before the clip (gap_logic's requested_steering
    # line), so a large term cannot push the command past max_steering_angle.
    # oracle: the configured limit. Added after the clip, a -5 rad/s yaw (term
    # +0.35 rad on top of the request) would publish about 0.6 rad.
    published, _intents = _drive_once(DAMPED, imu=dict(yaw_rate=-5.0))
    assert _steering(published) == pytest.approx(MAX_STEERING, abs=1e-9)  # rad


def test_a_healthy_imu_is_named_in_the_intent_reason():
    _published, intents = _drive_once(DAMPED, imu=dict(yaw_rate=1.0))
    reason = intents[-1].get('reason', '')
    assert 'steering damping' in reason
    assert 'steering_damping_off' not in reason


# ============================================================================
# A bad gyro: exactly zero damping, the feature-off command, and the reason
# ============================================================================

BAD_IMU = [
    pytest.param(dict(yaw_rate=1.0, state='fault'), 'imu fault', id='fault'),
    pytest.param(dict(yaw_rate=1.0, state='waiting'), 'imu waiting', id='waiting'),
    pytest.param(dict(yaw_rate=1.0, state='ok', age_sec=0.2), 'imu stale', id='stale'),
    pytest.param(dict(yaw_rate=None, state='ok'), 'no imu', id='no-data'),
    pytest.param(dict(yaw_rate=1.0, state=None), 'no imu', id='no-status'),
    pytest.param(dict(yaw_rate=None, state=None), 'no imu', id='never-received'),
    pytest.param(dict(yaw_rate=math.nan, state='ok'), 'imu yaw not finite', id='nan-yaw'),
]


@pytest.mark.parametrize('imu, why', BAD_IMU)
def test_a_bad_imu_adds_exactly_zero_damping_and_the_car_still_drives(imu, why):
    # Review Focus 5 (docs/gap-follow.md#steering-damping-gyro): with the feature
    # on and a bad gyro the command is the feature-off command, finite, and the
    # car keeps driving. The reason names what is wrong.
    undamped = _steering(_drive_once([])[0])
    published, intents = _drive_once(DAMPED, imu=imu)
    steering = _steering(published)
    assert math.isfinite(steering)
    assert steering == pytest.approx(undamped, abs=1e-12)  # rad: zero damping
    assert published[-1].drive.speed > 0.0, 'a bad gyro must not stop the car'
    assert f'steering_damping_off: {why}' in intents[-1].get('reason', '')


def test_with_damping_off_the_imu_is_not_read_at_all():
    # The feature is off, so a healthy gyro changes nothing and is not named.
    baseline, _ = _drive_once([])
    published, intents = _drive_once([], imu=dict(yaw_rate=1.0))
    assert _steering(published) == _steering(baseline)
    assert 'damping' not in intents[-1].get('reason', '')


# ============================================================================
# The deadman, with steering damping on: the four deny-path tests
# ============================================================================

def test_damping_on_still_drives_with_lb_held(node):
    published = _capture(node)
    _ready(node)
    _feed_imu(node, yaw_rate=1.0)
    node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed > 0.0
    assert node.last_decision_state == 'gap_follow'


def test_damping_on_refuses_to_drive_when_the_deadman_is_released(node):
    published = _capture(node)
    _ready(node)
    _feed_imu(node, yaw_rate=1.0)
    _hold_deadman(node, held=False)
    node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'deadman_released'


def test_damping_on_refuses_to_drive_before_any_joy_message(node):
    published = _capture(node)
    _feed_odom(node)
    _feed_imu(node, yaw_rate=1.0)
    node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'waiting_for_joy'


def test_damping_on_stops_when_the_joy_stream_goes_stale(node):
    published = _capture(node)
    _ready(node)
    _feed_imu(node, yaw_rate=1.0)
    node.last_joy_time = node.get_clock().now() - Duration(
        seconds=node.joy_timeout_sec + 0.1)
    node.scan_callback(_scan(_blocked_right()))
    assert published[-1].drive.speed == 0.0
    assert node.last_decision_state == 'joy_stale'


# ============================================================================
# Refusing a configuration that would steer with the rotation
# ============================================================================

def test_a_negative_gain_refuses_to_start():
    # oracle: spec -- docs/gap-follow.md#steering-damping-gyro: the gain is a
    # non-negative time constant. A negative one would make the term steer
    # with the rotation.
    rclpy.init(args=['--ros-args', *BASE, '-p', 'enable_steering_damping:=true',
                     '-p', 'steering_damping_gain:=-0.07', *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match='steering_damping_gain'):
            GapFollowNode()
    finally:
        rclpy.shutdown()


def test_a_non_positive_imu_timeout_refuses_to_start():
    # oracle: spec -- a timeout must be a positive time; zero would call every
    # sample stale and silently disable the term.
    rclpy.init(args=['--ros-args', *BASE, *DAMPED, '-p', 'imu_timeout_sec:=0.0',
                     *TEST_ONLY_DRIVE])
    try:
        with pytest.raises(ValueError, match='imu_timeout_sec'):
            GapFollowNode()
    finally:
        rclpy.shutdown()
