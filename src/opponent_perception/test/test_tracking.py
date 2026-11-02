import math
import numpy as np
import pytest

from opponent_perception.geometry import opponent_pose, transform_from_pose
from opponent_perception.tracking import Tracker, TrackingParams


def observation(x, y=0, vehicle='rb1', yaw=0):
    pose = transform_from_pose([x, y, 0], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])
    return vehicle, 7, pose, 0.1


def test_known_velocity_extrapolation_and_prediction():
    # oracle: closed form x(t) = 3 + 2*(t-1), y(t) = -t+1.
    tracker = Tracker()
    tracker.observe(1, [observation(3)])
    first = tracker.snapshot(1)['tracks'][0]
    assert first['velocity_mps'] is None and first['prediction'] == []
    tracker.observe(1.125, [observation(3.25, -.125, yaw=-.2)])
    track = tracker.snapshot(1.25)['tracks'][0]
    assert track['velocity_mps'] == pytest.approx([2, -1], abs=1e-9)
    assert track['position_m'] == pytest.approx([3.5, -.25], abs=1e-9)
    assert track['prediction'][-1]['position_m'] == pytest.approx([5.5, -1.25], abs=1e-9)
    assert track['last_seen_sec'] == 1.125
    assert track['yaw_rad'] == pytest.approx(-.2, abs=1e-9)


def test_ego_translation_and_turn_do_not_produce_opponent_velocity():
    # oracle: invariant: stationary world car remains stationary under moving camera transforms.
    tracker = Tracker()
    car = transform_from_pose([3, -1, 0], [0, 0, 0, 1])
    for index, angle in enumerate([0, .1, .2]):
        ego = transform_from_pose([angle, angle*.5, .3],
                                  [0, 0, math.sin(angle/2), math.cos(angle/2)])
        fixed_pose = opponent_pose(ego, np.linalg.inv(ego) @ car, np.eye(4))
        tracker.observe(1 + index*.125, [('rb1', 7, fixed_pose, .1)])
    assert tracker.snapshot(1.25)['tracks'][0]['velocity_mps'] == pytest.approx([0, 0], abs=1e-9)


def test_fresh_and_lost_boundaries_and_uncertainty_growth():
    # oracle: spec README.md: fresh <= .25 s, retained <= .75 s, never refresh last_seen on misses.
    tracker = Tracker()
    tracker.observe(0, [observation(3)])
    assert tracker.snapshot(.25)['tracks'][0]['fresh'] is True
    assert tracker.snapshot(.250001)['tracks'][0]['fresh'] is False
    stale = tracker.snapshot(.75)['tracks'][0]
    assert stale['uncertainty_m'] == pytest.approx(.05 + .5*.75, abs=1e-12)  # m
    assert tracker.snapshot(.750001)['tracks'] == []
    tracker.observe(1, [observation(4)])
    assert tracker.snapshot(1)['tracks'][0]['velocity_mps'] is None


def test_velocity_filter_and_speed_gate_do_not_refresh_an_outlier():
    # oracle: closed form EMA, then speed exactly at limit accepted; above it rejected.
    tracker = Tracker(params=TrackingParams(velocity_gain=.25))
    tracker.observe(0, [observation(0)])
    tracker.observe(.125, [observation(1)])  # exactly 8 m/s
    tracker.observe(.25, [observation(1.5)])  # 4 m/s -> .25*4 + .75*8 = 7
    assert tracker.snapshot(.25)['tracks'][0]['velocity_mps'] == pytest.approx([7, 0], abs=1e-9)
    tracker.observe(.375, [observation(2.50001)])
    assert tracker.snapshot(.375)['tracks'][0]['last_seen_sec'] == .25


@pytest.mark.parametrize('stamp', [0, -1, math.nan, math.inf])
def test_invalid_or_repeated_stamp_cannot_modify_track(stamp):
    # oracle: invariant: rejected batches leave the last accepted observation intact.
    tracker = Tracker()
    tracker.observe(0, [observation(3)])
    with pytest.raises(ValueError, match='stamp'):
        tracker.observe(stamp, [observation(4)])
    assert tracker.snapshot(0)['tracks'][0]['position_m'] == [3, 0]


def test_multiple_ids_misses_and_reset():
    # oracle: invariant: identity is stable across list reordering, misses age only that track.
    tracker = Tracker()
    tracker.observe(0, [observation(1), observation(2, vehicle='rb2')])
    tracker.observe(.125, [observation(2.125, vehicle='rb2')])
    rows = tracker.snapshot(.125)['tracks']
    assert [t['vehicle_id'] for t in rows] == ['rb1', 'rb2']
    assert [t['last_seen_sec'] for t in rows] == [0, .125]
    assert rows[1]['velocity_mps'] == pytest.approx([1, 0], abs=1e-9)
    tracker.clear()
    tracker.observe(0, [])
    assert tracker.snapshot(0)['tracks'] == []


def test_malformed_batch_is_atomic():
    # oracle: invariant: a duplicate vehicle cannot partially mutate accepted state.
    tracker = Tracker()
    tracker.observe(0, [observation(1)])
    with pytest.raises(ValueError, match='duplicate'):
        tracker.observe(.125, [observation(2), observation(3)])
    assert tracker.last_stamp == 0
    assert tracker.snapshot(0)['tracks'][0]['position_m'] == [1, 0]


@pytest.mark.parametrize('now', [-1, math.nan, math.inf, .5])
def test_snapshot_refuses_invalid_or_past_publication_time(now):
    # oracle: spec README.md: publication timestamp cannot precede accepted image.
    tracker = Tracker()
    tracker.observe(1, [])
    with pytest.raises(ValueError, match='publication'):
        tracker.snapshot(now)


@pytest.mark.parametrize('values', [{'velocity_gain': 1.01}, {'fresh_sec': 1},
                                    {'lost_sec': 0}, {'horizon_sec': 3},
                                    {'step_sec': .00001}, {'max_speed_mps': math.inf}])
def test_invalid_tracking_parameters(values):
    # oracle: spec README.md: bounded prediction and finite positive parameters.
    with pytest.raises(ValueError, match='tracking'):
        TrackingParams(**values)
