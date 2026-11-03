import json
import pytest

from opponent_perception.offline import make_demo, replay


def test_image_to_tracks_replay_with_independent_stationary_truth(tmp_path):
    # oracle: measured synthetic rasterization plus invariant: known car stays at [3,0].
    # Threshold is a deliberately coarse acceptance envelope (10 cm / 0.5 m/s),
    # not a precision claim. Analytical geometry/velocity is tested separately.
    root = tmp_path / 'demo'
    summary = make_demo(root)
    assert summary['detected_frames'] == 11
    assert summary['position_rmse_m'] < .10  # m against independent renderer truth
    assert summary['stationary_speed_rmse_mps'] < .5  # m/s, small rasterized tags jitter
    rows = [json.loads(line) for line in (root / 'tracks.jsonl').read_text().splitlines()]
    assert rows[0]['tracks'][0]['velocity_valid'] is False
    assert rows[-1]['tracks'][0]['last_seen_sec'] == 2.0
    assert rows[-1]['tracks'][0]['vehicle_id'] == 'synthetic_opponent'
    repeated = replay(root / 'synthetic-config.yaml', root / 'camera.yaml',
                      root / 'frames.jsonl', root / 'second.jsonl')
    assert repeated == rows
    with pytest.raises(FileExistsError):
        replay(root / 'synthetic-config.yaml', root / 'camera.yaml',
               root / 'frames.jsonl', root / 'tracks.jsonl')


def test_replay_refuses_input_overwrite(tmp_path):
    # oracle: invariant: a user-provided image manifest must survive a bad output path.
    root = tmp_path / 'demo'
    make_demo(root)
    manifest = root / 'frames.jsonl'
    original = manifest.read_bytes()
    with pytest.raises(ValueError, match='differ'):
        replay(root / 'synthetic-config.yaml', root / 'camera.yaml', manifest, manifest)
    assert manifest.read_bytes() == original


def test_replay_reports_missing_image_and_does_not_write_output(tmp_path):
    # oracle: spec README.md: unreadable input fails explicitly, no success-looking partial output.
    root = tmp_path / 'demo'
    make_demo(root)
    lines = (root / 'frames.jsonl').read_text().splitlines()
    first = json.loads(lines[0])
    first['image'] = 'absent.png'
    (root / 'broken.jsonl').write_text(json.dumps(first) + '\n')
    output = root / 'failed.jsonl'
    with pytest.raises(ValueError, match='cannot read image'):
        replay(root / 'synthetic-config.yaml', root / 'camera.yaml', root / 'broken.jsonl', output)
    assert not output.exists()
