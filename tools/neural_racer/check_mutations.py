"""Prove the neural-racer tests can fail (CLAUDE.md, Hard rule 1).

    PYTHONPATH=<torch> python3 tools/neural_racer/check_mutations.py [--only SUBSTR]

Copies the new packages into a temporary tree, applies one deliberate break
at a time -- the "break it with" column of the plan's test tables -- runs the
tests that should catch it, and requires them to FAIL. The real files are
never touched. Exit 0 only if every break is caught and the unbroken tree is
green.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NR = 'src/neural_racer/neural_racer'
IMU = 'src/racerbot_imu/racerbot_imu'
TR = 'tools/neural_racer/neural_racer_train'
T_NR = 'src/neural_racer/test'
T_IMU = 'src/racerbot_imu/test'
T_TR = 'tools/neural_racer/tests'

# (name, file, old, new, test node ids, needs ROS)
MUTATIONS = [
    ('pool mean not min', f'{NR}/observation.py', 'nearest = usable.min(axis=1)', 'nearest = usable.mean(axis=1)',
     [f'{T_NR}/test_observation.py::test_pool_keeps_minimum'], False),
    ('range_min inclusive', f'{NR}/observation.py', '& (r > range_min)', '& (r >= range_min)',
     [f'{T_NR}/test_observation.py::test_invalid_beam_values'], False),
    ('no beam-count check', f'{NR}/observation.py', 'if r.ndim != 1 or r.size != N_BEAMS:', 'if False:',
     [f'{T_NR}/test_observation.py::test_beam_count'], False),
    ('wrong gyro scale', f'{NR}/observation.py', "ego[EGO['gyro_z'].start + k] = gz / YAW_SCALE",
     "ego[EGO['gyro_z'].start + k] = gz / ACC_SCALE", [f'{T_NR}/test_observation.py::test_ego_layout_matches_spec'], False),
    ('IMU kept when missing', f'{NR}/observation.py', '    if imu_present:\n        if not history.imu_complete():',
     '    if True:\n        if not history.imu_complete():', [f'{T_NR}/test_observation.py::test_imu_missing_zeroes_imu_slots'], False),
    ('preview not mode-gated', f'{NR}/policy_numpy.py', "z_prev = _silu(self._lin('prev.fc', y.reshape(-1))) * mode",
     "z_prev = _silu(self._lin('prev.fc', y.reshape(-1)))", [f'{T_NR}/test_policy_numpy.py::test_reactive_mode_ignores_preview_and_race'], False),
    ('no tanh on steering', f'{NR}/policy_numpy.py', 'steering=float(obs.STEER_LIMIT * np.tanh(a[0]))',
     'steering=float(a[0])', [f'{T_NR}/test_policy_numpy.py::test_outputs_bounded_for_any_input'], False),
    ('GRU gate swapped', f'{NR}/policy_numpy.py', 'self.h = ((1.0 - zg) * n + zg * self.h)',
     'self.h = (zg * n + (1.0 - zg) * self.h)', [f'{T_NR}/test_policy_numpy.py::test_matches_naive_reference_over_a_sequence'], False),
    ('camera inside LayerNorm', f'{NR}/policy_numpy.py', 'z = np.concatenate([z_scan, z_ego, z_prev, z_race])\n        mu, var',
     'z = np.concatenate([z_scan, z_ego, z_prev, z_race, z_cam])[:-64]\n        mu, var = np.concatenate([z_scan, z_ego, z_prev, z_race, z_cam]).mean(), np.concatenate([z_scan, z_ego, z_prev, z_race, z_cam]).var()\n        _unused',
     [f'{T_NR}/test_policy_numpy.py::test_zero_initialised_camera_starts_identical'], False),
    ('no braking cap', f'{NR}/safety_wrapper.py', "caps['braking'] = gap_logic.braking_speed_limit(",
     "caps['braking'] = c.max_speed or gap_logic.braking_speed_limit(", [f'{T_NR}/test_safety_wrapper.py::test_braking_cap_matches_gap_logic'], False),
    ('curvature without tan', f'{NR}/safety_wrapper.py', 'kappa = abs(math.tan(steering)) / c.wheelbase',
     'kappa = abs(steering) / c.wheelbase', [f'{T_NR}/test_safety_wrapper.py::test_curvature_cap_closed_form'], False),
    ('invalid fraction >=', f'{NR}/safety_wrapper.py', 'if 1.0 - valid.mean() > c.max_invalid_fraction:',
     'if 1.0 - valid.mean() >= c.max_invalid_fraction - 0.001:', [f'{T_NR}/test_safety_wrapper.py::test_invalid_fraction_boundary'], False),
    ('no NaN stop', f'{NR}/safety_wrapper.py', "            return _stop('nan_output')", '            pass',
     [f'{T_NR}/test_safety_wrapper.py::test_non_finite_output_stops'], False),
    ('no hash check', f'{NR}/model_store.py', '    if actual != model_hash:', '    if False:',
     [f'{T_NR}/test_model_store.py::test_tampered_weights_refused'], False),
    ('shadow-only drives', f'{NR}/model_store.py', "    if driving and approval != 'full':", '    if False:',
     [f'{T_NR}/test_model_store.py::test_shadow_only_cannot_drive'], False),
    ('golden check skipped', f'{NR}/approve_model.py', '    if not err <= model_store.GOLDEN_ATOL:', '    if False:',
     [f'{T_NR}/test_model_store.py::test_golden_mismatch_refused'], False),
    ('deadman release ignored', f'{NR}/neural_racer_node.py', "        if not self.deadman_held:\n            return False, 'deadman_released'",
     "        if False:\n            return False, 'deadman_released'", [f'{T_NR}/test_neural_racer_node.py::test_lb_released_zero'], True),
    ('no joy = engaged', f'{NR}/neural_racer_node.py', "            return False, 'waiting_for_joy'", "            return True, None",
     [f'{T_NR}/test_neural_racer_node.py::test_joy_never_received_zero'], True),
    ('no joy timeout', f'{NR}/neural_racer_node.py', 'if self._age(self.last_joy_time) >= self.joy_timeout_sec:', 'if False:',
     [f'{T_NR}/test_neural_racer_node.py::test_joy_stale_zero'], True),
    ('no shadow guard', f'{NR}/neural_racer_node.py', 'if self.shadow and is_command_topic(self.shadow_topic):', 'if False:',
     [f'{T_NR}/test_neural_racer_node.py::test_shadow_topic_guard'], True),
    ('IMU not required', f'{NR}/neural_racer_node.py', 'if self.require_imu and not imu_fresh:', 'if False:',
     [f'{T_NR}/test_neural_racer_node.py::test_imu_stale_zero'], True),
    ('intent before command', f'{NR}/neural_racer_node.py',
     "        self._publish(res.steering, res.speed)                # the command, first",
     "        self._publish_intent(out, res, map_mode)\n        self._publish(res.steering, res.speed)",
     [f'{T_NR}/test_neural_racer_node.py::test_intent_published_after_command'], True),
    ('IMU scales swapped', f'{IMU}/imu_math.py', 'return (np.asarray(gyro_raw, dtype=np.float64) * gyro_scale,\n            np.asarray(accel_raw, dtype=np.float64) * accel_scale)',
     'return (np.asarray(gyro_raw, dtype=np.float64) * accel_scale,\n            np.asarray(accel_raw, dtype=np.float64) * gyro_scale)', [f'{T_IMU}/test_imu_math.py::test_to_si_vesc_units'], False),
    ('all-zero accepted', f'{IMU}/imu_math.py', '    if not (np.any(g != 0.0) or np.any(a != 0.0)):', '    if False:',
     [f'{T_IMU}/test_imu_math.py::test_all_zero_rejected'], False),
    ('gravity bound exclusive', f'{IMU}/imu_math.py', 'if not abs(norm - G0) <= tol + 1e-9:', 'if not abs(norm - G0) < tol - 1e-9:',
     [f'{T_IMU}/test_imu_math.py::test_gravity_check_bounds'], False),
    ('bias window not reset', f'{IMU}/imu_math.py', "            self._t, self._g, self._a = [], [], []\n            return False\n        self._t.append(t)",
     "            pass\n        self._t.append(t)", [f'{T_IMU}/test_imu_math.py::test_one_moving_tick_resets_the_window'], False),
    ('fault immediately', f'{IMU}/imu_crosscheck.py', 'return (t - self._since) < self.hold_sec - 1e-9, self.disagreement',
     'return False, self.disagreement', [f'{T_IMU}/test_imu_math.py::test_disagreement_needs_hold_time'], False),
    ('delay sign flipped', f'{IMU}/imu_crosscheck.py', 'lo, hi = t_truth_end - window + delay, t_truth_end + delay',
     'lo, hi = t_truth_end - window - delay, t_truth_end - delay', [f'{T_IMU}/test_imu_math.py::test_delay_alignment'], False),
    ('pid gain +10%', f'{TR}/gpu_sim/dynamics.py', 'fwd = torch.where(diff > 0, 10.0 * a_max / v_max,',
     'fwd = torch.where(diff > 0, 11.0 * a_max / v_max,', [f'{T_TR}/test_dynamics_parity.py::test_constraints_match_gym'], False),
    ('kinematic threshold', f'{TR}/gpu_sim/dynamics.py', "return torch.where((v < 0.5).unsqueeze(-1), kin, dyn)",
     "return torch.where((v < 0.4).unsqueeze(-1), kin, dyn)", [f'{T_TR}/test_dynamics_parity.py::test_derivative_matches_gym_both_branches'], False),
    ('servo clamp deleted', f'{TR}/gpu_sim/actuation.py', 'return torch.clamp(demand, -p.steering_rate_max, p.steering_rate_max)',
     'return demand', [f'{T_TR}/test_dynamics_parity.py::test_servo_matches_sim_fidelity'], False),
    ('torch pool mean', f'{TR}/gpu_sim/observation_torch.py', 'nearest = usable.min(dim=-1).values',
     'nearest = torch.where(torch.isinf(usable), torch.zeros_like(usable), usable).mean(dim=-1)',
     [f'{T_TR}/test_observation_parity.py::test_pool_scan_matches'], False),
    ('no oldest-repeat', f'{TR}/gpu_sim/observation_torch.py', 'buf[:, :, k] = torch.where(short, oldest, buf[:, :, k])', 'pass',
     [f'{T_TR}/test_observation_parity.py::test_history_and_build_match_over_a_sequence'], False),
    ('torch curvature no tan', f'{TR}/gpu_sim/safety_torch.py', 'kappa = torch.tan(steer).abs() / c.wheelbase',
     'kappa = steer.abs() / c.wheelbase', [f'{T_TR}/test_observation_parity.py::test_safety_matches_numpy_wrapper'], False),
    ('fault inverted', f'{TR}/gpu_sim/contact.py', 'fault[:, i] |= hit & (close_i > close_j)', 'fault[:, i] |= hit & (close_i < close_j)',
     [f'{T_TR}/test_sim.py::test_fault_rear_ender'], False),
    ('IMU lever arm dropped', f'{TR}/gpu_sim/imu.py', 'ax = vx_dot - w * vy - w * w * d', 'ax = vx_dot - w * vy',
     [f'{T_TR}/test_sim.py::test_imu_steady_turn_closed_form'], False),
    ('ray box axes swapped', f'{TR}/gpu_sim/lidar.py', 'tx1, tx2 = (-hx - lx) * inv_x, (hx - lx) * inv_x',
     'tx1, tx2 = (-hy - lx) * inv_x, (hy - lx) * inv_x', [f'{T_TR}/test_sim.py::test_ray_box_closed_form'], False),
    ('no place hysteresis', f'{TR}/gpu_sim/race.py', 'both & (d > 0.3)', 'both & (d > 0.0)',
     [f'{T_TR}/test_sim.py::test_place_hysteresis'], False),
    ('shared IMU delay bug', f'{TR}/gpu_sim/sim.py', 'imu = self.imu_delay.push(self.imu_held) if mask is None else self.imu_held',
     'imu = self.imu_delay.push(self.imu_held)', [f'{T_TR}/test_sim.py::test_resetting_one_env_does_not_disturb_another'], False),
    ('PFSP inverted', f'{TR}/matchmaking.py', "w = (1.0 - p) ** 2 if kind == 'hard' else p * (1.0 - p)",
     "w = p ** 2 if kind == 'hard' else p * (1.0 - p)", [f'{T_TR}/test_training_units.py::test_pfsp_hard_closed_form'], False),
    ('no scripted floor', f'{TR}/matchmaking.py', 'return max(floor, min(base, base * (1.0 - p_scripted) / 0.5))',
     'return min(base, base * (1.0 - p_scripted) / 0.5)', [f'{T_TR}/test_training_units.py::test_scripted_share_fades'], False),
    ('no uniform floor', f'{TR}/matchmaking.py', "return {'self': 0.35, 'pfsp': 0.40 + (0.15 - sc), 'scripted': sc, 'uniform': 0.10}",
     "return {'self': 0.35, 'pfsp': 0.50 + (0.15 - sc), 'scripted': sc, 'uniform': 0.0}", [f'{T_TR}/test_training_units.py::test_floor_keeps_everyone'], False),
    ('sampling ignores PFSP', f'{TR}/matchmaking.py', "out.append(OpponentRef('archive', archive_ids[int(rng.choice(len(archive_ids), p=w))]))",
     "out.append(OpponentRef('archive', archive_ids[int(rng.integers(len(archive_ids)))]))",
     [f'{T_TR}/test_training_units.py::test_sampling_follows_the_probabilities'], False),
    ('Wilson z = 1', f'{TR}/selection.py', 'Z95 = 1.959963984540054', 'Z95 = 1.0',
     [f'{T_TR}/test_selection.py::test_sequential_stops_when_clear'], False),
    ('no contact veto', f'{TR}/selection.py', "        if contacts.get(c, 0) > 0:", "        if False:",
     [f'{T_TR}/test_selection.py::test_veto_any_contact'], False),
    ('GAE without lambda', f'{TR}/ppo.py', 'last = delta + gamma * lam * boot * last', 'last = delta + gamma * boot * last',
     [f'{T_TR}/test_training_units.py::test_gae_hand_computed'], False),
    ('PLR replay inverted', f'{TR}/plr.py', 'if ids and self.rng.random() < self.replay_prob:', 'if ids and self.rng.random() > self.replay_prob:',
     [f'{T_TR}/test_training_units.py::test_plr_replay_ratio'], False),
    ('no radius check', f'{TR}/tracks.py', 'if np.any(kappa > 1.0 / cfg.min_radius + 1e-9):', 'if False:',
     [f'{T_TR}/test_training_units.py::test_check_catches_tight_corner'], False),
    ('held-out seed allowed', f'{TR}/learn.py', "    if seed >= HELDOUT_SEED_BASE:\n        raise ValueError('held-out seed requested for training')", "    pass",
     [f'{T_TR}/test_training_units.py::test_heldout_seed_refused_for_training'], False),
    ('log_std exported', f'{TR}/export.py', "EXCLUDE = ('log_std',)", 'EXCLUDE = ()',
     [f'{T_TR}/test_export_golden.py::test_state_dict_names_are_policy_numpy_names'], False),
    ('camera not zero-init', f'{TR}/model.py', '            self.fuse[\'fc\'].weight[:, ln:] = 0.0', '            pass',
     [f'{T_TR}/test_export_golden.py::test_camera_weights_start_at_zero'], False),
    ('wall contact not at fault', f'{TR}/referee.py', '    if best is None or best[0] > 0.8:\n        return True',
     '    if best is None or best[0] > 0.8:\n        return False', [f'{T_TR}/test_referee.py::test_wall_contact_is_flagged_and_at_fault'], False),
    ('SIGUSR1 exits 0', f'{TR}/common.py', 'return EXIT_DONE if finished else EXIT_CHECKPOINTED', 'return EXIT_DONE',
     [f'{T_TR}/test_training_units.py::test_sigusr1_checkpoints_and_exits_85'], False),
]

COPY = ('src/neural_racer', 'src/racerbot_imu', 'tools/neural_racer')
LINK = ('src/gap_follow', 'src/drive_intent', 'tools/f1tenth_sim', '.sim')


def build_tree(tmp: Path) -> None:
    for rel in COPY:
        shutil.copytree(ROOT / rel, tmp / rel, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    for rel in LINK:
        (tmp / rel).parent.mkdir(parents=True, exist_ok=True)
        os.symlink((ROOT / rel).resolve(), tmp / rel)


def run(tmp: Path, tests, ros: bool) -> int:
    # No bytecode cache: a break that keeps a file's size, written within the
    # same second as the last write, would otherwise run the *old* .pyc --
    # Python validates caches by size and whole-second mtime. (This checker
    # reported 'delay sign flipped' as missed for exactly that reason.)
    for cache in tmp.rglob('__pycache__'):
        if not cache.is_symlink() and tmp in cache.parents and not any(p.is_symlink() for p in cache.parents):
            shutil.rmtree(cache, ignore_errors=True)
    cmd = f'PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -x -q -p no:cacheprovider {" ".join(tests)}'
    if ros:
        cmd = ('source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash && '
               'export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && ' + cmd)
    return subprocess.run(['bash', '-c', cmd], cwd=tmp, capture_output=True, text=True, timeout=900).returncode


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--only', default='')
    args = ap.parse_args(argv)
    chosen = [m for m in MUTATIONS if args.only in m[0]]
    failures = []
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        build_tree(tmp)
        for name, rel, old, new, tests, ros in chosen:
            path = tmp / rel
            original = path.read_text()
            if original.count(old) != 1:
                failures.append(f'{name}: break text not found exactly once in {rel}')
                print(f'?? {name}: break text not found')
                continue
            if run(tmp, tests, ros) != 0:
                failures.append(f'{name}: tests fail on the UNBROKEN tree')
                print(f'!! {name}: unbroken tree is not green')
                continue
            path.write_text(original.replace(old, new))
            try:
                caught = run(tmp, tests, ros) != 0
            finally:
                path.write_text(original)
            print(f"{'ok' if caught else 'MISSED'}  {name}  ({', '.join(t.split('::')[1] for t in tests)})", flush=True)
            if not caught:
                failures.append(f'{name}: not caught by {tests}')
    print(f'\n{len(chosen) - len(failures)}/{len(chosen)} breaks caught')
    for f in failures:
        print('  -', f)
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
