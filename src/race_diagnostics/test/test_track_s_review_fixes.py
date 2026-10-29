"""Track S regressions; runner: pytest src/race_diagnostics/test/."""
import importlib.util
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_error_file_failure_does_not_stop_other_metadata_writer(tmp_path, monkeypatch, capsys):
    from launch import LaunchContext
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'record_run.py'
    spec = importlib.util.spec_from_file_location('record_run_review', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    run = tmp_path / 'run'
    (run / 'metadata' / 'ERROR.txt').mkdir(parents=True)
    def fail_packages(meta):
        raise OSError('package failure')
    monkeypatch.setattr(module, 'write_packages', fail_packages)
    context = LaunchContext()
    context.launch_configurations.update({
        'run_directory': str(run), 'record_bag': 'false', 'name_prefix': '',
        'system_log': 'false', 'topics': ' '.join(module.DEFAULT_TOPICS)})
    actions = module._setup(context)
    # oracle: each metadata writer is independent and recording must continue.
    assert len(actions) == 3
    assert 'src/racerbot_a' in (run / 'metadata' / 'submodules.txt').read_text()
    assert 'package failure' in capsys.readouterr().err


def test_tegrastats_child_checks_parent_after_signal_registration():
    from race_diagnostics import system_log
    parent = os.getpid()
    # Old preexec only registered the signal; it never checked the parent.
    helper = getattr(system_log, '_prepare_tegrastats_child',
                     lambda expected: system_log._set_parent_death_signal())
    statuses = []
    for expected in (parent + 1, parent):
        pid = os.fork()
        if pid == 0:
            helper(expected)
            os._exit(0)
        _, status = os.waitpid(pid, 0)
        statuses.append(os.waitstatus_to_exitcode(status))
    # oracle: a changed parent must exit 1; the original parent continues.
    assert statuses == [1, 0]
