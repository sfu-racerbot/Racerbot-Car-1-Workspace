"""What a run was made with: code, calibration and parameters. No rclpy.

A bag records what happened; this records what produced it, so a run can
be explained (and replayed) weeks later, after the code and the
calibration have moved on:

    <run>/metadata/git/<repo>.txt    HEAD, branch, `git status`, submodules
    <run>/metadata/git/<repo>.diff   every uncommitted change to tracked files
    <run>/metadata/files/...         copies of files read outside ROS
                                     parameters (imu-calibration's car.yaml
                                     and calibration JSONs)
    <run>/metadata/params/<node>.yaml  every node's parameters, written by
                                     snapshot_params a few seconds in

Recording must never stop a run: every failure here is written into the
metadata as text, not raised.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

GIT_TIMEOUT_S = 20.0


def find_repo_root(start):
    """The nearest directory at or above `start` holding a .git, or None."""
    path = Path(start).resolve()
    for candidate in (path, *path.parents):
        if (candidate / '.git').exists():
            return candidate
    return None


def _git(repo, *args):
    try:
        done = subprocess.run(['git', '-C', str(repo), *args], capture_output=True,
                              text=True, timeout=GIT_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired) as e:
        return f'<git {" ".join(args)} failed: {e}>'
    if done.returncode != 0:
        return f'<git {" ".join(args)} exited {done.returncode}: {done.stderr.strip()}>'
    return done.stdout


def write_git_state(repo, out_dir, name):
    """Write <name>.txt and <name>.diff for `repo` into out_dir. Returns the
    two paths. A missing or broken repository is written down, not raised."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    txt, diff = out_dir / f'{name}.txt', out_dir / f'{name}.diff'
    repo = Path(repo).expanduser() if repo is not None else None
    if repo is None or not (repo / '.git').exists():
        txt.write_text(f'not a git repository: {repo}\n')
        diff.write_text('')
        return txt, diff
    head = _git(repo, 'rev-parse', 'HEAD').strip()
    branch = _git(repo, 'rev-parse', '--abbrev-ref', 'HEAD').strip()
    txt.write_text(
        f'repository: {repo}\nHEAD: {head}\nbranch: {branch}\n\n'
        f'# git status --porcelain (?? = untracked: NOT in the .diff)\n'
        f'{_git(repo, "status", "--porcelain")}\n'
        f'# git submodule status\n{_git(repo, "submodule", "status")}')
    diff.write_text(_git(repo, 'diff', 'HEAD'))
    return txt, diff


def copy_files(paths, out_dir):
    """Copy each existing file into out_dir, keeping its name; returns
    {source: copy or reason it was not copied}."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    result = {}
    for path in paths:
        path = Path(path).expanduser()
        if not path.is_file():
            result[str(path)] = 'missing'
            continue
        target = out_dir / path.name
        try:
            shutil.copy2(path, target)
            result[str(path)] = str(target)
        except OSError as e:
            result[str(path)] = f'not copied: {e}'
    return result


TEAM_REPOS = ('racerbot_a', 'racerbot_b')


def team_repos(workspace_root):
    """{name: path} for the other teams' submodules checked out under
    <workspace>/src, so a run of their code records which commit of it ran.
    The superproject's own git state names only the submodule's commit, not
    the edits inside it. A submodule that is not checked out is left out."""
    if workspace_root is None:
        return {}
    src = Path(workspace_root) / 'src'
    return {name: src / name for name in TEAM_REPOS if (src / name / '.git').exists()}


def imu_calibration_files(racer_imu_root):
    """car.yaml and every calibration JSON under racer_imu_root/calib."""
    root = Path(racer_imu_root).expanduser()
    return [root / 'config' / 'car.yaml', *sorted((root / 'calib').glob('*.json'))]


def write_run_metadata(run_dir, repos, files):
    """repos: {name: path}; files: paths to copy. Writes <run_dir>/metadata/
    and returns it. Writes a manifest of what was and wasn't copied."""
    meta = Path(run_dir) / 'metadata'
    for name, repo in repos.items():
        write_git_state(repo, meta / 'git', name)
    copied = copy_files(files, meta / 'files')
    (meta / 'files' / 'MANIFEST.txt').write_text(
        ''.join(f'{src} -> {dst}\n' for src, dst in copied.items()))
    return meta


def params_tree(parameters):
    """{'a.b.c': v} -> {'a': {'b': {'c': v}}}, the shape `ros2 param dump`
    writes and `--params-file` reads. A name that is both a value and a
    prefix of others keeps its value under the dotted name it came with."""
    tree = {}
    for name in sorted(parameters):
        node = tree
        parts = name.split('.')
        for part in parts[:-1]:
            child = node.get(part)
            if not isinstance(child, dict):
                child = node[part] = {} if child is None else {'': child}
            node = child
        if isinstance(node.get(parts[-1]), dict):
            node[parts[-1]][''] = parameters[name]
        else:
            node[parts[-1]] = parameters[name]
    return tree


def node_file_name(full_name):
    """'/ns/node' -> 'ns__node.yaml' (a file name, never a path)."""
    cleaned = full_name.strip('/').replace('/', '__') or 'unnamed'
    return ''.join(c if c.isalnum() or c in '_-.' else '_' for c in cleaned) + '.yaml'


class NodeWatch:
    """Track snapshots across successive samples of full ROS node names.

    First appearances use node_file_name; after a successful snapshot, an
    absence and reappearance adds .1, .2, ... before .yaml. Absence before
    success does not advance the suffix. Colliding names receive the first
    free ~2, ~3, ... stem, retained across retries and restarts. Successful
    continuously present nodes are not returned again.

    A node that does not answer is backed off rather than retried every
    poll: after the k-th consecutive failure of one appearance, update()
    skips it for the next min(2**(k-1), 32) - 1 polls. A node that keeps
    failing is therefore returned on polls 1, 2, 4, 8, 16, 32, 64, 96, ...
    done() and an absence (a restart) reset the failure count.
    """

    def __init__(self, ignore=()):
        self.ignore = set(ignore)
        self._present = set()
        self._stems = {}
        self._owners = {}
        self._versions = {}
        self._done = set()
        self._failures = {}
        self._skip_polls = {}

    def update(self, names) -> list:
        """Return sorted (full_name, file_name) pairs needing a snapshot."""
        present = set(names) - self.ignore
        for name in self._present - present:
            self._failures.pop(name, None)
            self._skip_polls.pop(name, None)
            if name in self._done:
                self._done.remove(name)
                self._versions[name] += 1
        self._present = present
        result = []
        for name in sorted(present):
            if name not in self._stems:
                base = node_file_name(name)[:-5]
                stem = base
                k = 2
                while stem in self._owners:
                    stem = f'{base}~{k}'
                    k += 1
                self._stems[name] = stem
                self._owners[stem] = name
                self._versions[name] = 0
            if name in self._done:
                continue
            skip = self._skip_polls.get(name, 0)
            if skip:
                self._skip_polls[name] = skip - 1
                continue
            version = self._versions[name]
            suffix = f'.{version}' if version else ''
            result.append((name, f'{self._stems[name]}{suffix}.yaml'))
        return result

    def done(self, full_name) -> None:
        """Mark the current appearance successfully snapshotted."""
        self._done.add(full_name)
        self._failures.pop(full_name, None)
        self._skip_polls.pop(full_name, None)

    def failed(self, full_name) -> None:
        """Record a failed snapshot of the current appearance and back off.

        The k-th consecutive failure skips min(2**(k-1), 32) - 1 polls.
        """
        count = self._failures.get(full_name, 0) + 1
        self._failures[full_name] = count
        self._skip_polls[full_name] = min(2 ** (count - 1), 32) - 1
