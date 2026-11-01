"""tools/check_mutations.py, on a toy tree whose answers are known.

Runner: python3 -m pytest tools/run_analysis/tests/ (with the tool suites; no ROS needed: setup=None).
Oracle: closed form. A test that checks add(2, 3) == 5 must catch '+' -> '-'; one that only checks that add is
callable must not; a break that is not in the file, or tests that already fail, prove nothing.
"""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # insert tools/
import check_mutations as cm  # noqa: E402

CALC = 'def add(a, b):\n    return a + b\n'
TESTS = '''import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mod'))
import calc


def test_strong():
    assert calc.add(2, 3) == 5


def test_hollow():
    assert callable(calc.add)


def test_already_red():
    assert calc.add(2, 3) == 6
'''


@pytest.fixture
def tree(tmp_path):
    (tmp_path / 'mod').mkdir()
    (tmp_path / 'mod/calc.py').write_text(CALC)
    (tmp_path / 'tests').mkdir()
    (tmp_path / 'tests/test_calc.py').write_text(TESTS)
    return tmp_path


def _spec(*mutations):
    return {'copy': ['mod', 'tests'], 'mutations': [
        {'file': 'mod/calc.py', 'old': old, 'new': new, 'tests': [f'tests/test_calc.py::{test}']}
        for old, new, test in mutations]}


def test_a_real_test_catches_its_break(tree):
    assert cm.check(_spec(('a + b', 'a - b', 'test_strong')), root=tree) == [('calc: a + b', 'caught')]


def test_a_hollow_test_is_reported_missed(tree):
    assert cm.check(_spec(('a + b', 'a - b', 'test_hollow')), root=tree)[0][1] == 'MISSED'


def test_a_break_that_is_not_in_the_file_proves_nothing(tree):
    assert cm.check(_spec(('a * b', 'a - b', 'test_strong')), root=tree)[0][1] == 'not-found'


def test_tests_that_fail_unbroken_prove_nothing(tree):
    assert cm.check(_spec(('a + b', 'a - b', 'test_already_red')), root=tree)[0][1] == 'red-baseline'


def test_the_real_files_are_never_touched(tree):
    cm.check(_spec(('a + b', 'a - b', 'test_strong'), ('a + b', 'a * b', 'test_hollow')), root=tree)
    assert (tree / 'mod/calc.py').read_text() == CALC


def test_only_selects_by_file_or_line(tree):
    spec = _spec(('a + b', 'a - b', 'test_strong'))
    assert cm.check(spec, root=tree, only='nothing like it') == []
    assert len(cm.check(spec, root=tree, only='calc')) == 1


@pytest.mark.parametrize('text,count', [('x = 1\nx = 1\n', 2), ('y = 2\n', 0)])
def test_a_break_must_occur_exactly_once(text, count):
    with pytest.raises(LookupError, match=f'found {count} times'):
        cm.apply(text, 'x = 1', 'x = 2')


def test_a_break_is_applied_where_it_is():
    assert cm.apply('a\nx = 1\nb\n', 'x = 1', 'x = 2') == 'a\nx = 2\nb\n'


@pytest.mark.parametrize('mutation,message', [
    ({'file': 'f', 'old': 'a', 'new': 'b'}, 'no tests'),
    ({'file': 'f', 'old': 'a', 'new': 'a', 'tests': ['t']}, 'changes nothing'),
])
def test_a_bad_spec_is_refused(tmp_path, mutation, message):
    path = tmp_path / 'spec.json'
    path.write_text(json.dumps({'copy': [], 'mutations': [mutation]}))
    with pytest.raises(ValueError, match=message):
        cm.load_spec(path)


def test_the_shipped_spec_loads_and_every_break_is_in_its_file():
    path = Path(__file__).resolve().parents[2] / 'mutations' / 'slam_reliability.json'
    spec = cm.load_spec(path)
    root = Path(__file__).resolve().parents[3]
    for mutation in spec['mutations']:
        cm.apply((root / mutation['file']).read_text(), mutation['old'], mutation['new'])
        for test in mutation['tests']:
            assert (root / test.split('::')[0]).is_file(), test
