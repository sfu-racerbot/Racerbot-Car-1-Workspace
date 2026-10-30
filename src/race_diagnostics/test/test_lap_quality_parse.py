"""Supervisor quality-message contract; runner: pytest race_diagnostics/test/."""
import pytest

from race_diagnostics.run_events import parse_lap_quality


CEILING = 'Lap 5 did not pass the quality check (revolutions=2.04 > 1.50 (closed over roughly two overlapping laps, not one)), but the 5-lap ceiling is reached -- accepting it anyway rather than mapping indefinitely.'
MULTI = 'Lap 4 did not pass the quality check (heading_error=19.2deg > 15.0deg (50% of the 30deg gate); revolutions=2.09 > 1.50 (...)) and the 5-lap ceiling has not been reached -- ...'
SINGLE = 'Lap 3 did not pass the quality check (heading_error=29.5deg > 15.0deg (50% of the 30deg gate)) and the 5-lap ceiling has not been reached -- recording one more lap instead of accepting it. Keep holding LB.'


def test_revolutions_failure_accepted_at_ceiling():
    # oracle: spec, the verbatim supervisor message says failed and accepted.
    assert parse_lap_quality(CEILING) == dict(
        lap=5, passed=False, metric='revolutions', value=2.04, limit=1.50,
        accepted_at_ceiling=True)


def test_every_metric_is_preserved_in_order():
    # oracle: spec, both comparisons in the verbatim message must survive.
    assert parse_lap_quality(MULTI) == dict(
        lap=4, passed=False, metric='heading_error', value=19.2, limit=15.0,
        accepted_at_ceiling=False, metrics=[
            dict(metric='heading_error', value=19.2, limit=15.0),
            dict(metric='revolutions', value=2.09, limit=1.50)])


def test_single_degree_metric_keeps_existing_contract():
    # oracle: spec, backwards compatibility with test_lap_closed_lines.py.
    assert parse_lap_quality(SINGLE) == dict(
        lap=3, passed=False, metric='heading_error', value=29.5, limit=15.0,
        accepted_at_ceiling=False)


@pytest.mark.parametrize('line', [
    CEILING[:80], SINGLE.split('))')[0], 'garbage', '', None, 17, [], {},
    CEILING.encode(), CEILING.replace('2.04', 'NaN'),
    MULTI.replace('2.09', 'garbage'),
])
def test_incomplete_or_invalid_quality_message_is_not_a_verdict(line):
    # oracle: invariant, an incomplete/malformed message cannot be a verdict.
    assert parse_lap_quality(line) is None


def test_unlisted_metric_without_degree_unit():
    # oracle: spec, metric names are not restricted to today's known gates.
    line = SINGLE.replace('heading_error=29.5deg > 15.0deg', 'new_gate=21 > 20')
    assert parse_lap_quality(line) == dict(
        lap=3, passed=False, metric='new_gate', value=21.0, limit=20.0,
        accepted_at_ceiling=False)
