"""The derivation rule, and the GNSS measurement model built on it.

The rule is stateless by design: every value is a pure function of
`(seed, stream, tick)`. Most of what is worth testing here is the *absence* of
things a sequential generator would have — order dependence, cross-stream
coupling, and state that has to be checkpointed.
"""

from __future__ import annotations

import math
import statistics

import pymap3d
import pytest

from keelson_sil_gnss.model import GnssConfig, gaussian_pair, observed_lat_lon, reports_fix

NOISY = GnssConfig(sigma_m=3.0)
# Gothenburg. Latitude matters: the metre to degree conversion is done at the
# vessel's actual position.
LAT, LON = 57.68, 11.83


# --- the derivation rule ---------------------------------------------------


def enu(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """East and north metres of `(lat, lon)` from `(lat0, lon0)`, on WGS84."""
    east, north, _ = pymap3d.geodetic2enu(lat, lon, 0.0, lat0, lon0, 0.0)
    return float(east), float(north)


def test_the_same_coordinate_always_gives_the_same_value():
    """The whole basis of reproducibility: seed, stream and tick decide the
    value and nothing else does."""
    assert gaussian_pair(42, "slipway-alpha/gnss", 100) == gaussian_pair(
        42, "slipway-alpha/gnss", 100
    )


def test_seed_stream_and_tick_each_change_the_value():
    base = gaussian_pair(42, "slipway-alpha/gnss", 100)
    assert gaussian_pair(43, "slipway-alpha/gnss", 100) != base
    assert gaussian_pair(42, "slipway-bravo/gnss", 100) != base
    assert gaussian_pair(42, "slipway-alpha/gnss", 101) != base


def test_a_stream_is_unaffected_by_any_other_stream():
    """The property a per-stream sequential generator loses quietly.

    Adding a vessel, or publishing in a different order, must not shift an
    existing vessel's values — otherwise "deterministic" holds only for the
    exact fleet the run was first written against, and the divergence surfaces
    months later as an unexplained replay mismatch.
    """
    alone = [gaussian_pair(7, "slipway-alpha/gnss", t) for t in range(50)]

    # Interleave draws from other streams, in a different order each time.
    crowded = []
    for tick in range(50):
        gaussian_pair(7, "slipway-bravo/gnss", tick)
        gaussian_pair(7, "traffic/mmsi_999001001", tick * 3)
        crowded.append(gaussian_pair(7, "slipway-alpha/gnss", tick))
        gaussian_pair(7, "vessel-charlie/gnss", tick)

    assert crowded == alone


def test_any_tick_can_be_drawn_without_drawing_the_ones_before_it():
    """What makes a restore at tick 500 correct without fast-forwarding
    through 499 discarded draws."""
    direct = gaussian_pair(11, "slipway-alpha/gnss", 500)
    after_walking = None
    for tick in range(501):
        after_walking = gaussian_pair(11, "slipway-alpha/gnss", tick)
    assert direct == after_walking


def test_the_stream_separator_cannot_be_forged():
    """`("a|b", 1)` and `("a", "b|1")` must not land on the same draw — an
    entity id cannot contain the separator, and the schema enforces that, but
    the rule should not depend on it."""
    assert gaussian_pair(1, "a|b", 1) != gaussian_pair(1, "a", 1)


def test_the_draws_are_actually_gaussian():
    """Box–Muller sanity: standard normals, not uniforms in disguise."""
    values = []
    for tick in range(4000):
        first, second = gaussian_pair(3, "s", tick)
        values += [first, second]

    assert statistics.fmean(values) == pytest.approx(0.0, abs=0.05)
    assert statistics.stdev(values) == pytest.approx(1.0, abs=0.05)
    # A uniform would never reach these; a normal reaches them rarely.
    assert max(values) > 3.0
    assert min(values) < -3.0


def test_no_draw_is_infinite():
    """`log(0)` is the one input Box–Muller cannot take, so the first uniform
    is drawn from (0, 1] rather than [0, 1)."""
    for tick in range(5000):
        for value in gaussian_pair(5, "s", tick):
            assert math.isfinite(value)


# --- the GNSS model --------------------------------------------------------


def test_a_receiver_with_no_declared_error_reports_truth_exactly():
    """sigma 0 must be exact, not nearly exact — it is how a test isolates a
    fault from the noise around it."""
    quiet = GnssConfig(sigma_m=0.0)
    assert observed_lat_lon("a", LAT, LON, quiet, 42, 7) == (LAT, LON)


def test_noise_moves_the_reported_position_but_not_by_much():
    lat, lon = observed_lat_lon("a", LAT, LON, NOISY, 42, 7)
    assert (lat, lon) != (LAT, LON)
    metres = math.dist(enu(lat, lon, LAT, LON), (0.0, 0.0))
    # Five sigma would mean the scale is wrong, not that the draw was unlucky.
    assert metres < 5 * NOISY.sigma_m


def test_the_error_has_the_declared_standard_deviation_in_metres():
    """The conversion is the part that can be silently wrong: a standard
    deviation is a distance, and degrees are not."""
    north, east = [], []
    for tick in range(3000):
        lat, lon = observed_lat_lon("a", LAT, LON, NOISY, 42, tick)
        e, n = enu(lat, lon, LAT, LON)
        north.append(n)
        east.append(e)

    for axis in (north, east):
        assert statistics.fmean(axis) == pytest.approx(0.0, abs=0.3)
        assert statistics.stdev(axis) == pytest.approx(NOISY.sigma_m, rel=0.06)


def test_the_error_is_circular_not_stretched_by_latitude():
    """Applying a metre sigma to degrees directly would make the east error
    shrink towards the poles. At 70 N that is a factor of nearly three, so a
    high-latitude exercise would silently get a different receiver.
    """
    east = []
    for tick in range(2000):
        lat, lon = observed_lat_lon("a", 70.0, 20.0, NOISY, 42, tick)
        east.append(enu(lat, lon, 70.0, 20.0)[0])
    assert statistics.stdev(east) == pytest.approx(NOISY.sigma_m, rel=0.06)


def test_two_vessels_at_the_same_place_report_different_positions():
    """Independent receivers, not one error applied to the fleet."""
    first = observed_lat_lon("slipway-alpha", LAT, LON, NOISY, 42, 7)
    second = observed_lat_lon("slipway-bravo", LAT, LON, NOISY, 42, 7)
    assert first != second


# --- losing lock --------------------------------------------------------------


def test_a_healthy_receiver_always_reports_and_a_dead_one_never_does():
    assert all(reports_fix(42, "a", tick, 1.0) for tick in range(500))
    assert not any(reports_fix(42, "a", tick, 0.0) for tick in range(500))


def test_duty_is_the_fraction_of_ticks_that_report():
    reported = sum(reports_fix(42, "a", tick, 0.3) for tick in range(5000))
    assert reported / 5000 == pytest.approx(0.3, abs=0.03)


def test_losing_lock_does_not_move_the_position_noise():
    """Its own stream: degrading lock must not shift where the fixes that do
    get out are reported."""
    before = [observed_lat_lon("a", LAT, LON, NOISY, 42, t) for t in range(50)]
    [reports_fix(42, "a", t, 0.5) for t in range(50)]
    assert [observed_lat_lon("a", LAT, LON, NOISY, 42, t) for t in range(50)] == before
