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

from keelson_sil_gnss.model import gaussian_pair, offset_lat_lon, reports_fix

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


# --- losing lock --------------------------------------------------------------


def test_a_healthy_receiver_always_reports_and_a_dead_one_never_does():
    assert all(reports_fix(42, "a", tick, 1.0) for tick in range(500))
    assert not any(reports_fix(42, "a", tick, 0.0) for tick in range(500))


def test_duty_is_the_fraction_of_ticks_that_report():
    reported = sum(reports_fix(42, "a", tick, 0.3) for tick in range(5000))
    assert reported / 5000 == pytest.approx(0.3, abs=0.03)


def test_losing_lock_draws_on_its_own_stream():
    """Degrading lock must not shift the position jitter, which draws on
    `<entity>/gnss`: lock draws on `<entity>/gnss-duty`."""
    before = [gaussian_pair(42, "a/gnss", t) for t in range(50)]
    [reports_fix(42, "a", t, 0.5) for t in range(50)]
    assert [gaussian_pair(42, "a/gnss", t) for t in range(50)] == before


# --- the ellipsoid ------------------------------------------------------------


def test_an_offset_is_metres_on_the_ground_at_any_latitude():
    """Applying metres to degrees directly would make the east error shrink
    towards the poles — at 70 N by a factor of nearly three."""
    for lat in (0.0, 57.68, 70.0):
        moved = offset_lat_lon(lat, 20.0, 100.0, -50.0)
        east, north = enu(*moved, lat, 20.0)
        assert east == pytest.approx(100.0, abs=1e-6)
        assert north == pytest.approx(-50.0, abs=1e-6)
