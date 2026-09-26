"""What a receiver of each grade reports, faults and all.

The first model drew a fresh error on every fix, so a 10 Hz receiver made a
vessel jump metres every tenth of a second. These hold the replacement to
what a receiver actually does: its error wanders over minutes, its accuracy
follows its satellites, and each fault degrades it the way that fault does.
"""

from __future__ import annotations

import itertools
import math
import statistics

import pymap3d
import pytest

from keelson_sil_gnss.solution import (
    FIX_3D,
    FIX_NO,
    GRADES,
    INTEGRITY_SAFE,
    NS,
    POS_RTK_FLOAT,
    POS_RTK_INT,
    Faults,
    custom_grade,
    solve,
    wander,
)

LAT, LON = 57.68, 11.83
EPOCH = 1_767_772_800_000_000_000
DT = 100_000_000  # a 10 Hz simulator


def run(grade, faults=None, seconds=600.0, entity="a", seed=42):
    """(east, north) errors in metres, one per tick."""
    faults = faults or Faults()
    out = []
    for tick in range(int(seconds * NS / DT)):
        s = solve(entity, LAT, LON, grade, faults, seed, tick, EPOCH + tick * DT)
        e, n, _ = pymap3d.geodetic2enu(s.latitude_deg, s.longitude_deg, 0, LAT, LON, 0)
        out.append((float(e), float(n)))
    return out


# --- the error wanders rather than jumps --------------------------------------


@pytest.mark.parametrize("name", sorted(GRADES))
def test_consecutive_fixes_sit_close_together(name):
    """The point of the change: a step between fixes is the jitter, not the
    whole error again. Independent errors with per-axis spread s step a median
    of about 1.66 s (a Rayleigh median of a difference); a receiver steps well
    under half that."""
    grade = GRADES[name]
    errors = run(grade, seconds=120)
    steps = [math.dist(a, b) for a, b in itertools.pairwise(errors)]
    spread = math.hypot(grade.sigma_corr_m, grade.sigma_white_m)
    assert statistics.median(steps) < 0.5 * 1.66 * spread


@pytest.mark.parametrize("name", sorted(GRADES))
def test_over_a_long_run_the_error_has_the_grades_spread(name):
    """Many independent receivers, so the wander's slow timescale does not
    make the sample small."""
    grade = GRADES[name]
    east = [
        run(grade, seconds=1, entity=f"v{i}")[0][0]
        for i in range(400)
    ]
    expected = math.hypot(grade.sigma_corr_m, grade.sigma_white_m)
    assert statistics.pstdev(east) == pytest.approx(expected, rel=0.15)


def test_the_grades_are_ordered_by_accuracy():
    spreads = {n: math.hypot(g.sigma_corr_m, g.sigma_white_m) for n, g in GRADES.items()}
    assert spreads["recreational"] > spreads["professional"] > spreads["survey"]
    assert spreads["survey"] < 0.02, "RTK fixed is centimetre level"


def test_the_wander_has_unit_variance_between_knots_as_well_as_on_them():
    """A plain blend of two independent values has less spread mid-way; the
    renormalisation is what stops the error breathing in and out every tau."""
    mid = [wander(7, f"s{i}", 5 * NS, 10.0) for i in range(4000)]
    on = [wander(7, f"s{i}", 0, 10.0) for i in range(4000)]
    assert statistics.pstdev(mid) == pytest.approx(1.0, rel=0.06)
    assert statistics.pstdev(on) == pytest.approx(1.0, rel=0.06)


def test_the_same_instant_always_gives_the_same_solution():
    g = GRADES["professional"]
    first = solve("a", LAT, LON, g, Faults(), 42, 17, EPOCH + 17 * DT)
    assert solve("a", LAT, LON, g, Faults(), 42, 17, EPOCH + 17 * DT) == first
    assert solve("b", LAT, LON, g, Faults(), 42, 17, EPOCH + 17 * DT) != first


def test_a_custom_sigma_is_the_total_per_axis_spread():
    grade = custom_grade(2.5)
    assert math.hypot(grade.sigma_corr_m, grade.sigma_white_m) == pytest.approx(2.5)


# --- satellites, geometry and what the receiver says about itself -------------


@pytest.mark.parametrize("name", sorted(GRADES))
def test_a_healthy_receiver_has_a_3d_fix_on_plenty_of_satellites(name):
    grade = GRADES[name]
    s = solve("a", LAT, LON, grade, Faults(), 42, 0, EPOCH)
    assert s.fix_type == FIX_3D and s.integrity == INTEGRITY_SAFE
    assert grade.visible[0] <= s.satellites_visible <= grade.visible[1]
    assert 4 <= s.satellites_used <= s.satellites_visible
    assert 0.4 < s.hdop < 1.5 and s.pdop > s.hdop


def test_a_masked_sky_uses_fewer_satellites_and_worsens_geometry():
    g = GRADES["professional"]
    clear = solve("a", LAT, LON, g, Faults(), 42, 0, EPOCH)
    masked = solve("a", LAT, LON, g, Faults(sky_mask=0.6), 42, 0, EPOCH)
    assert masked.satellites_used < clear.satellites_used
    assert masked.hdop > clear.hdop
    assert masked.sigma_east_m > clear.sigma_east_m


def test_jamming_drops_satellites_grows_the_error_then_takes_the_fix():
    g = GRADES["recreational"]
    levels = [solve("a", LAT, LON, g, Faults(jamming=j), 42, 0, EPOCH) for j in (0, 0.4, 0.8, 1.0)]
    used = [s.satellites_used for s in levels]
    assert used == sorted(used, reverse=True) and used[0] > used[2]
    assert levels[1].sigma_east_m > levels[0].sigma_east_m
    assert levels[-1].fix_type == FIX_NO and not levels[-1].has_fix


def test_hard_jamming_or_lost_corrections_drop_rtk_fixed_to_float():
    g = GRADES["survey"]
    assert solve("a", LAT, LON, g, Faults(), 42, 0, EPOCH).pos_type == POS_RTK_INT
    lost = solve("a", LAT, LON, g, Faults(corrections=False), 42, 0, EPOCH)
    jammed = solve("a", LAT, LON, g, Faults(jamming=0.6), 42, 0, EPOCH)
    assert lost.pos_type == jammed.pos_type == POS_RTK_FLOAT
    assert lost.sigma_east_m > 5 * g.sigma_corr_m


def test_multipath_adds_error_the_receiver_admits_to():
    g = GRADES["professional"]
    clean = run(g, seconds=60)
    reflected = run(g, Faults(multipath=1.0), seconds=60)
    assert statistics.pstdev(e for e, _ in reflected) > statistics.pstdev(e for e, _ in clean)
    assert (
        solve("a", LAT, LON, g, Faults(multipath=1.0), 42, 0, EPOCH).sigma_east_m
        > solve("a", LAT, LON, g, Faults(), 42, 0, EPOCH).sigma_east_m
    )


def test_spoofing_moves_the_fix_while_the_receiver_believes_it():
    """The receiver's estimate and integrity are unchanged: that is the attack."""
    g = GRADES["professional"]
    spoof = Faults(spoof_offset_m=200.0, spoof_bearing_deg=90.0)
    honest = solve("a", LAT, LON, g, Faults(), 42, 0, EPOCH)
    fooled = solve("a", LAT, LON, g, spoof, 42, 0, EPOCH)
    east, north, _ = pymap3d.geodetic2enu(fooled.latitude_deg, fooled.longitude_deg, 0, LAT, LON, 0)
    assert east == pytest.approx(200.0, abs=3.0) and abs(north) < 3.0
    assert fooled.sigma_east_m == honest.sigma_east_m
    assert fooled.integrity == INTEGRITY_SAFE and fooled.fix_type == FIX_3D


def test_a_spoofing_drift_walks_the_fix_away_over_time():
    g = GRADES["survey"]
    spoof = Faults(spoof_drift_mps=2.0, spoof_bearing_deg=0.0, spoof_since_ns=EPOCH)
    later = solve("a", LAT, LON, g, spoof, 42, 0, EPOCH + 30 * NS)
    _, north, _ = pymap3d.geodetic2enu(later.latitude_deg, later.longitude_deg, 0, LAT, LON, 0)
    assert north == pytest.approx(60.0, abs=0.5)
