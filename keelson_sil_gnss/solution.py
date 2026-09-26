"""What a receiver of a given grade reports, faults and all.

Pure — no I/O, no wall clock, no zenoh — and stateless in the sense of the
derivation rule in `model.py`: every random value is a function of
`(seed, stream, instant)`. So a restored run recomputes the same solution, and
any instant can be drawn without drawing the ones before it.

## Why the error wanders instead of jumping

The first model drew a fresh, independent error on every fix. A real receiver
does not do that: most of its error comes from the atmosphere, the orbits and
the clocks, which change over minutes, so consecutive fixes sit close together
and the whole cloud drifts. Only a small part is per-fix jitter. A chart fed
independent noise at 10 Hz shows a vessel teleporting a few metres every tenth
of a second — which is what was on the station.

So the error is two parts per axis:

* **correlated** — Gaussian values hashed at knots `tau` seconds apart and
  smoothly interpolated between them (`wander`). A stateless stand-in for a
  first-order Gauss–Markov process: same scale, same timescale, nothing to
  checkpoint.
* **white** — a small independent term per fix, drawn exactly as before.

## What drives what

Satellites decide geometry, geometry decides accuracy: `hdop` grows as fewer
satellites are used, and the error scales with `hdop / hdop0`. Jamming and a
masked sky therefore degrade the fix *through* the satellite count, the way a
real receiver degrades, rather than through a separate knob that could
disagree with the numbers it reports.

## What the receiver knows, and what it does not

The receiver's own error estimate (`sigma_*_m`, which becomes GST and the fix's
covariance) is the *modelled* spread. It is not the actual error of this fix —
only truth knows that — and spoofing does not appear in it at all, because a
spoofed receiver believes its solution. That gap is the thing an evaluator
needs to be able to see.

The grade figures are nominal, spec-sheet-class values for each kind of
receiver, not measurements of a particular unit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .model import gaussian_pair, offset_lat_lon

NS = 1_000_000_000

# keelson.LocationFixQuality enums, by value, so this module needs no protobuf.
FIX_NO, FIX_2D, FIX_3D = 2, 3, 4
POS_SINGLE, POS_PSRDIFF, POS_SBAS, POS_RTK_FLOAT, POS_RTK_INT = 2, 3, 4, 6, 7
RTK_NONE, RTK_DIFFERENTIAL, RTK_FLOAT, RTK_FIXED = 1, 2, 3, 4
INTEGRITY_SAFE, INTEGRITY_CAUTION, INTEGRITY_UNSAFE = 2, 3, 4


@dataclass(frozen=True)
class Grade:
    """A class of receiver: what it reports when nothing is wrong, and what it
    falls back to when its corrections are lost."""

    name: str
    rate_hz: float
    #: Per-axis 1-sigma of the slowly wandering part, and its timescale.
    sigma_corr_m: float
    tau_s: float
    #: Per-axis 1-sigma of the per-fix jitter.
    sigma_white_m: float
    pos_type: int
    rtk_status: int
    #: Satellites in view, as a range the count wanders through.
    visible: tuple[int, int]
    #: HDOP with the nominal number of satellites in use.
    hdop0: float
    #: The same receiver without its corrections (SBAS, DGNSS or RTK).
    lost_sigma_corr_m: float
    lost_sigma_white_m: float
    lost_pos_type: int
    lost_rtk_status: int
    #: How far multipath can pull this class of receiver, per axis, 1-sigma.
    multipath_m: float


GRADES: dict[str, Grade] = {
    # A chartplotter or handheld: single-frequency GPS+GLONASS with SBAS.
    "recreational": Grade(
        name="recreational",
        rate_hz=1.0,
        sigma_corr_m=1.5,
        tau_s=180.0,
        sigma_white_m=0.3,
        pos_type=POS_SBAS,
        rtk_status=RTK_DIFFERENTIAL,
        visible=(14, 20),
        hdop0=0.9,
        lost_sigma_corr_m=2.5,
        lost_sigma_white_m=0.5,
        lost_pos_type=POS_SINGLE,
        lost_rtk_status=RTK_NONE,
        multipath_m=3.0,
    ),
    # A commercial marine receiver: dual-frequency, four constellations, DGNSS.
    "professional": Grade(
        name="professional",
        rate_hz=10.0,
        sigma_corr_m=0.4,
        tau_s=120.0,
        sigma_white_m=0.08,
        pos_type=POS_PSRDIFF,
        rtk_status=RTK_DIFFERENTIAL,
        visible=(25, 35),
        hdop0=0.6,
        lost_sigma_corr_m=1.2,
        lost_sigma_white_m=0.2,
        lost_pos_type=POS_SINGLE,
        lost_rtk_status=RTK_NONE,
        multipath_m=1.5,
    ),
    # A survey receiver on RTK: fixed-integer, centimetre level; RTK float
    # when the corrections stop arriving.
    "survey": Grade(
        name="survey",
        rate_hz=10.0,
        sigma_corr_m=0.008,
        tau_s=60.0,
        sigma_white_m=0.004,
        pos_type=POS_RTK_INT,
        rtk_status=RTK_FIXED,
        visible=(25, 35),
        hdop0=0.6,
        lost_sigma_corr_m=0.15,
        lost_sigma_white_m=0.03,
        lost_pos_type=POS_RTK_FLOAT,
        lost_rtk_status=RTK_FLOAT,
        multipath_m=0.3,
    ),
}


def custom_grade(sigma_m: float) -> Grade:
    """A receiver described only by a total per-axis sigma — the old
    `--sigma-m`. Recreational in every other respect, with the sigma split
    between wander and jitter the way that grade splits it."""
    base = GRADES["recreational"]
    total = math.hypot(base.sigma_corr_m, base.sigma_white_m)
    scale = sigma_m / total if total else 0.0
    return replace(
        base,
        name="custom",
        sigma_corr_m=base.sigma_corr_m * scale,
        sigma_white_m=base.sigma_white_m * scale,
        lost_sigma_corr_m=base.lost_sigma_corr_m * scale,
        lost_sigma_white_m=base.lost_sigma_white_m * scale,
    )


@dataclass(frozen=True)
class Faults:
    """What is being done to the receiver. All zero is a healthy one."""

    #: 0..1: interference. Satellites drop out, the error grows, and near 1
    #: there is no fix at all. Above 0.5 it also takes out RTK/DGNSS links.
    jamming: float = 0.0
    #: 0..1: reflections off a hull, a quay or a cliff — a short-timescale
    #: extra error on top of the normal wander.
    multipath: float = 0.0
    #: 0..1: the fraction of the sky blocked — a bridge, a lock wall, a fjord.
    sky_mask: float = 0.0
    #: Whether SBAS / DGNSS / RTK corrections are arriving.
    corrections: bool = True
    #: Spoofing: the reported position is pulled `offset_m` towards
    #: `bearing_deg` and then keeps walking at `drift_mps`, from `since_ns`.
    spoof_offset_m: float = 0.0
    spoof_bearing_deg: float = 0.0
    spoof_drift_mps: float = 0.0
    spoof_since_ns: int | None = None

    @property
    def spoofing(self) -> bool:
        return self.spoof_offset_m > 0.0 or self.spoof_drift_mps > 0.0


@dataclass(frozen=True)
class Solution:
    """One instant of the receiver's output."""

    has_fix: bool
    latitude_deg: float
    longitude_deg: float
    satellites_visible: int
    satellites_used: int
    hdop: float
    vdop: float
    pdop: float
    fix_type: int
    pos_type: int
    rtk_status: int
    integrity: int
    #: What the receiver believes its 1-sigma error is, per axis.
    sigma_east_m: float
    sigma_north_m: float
    sigma_up_m: float

    @property
    def error_rms_m(self) -> float:
        """Horizontal RMS, the way GST's first field is read."""
        return math.hypot(self.sigma_east_m, self.sigma_north_m)


def wander(seed: int, stream: str, t_ns: int, tau_s: float) -> float:
    """A standard normal that drifts smoothly with time, timescale `tau_s`.

    Hashed values at knots `tau` apart, blended with a smoothstep and divided by
    the blend's own spread, so the result has unit variance at every instant —
    between two independent knots a plain blend would have less. Integer
    arithmetic on nanoseconds, so the same instant always picks the same knots.
    """
    tau_ns = max(1, int(tau_s * NS))
    knot, into = divmod(t_ns, tau_ns)
    s = into / tau_ns
    w = s * s * (3.0 - 2.0 * s)
    a = gaussian_pair(seed, stream, knot)[0]
    b = gaussian_pair(seed, stream, knot + 1)[0]
    return ((1.0 - w) * a + w * b) / math.sqrt((1.0 - w) ** 2 + w**2)


def satellites(
    entity_id: str, grade: Grade, faults: Faults, seed: int, t_ns: int
) -> tuple[int, int]:
    """(visible, used). Visible wanders over minutes as satellites rise and
    set; the sky mask hides some, jamming drowns the weak ones, and a receiver
    uses most but not all of what it can track."""
    low, high = grade.visible
    mid, half = (low + high) / 2.0, (high - low) / 2.0
    drift = max(-1.0, min(1.0, wander(seed, f"{entity_id}/gnss-sv", t_ns, 600.0) / 2.0))
    visible = round(mid + half * drift)
    trackable = visible * (1.0 - faults.sky_mask)
    used = round(trackable * 0.8 * (1.0 - faults.jamming**1.5))
    return visible, max(0, min(visible, used))


def nominal_used(grade: Grade) -> float:
    return (grade.visible[0] + grade.visible[1]) / 2.0 * 0.8


def solve(
    entity_id: str,
    latitude_deg: float,
    longitude_deg: float,
    grade: Grade,
    faults: Faults,
    seed: int,
    tick: int,
    t_ns: int,
) -> Solution:
    """The receiver's answer for this instant: where, how well, and from what.

    `tick` feeds the per-fix jitter (as in `model.py`); `t_ns` — the simulated
    time of the sample — feeds everything that wanders, so the wander has a
    real timescale whatever the simulator's step.
    """
    visible, used = satellites(entity_id, grade, faults, seed, t_ns)

    if used >= 4:
        fix_type = FIX_3D
    elif used == 3:
        fix_type = FIX_2D
    else:
        fix_type = FIX_NO
    has_fix = fix_type != FIX_NO

    # Geometry: fewer satellites, worse DOP; a masked sky also leaves them
    # bunched in what remains of it.
    geometry = math.sqrt(nominal_used(grade) / max(used, 3)) * (1.0 + 0.8 * faults.sky_mask)
    hdop = grade.hdop0 * geometry
    vdop = 1.6 * hdop
    pdop = math.hypot(hdop, vdop)

    # Corrections: the link is lost if nobody sends them, or if the band is
    # jammed hard enough that the receiver cannot hear them.
    corrected = faults.corrections and faults.jamming < 0.5
    if corrected:
        sigma_corr, sigma_white = grade.sigma_corr_m, grade.sigma_white_m
        pos_type, rtk_status = grade.pos_type, grade.rtk_status
    else:
        sigma_corr, sigma_white = grade.lost_sigma_corr_m, grade.lost_sigma_white_m
        pos_type, rtk_status = grade.lost_pos_type, grade.lost_rtk_status

    scale = (hdop / grade.hdop0) * (1.0 + 3.0 * faults.jamming**2)
    sigma_corr *= scale
    sigma_white *= scale
    sigma_mp = grade.multipath_m * faults.multipath

    east = (
        sigma_corr * wander(seed, f"{entity_id}/gnss-e", t_ns, grade.tau_s)
        + sigma_mp * wander(seed, f"{entity_id}/gnss-mp-e", t_ns, 15.0)
    )
    north = (
        sigma_corr * wander(seed, f"{entity_id}/gnss-n", t_ns, grade.tau_s)
        + sigma_mp * wander(seed, f"{entity_id}/gnss-mp-n", t_ns, 15.0)
    )
    white_e, white_n = gaussian_pair(seed, f"{entity_id}/gnss", tick)
    east += sigma_white * white_e
    north += sigma_white * white_n

    if faults.spoofing:
        walked = 0.0
        if faults.spoof_since_ns is not None and faults.spoof_drift_mps > 0.0:
            walked = faults.spoof_drift_mps * max(0, t_ns - faults.spoof_since_ns) / NS
        pull = faults.spoof_offset_m + walked
        bearing = math.radians(faults.spoof_bearing_deg)
        east += pull * math.sin(bearing)
        north += pull * math.cos(bearing)

    lat, lon = offset_lat_lon(latitude_deg, longitude_deg, east, north)
    # Per axis: the three parts are independent, so their variances add.
    sigma_axis = math.hypot(sigma_corr, sigma_white, sigma_mp)

    if not has_fix:
        integrity = INTEGRITY_UNSAFE
    elif faults.jamming >= 0.3 or hdop > 2.5 * grade.hdop0:
        integrity = INTEGRITY_CAUTION
    else:
        # Spoofing included: the receiver's own check passes, which is the
        # attack.
        integrity = INTEGRITY_SAFE

    return Solution(
        has_fix=has_fix,
        latitude_deg=lat,
        longitude_deg=lon,
        satellites_visible=visible,
        satellites_used=used,
        hdop=hdop,
        vdop=vdop,
        pdop=pdop,
        fix_type=fix_type,
        pos_type=pos_type if has_fix else 0,
        rtk_status=rtk_status if has_fix else RTK_NONE,
        integrity=integrity,
        sigma_east_m=sigma_axis,
        sigma_north_m=sigma_axis,
        sigma_up_m=sigma_axis * vdop / hdop,
    )
