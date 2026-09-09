"""Measurement models: what a receiver reports, as opposed to what is true.

Pure — no I/O, no wall clock, no zenoh. This is the middle stage of the
pipeline a software-in-the-loop sensor sits in:

    truth -> measurement model -> noise/dropout -> device protocol OR payload

The stage matters because without it truth and observation are byte-identical,
which makes a two-plane split decorative: an evaluator cannot tell a navigation
failure from a display that is quietly cheating.

## The derivation rule

Every random value is a pure function of `(seed, stream, tick)`, computed from
a hash. There is no generator object and no sequence state, and three things
follow that a per-stream sequential generator does not give you:

* **Nothing to checkpoint.** A simulator that restores a run from `seed` and
  `tick` gets exactly the same noise back, without this process knowing a
  restore happened — and without anyone having remembered to serialise a
  generator.
* **Streams are independent.** Adding a second vessel, or changing the order
  things are published in, cannot perturb the first vessel's values. A
  sequential generator shared or ordered across streams gets this wrong
  quietly, and the divergence surfaces months later as "determinism broke".
* **Any tick is drawable directly**, so a restore at tick 500 needs no
  fast-forward through 499 discarded draws.

`blake2b` and Python floats only, so it is bitwise reproducible for a given
build — which is exactly the claim, and no more.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass

from keelson.geodesy import TangentPlane

# 53 bits is the mantissa of a float64, so this maps a 64-bit draw onto the
# representable reals in [0, 1) without bunching.
_MANTISSA = 1 << 53
_SCALE = 1.0 / _MANTISSA


@dataclass(frozen=True)
class GnssConfig:
    """What a vessel's GNSS receiver gets wrong.

    `sigma_m` is a standard deviation in metres, applied independently to the
    east and north components — a circular error, which is the honest first
    approximation for a receiver with no declared error ellipse.
    """

    sigma_m: float = 0.0

    @property
    def is_noisy(self) -> bool:
        return self.sigma_m > 0.0


def _uniforms(seed: int, stream: str, tick: int) -> tuple[float, float]:
    """Two independent uniforms from one hash of the coordinates.

    The stream name is separated from the numbers by a character that cannot
    occur in an entity id, so `("a|b", 1)` and `("a", "b|1")` cannot collide
    into the same draw.
    """
    digest = hashlib.blake2b(
        f"{seed}|{stream}|{tick}".encode(), digest_size=16
    ).digest()
    first = int.from_bytes(digest[:8], "big") >> 11
    second = int.from_bytes(digest[8:], "big") >> 11
    # first lands in (0, 1]: Box–Muller takes its log, and log(0) is not a
    # number this is willing to produce once in every 2^53 draws.
    return (first + 1) * _SCALE, second * _SCALE


def gaussian_pair(seed: int, stream: str, tick: int) -> tuple[float, float]:
    """Two independent standard normals for this exact coordinate.

    Box–Muller rather than a library generator because it is a closed-form
    function of the two uniforms — no state, and no dependence on how many
    values were drawn before.
    """
    u1, u2 = _uniforms(seed, stream, tick)
    radius = math.sqrt(-2.0 * math.log(u1))
    angle = 2.0 * math.pi * u2
    return radius * math.cos(angle), radius * math.sin(angle)


def observed_lat_lon(
    entity_id: str,
    latitude_deg: float,
    longitude_deg: float,
    config: GnssConfig,
    seed: int,
    tick: int,
) -> tuple[float, float]:
    """Where the receiver says the vessel is.

    Takes and returns WGS84 degrees, because that is what arrives from a
    simulator and what goes out as a `location_fix` — this service has no
    tangent plane of its own and needs none.

    The error is drawn in **metres** and converted at the vessel's actual
    latitude using `keelson.geodesy`. A standard deviation is a distance:
    applying it to degrees directly would make the east error shrink towards
    the poles and differ from the north error everywhere. Taking the ellipsoid
    from the SDK rather than hard-coding metres-per-degree keeps one definition
    of it in the system — two would agree until someone corrected one of them.
    """
    if not config.is_noisy:
        return latitude_deg, longitude_deg

    east_error, north_error = gaussian_pair(seed, f"{entity_id}/gnss", tick)
    plane = TangentPlane.at(latitude_deg, longitude_deg)
    sigma = config.sigma_m
    return (
        latitude_deg + north_error * sigma / plane.m_per_deg_lat,
        longitude_deg + east_error * sigma / plane.m_per_deg_lon,
    )
