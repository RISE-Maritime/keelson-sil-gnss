"""The receiver's live settings: which grade it is, and what is being done to it.

Served over keelson's own `configurable/v1` (`get_config` / `set_config`,
JSON), so an operator's console changes a receiver the way it would change any
other keelson component, and every applied change is republished on
`configuration_json` — which is how a recording comes to say what was done to
the receiver and when, without this component inventing an event for it.

Pure: `merge` takes the current settings and a request and returns new
settings or raises `SettingsError`, so the rules are testable without a bus.

**Real time, not replayable from the seed.** A setting takes effect on the next
sample the receiver sees. The noise it applies is still drawn from
`(seed, instant)`, but *when* an operator moved a slider is not in any seed:
what the recording holds is the `configuration_json` trail.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from .solution import GRADES, Faults

#: The keys `set_config` accepts. Anything else is refused rather than ignored:
#: a misspelt `jaming` silently doing nothing is how a fault gets "tested".
KEYS = (
    "grade",
    "receiver_on",
    "lock_duty",
    "jamming",
    "multipath",
    "sky_mask",
    "corrections",
    "spoofing",
)
SPOOF_KEYS = ("offset_m", "bearing_deg", "drift_mps")


class SettingsError(ValueError):
    """A request this receiver refuses, with the reason in words."""


@dataclass(frozen=True)
class Settings:
    grade: str = "recreational"
    #: Off is silence: no fix, no metrics — a receiver with its power cut.
    receiver_on: bool = True
    #: The fraction of fixes that get out, 0..1: a receiver losing lock.
    lock_duty: float = 1.0
    faults: Faults = field(default_factory=Faults)

    def as_config(self) -> dict:
        """What `get_config` answers and `configuration_json` carries."""
        f = self.faults
        return {
            "grade": self.grade,
            "grades": sorted(GRADES),
            "receiver_on": self.receiver_on,
            "lock_duty": self.lock_duty,
            "jamming": f.jamming,
            "multipath": f.multipath,
            "sky_mask": f.sky_mask,
            "corrections": f.corrections,
            "spoofing": (
                {
                    "offset_m": f.spoof_offset_m,
                    "bearing_deg": f.spoof_bearing_deg,
                    "drift_mps": f.spoof_drift_mps,
                    "since_ns": f.spoof_since_ns,
                }
                if f.spoofing
                else None
            ),
        }

    def cleared(self) -> Settings:
        """Every fault off, the grade kept."""
        return Settings(grade=self.grade)


def _fraction(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SettingsError(f"`{name}` must be a number from 0 to 1")
    value = float(value)
    if not 0.0 <= value <= 1.0:
        raise SettingsError(f"`{name}` must be from 0 to 1, not {value}")
    return value


def _flag(name: str, value) -> bool:
    if not isinstance(value, bool):
        raise SettingsError(f"`{name}` must be true or false")
    return value


def merge(current: Settings, request: dict, now_ns: int | None = None) -> Settings:
    """`current` with the keys in `request` changed; the rest kept.

    `{"clear": true}` first clears every fault, so a console's "clear all" is
    one call. `spoofing: null` stops spoofing; an object starts or changes it,
    and a change restarts the drift from `now_ns` — the instant the receiver
    last saw, which the caller supplies.
    """
    if not isinstance(request, dict):
        raise SettingsError("a configuration is a JSON object")
    request = dict(request)
    settings = current.cleared() if request.pop("clear", False) is True else current
    unknown = sorted(set(request) - set(KEYS))
    if unknown:
        raise SettingsError(f"unknown setting(s): {', '.join(unknown)}; known: {', '.join(KEYS)}")

    faults = settings.faults
    changes: dict = {}
    if "grade" in request:
        if request["grade"] not in GRADES:
            raise SettingsError(f"`grade` must be one of {', '.join(sorted(GRADES))}")
        changes["grade"] = request["grade"]
    if "receiver_on" in request:
        changes["receiver_on"] = _flag("receiver_on", request["receiver_on"])
    if "lock_duty" in request:
        changes["lock_duty"] = _fraction("lock_duty", request["lock_duty"])
    for name in ("jamming", "multipath", "sky_mask"):
        if name in request:
            faults = replace(faults, **{name: _fraction(name, request[name])})
    if "corrections" in request:
        faults = replace(faults, corrections=_flag("corrections", request["corrections"]))
    if "spoofing" in request:
        spoof = request["spoofing"]
        if spoof is None:
            faults = replace(
                faults,
                spoof_offset_m=0.0,
                spoof_bearing_deg=0.0,
                spoof_drift_mps=0.0,
                spoof_since_ns=None,
            )
        elif isinstance(spoof, dict):
            extra = sorted(set(spoof) - set(SPOOF_KEYS) - {"since_ns"})
            if extra:
                raise SettingsError(f"unknown spoofing setting(s): {', '.join(extra)}")
            values = {}
            for key in SPOOF_KEYS:
                value = spoof.get(key, 0.0)
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise SettingsError(f"`spoofing.{key}` must be a number")
                values[key] = float(value)
            if values["offset_m"] < 0 or values["drift_mps"] < 0:
                raise SettingsError("a spoofing offset and drift are distances and speeds, not negative")
            faults = replace(
                faults,
                spoof_offset_m=values["offset_m"],
                spoof_bearing_deg=values["bearing_deg"] % 360.0,
                spoof_drift_mps=values["drift_mps"],
                spoof_since_ns=now_ns,
            )
        else:
            raise SettingsError("`spoofing` is an object or null")
    return replace(settings, faults=faults, **changes)
