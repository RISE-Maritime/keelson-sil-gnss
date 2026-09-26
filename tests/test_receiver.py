"""The receiver process, against a stand-in session: what goes out, how often,
and that a setting takes effect on the next sample it sees."""

from __future__ import annotations

import json
from collections import Counter

import keelson
import pytest
from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix
from keelson.payloads.Primitives_pb2 import TimestampedInt, TimestampedString

from keelson_sil_gnss.main import PUBSUB_SUBJECTS, Receiver, parse_args
from keelson_sil_gnss.settings import SettingsError

ENTITY = "slipway-alpha"
EPOCH = 1_767_772_800_000_000_000
DT = 100_000_000


class _Publisher:
    def __init__(self, key, sink):
        self.key_expr, self._sink = key, sink

    def put(self, payload):
        self._sink.append((self.key_expr, payload))


class _Session:
    def __init__(self):
        self.puts: list[tuple[str, bytes]] = []

    def declare_publisher(self, key, **_qos):
        return _Publisher(key, self.puts)


class _Sample:
    def __init__(self, payload: bytes):
        self.payload = self

        self._bytes = payload

    def to_bytes(self):
        return self._bytes


def truth(tick: int, **extra) -> _Sample:
    record = {"entity_id": ENTITY, "latitude_deg": 57.68, "longitude_deg": 11.83,
              "seed": 42, "tick": tick, **extra}
    message = TimestampedString(value=json.dumps(record))
    return _Sample(keelson.enclose(message.SerializeToString(), enclosed_at=EPOCH + tick * DT))


def receiver(*argv):
    session = _Session()
    return Receiver(session, parse_args(["--entity-id", ENTITY, *argv])), session


def subjects(session):
    return Counter(key.split("/")[4] for key, _ in session.puts)


def feed(rx, ticks, **extra):
    for tick in ticks:
        rx.on_truth(truth(tick, **extra))


@pytest.mark.parametrize(("grade", "fixes"), [("recreational", 3), ("professional", 30)])
def test_fixes_go_out_at_the_grades_rate_and_metrics_once_a_second(grade, fixes):
    rx, session = receiver("--grade", grade)
    feed(rx, range(30))  # three seconds of a 10 Hz simulator
    seen = subjects(session)
    assert seen["location_fix"] == fixes
    assert seen["location_fix_satellites_used"] == 3
    assert seen["location_fix_quality"] == 3
    assert seen["gnss_error_rms_m"] == 3
    assert set(seen) <= set(PUBSUB_SUBJECTS)


def test_a_fix_carries_what_the_receiver_believes_its_error_is():
    rx, session = receiver("--grade", "professional")
    feed(rx, range(1))
    _, payload = next(p for p in session.puts if "/location_fix/" in p[0])
    _, stamp, inner = keelson.uncover(payload)
    fix = LocationFix()
    fix.ParseFromString(inner)
    assert stamp == EPOCH, "sim time on the envelope, not this process's clock"
    assert fix.position_covariance_type == LocationFix.PositionCovarianceType.DIAGONAL_KNOWN
    assert 0.01 < fix.position_covariance[0] < 1.0


def test_a_setting_takes_effect_on_the_next_sample():
    rx, session = receiver("--grade", "professional")
    feed(rx, range(10))
    rx.set_config({"jamming": 1.0})
    session.puts.clear()
    feed(rx, range(10, 30))
    seen = subjects(session)
    assert seen["location_fix"] == 0, "no satellites, no fix: a gap"
    used = [p for k, p in session.puts if k.endswith("/location_fix_satellites_used/gnss")]
    value = TimestampedInt()
    value.ParseFromString(keelson.uncover(used[-1])[2])
    assert value.value == 0


def test_switched_off_is_silence():
    rx, session = receiver()
    rx.set_config({"receiver_on": False})
    feed(rx, range(20))
    assert session.puts == []


def test_the_simulators_failure_is_honoured_too():
    rx, session = receiver()
    feed(rx, range(20), gnss_enabled=False)
    assert session.puts == []


def test_spoofing_starts_from_the_last_instant_the_receiver_saw():
    rx, _ = receiver()
    feed(rx, range(5))
    rx.set_config({"spoofing": {"drift_mps": 1.0}})
    assert rx.settings.faults.spoof_since_ns == EPOCH + 4 * DT


def test_a_refused_setting_changes_nothing():
    rx, _ = receiver("--grade", "survey")
    with pytest.raises(SettingsError):
        rx.set_config({"jamming": 3})
    assert rx.settings.faults.jamming == 0.0


def test_a_sigma_receiver_is_the_custom_grade():
    rx, _ = receiver("--sigma-m", "2.5")
    assert rx.settings.grade == "custom"
    assert "custom" in rx.get_config()["grades"]
    rx.set_config({"grade": "survey"})
    rx.set_config({"grade": "custom"})
    assert rx.settings.grade == "custom"
