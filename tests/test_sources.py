"""Reading a simulator's truth, which used to be four lines inside a callback.

Pulling the parsing out into a source made it testable for the first time, and
what it is worth testing is mostly the refusals: a subscriber callback that
raised on a message addressed to somebody else would take down a whole run.
"""

from __future__ import annotations

import json

import keelson
import pytest
from keelson.payloads.Primitives_pb2 import TimestampedString

from keelson_sil_gnss.sources import VesselTruthRecord

ENTITY = "slipway-alpha"
STAMP_NS = 1_767_772_800_000_000_000

RECORD = {
    "entity_id": ENTITY,
    "latitude_deg": 57.68,
    "longitude_deg": 11.83,
    "seed": 42,
    "tick": 7,
    "gnss_enabled": True,
}


def envelope(record: dict | str, timestamp_ns: int = STAMP_NS) -> bytes:
    message = TimestampedString()
    message.value = record if isinstance(record, str) else json.dumps(record)
    return keelson.enclose(message.SerializeToString(), enclosed_at=timestamp_ns)


@pytest.fixture
def source() -> VesselTruthRecord:
    return VesselTruthRecord()


def test_the_key_names_the_vessels_own_entity_and_the_truth_source(source):
    """Truth is a source tag, not a realm and not a separate key space. The
    receiver reads its own hull on the one realm and picks truth out by
    source."""
    key = source.key("rise", ENTITY)
    assert ENTITY in key
    assert key.endswith("/raw_json/truth")


def test_a_well_formed_record_becomes_a_fix(source):
    fix = source.fix(envelope(RECORD), ENTITY)
    assert fix is not None
    assert (fix.latitude_deg, fix.longitude_deg) == (57.68, 11.83)
    assert (fix.seed, fix.tick) == (42, 7)
    assert fix.enabled


def test_the_timestamp_is_the_simulators_and_not_this_process_clock(source):
    """In a stepped or fast-time run, simulated time and transport time
    diverge by design. A fix stamped with the wall clock would not line up
    with the tick that produced it, which is the whole basis of comparing a
    reported position against a true one after the fact."""
    fix = source.fix(envelope(RECORD), ENTITY)
    assert fix.timestamp_ns == STAMP_NS


def test_another_vessels_record_is_not_this_receivers(source):
    """One container per fitted device, and they share a subject."""
    assert source.fix(envelope(RECORD), "slipway-bravo") is None


def test_a_failed_receiver_is_reported_rather_than_dropped(source):
    """The service needs to tell "the receiver is blind" from "nothing
    arrived" — one is a modelled fault and the other is a broken stack."""
    fix = source.fix(envelope(RECORD | {"gnss_enabled": False}), ENTITY)
    assert fix is not None
    assert not fix.enabled


def test_a_record_that_does_not_say_is_taken_as_working(source):
    """A simulator with no fault model at all publishes no such field, and
    should get a working receiver rather than a silent one."""
    without = {k: v for k, v in RECORD.items() if k != "gnss_enabled"}
    assert source.fix(envelope(without), ENTITY).enabled


@pytest.mark.parametrize(
    "broken",
    [
        b"",
        b"not an envelope at all",
        envelope("{ this is not json"),
        envelope({"entity_id": ENTITY}),  # right vessel, no position
        envelope(RECORD | {"latitude_deg": "north a bit"}),
        envelope(RECORD | {"tick": None}),
    ],
    ids=["empty", "garbage", "bad-json", "no-position", "bad-latitude", "null-tick"],
)
def test_nothing_it_cannot_read_raises(source, broken):
    """Silence, not an exception. This runs on a zenoh subscriber callback: a
    record it does not understand is not its business, and dying on one would
    end a run over a message meant for somebody else."""
    assert source.fix(broken, ENTITY) is None
