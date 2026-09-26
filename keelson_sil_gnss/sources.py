"""Where the truth comes from, kept separate from what is done to it.

The measurement model in `model.py` is generic: it needs a position, a run
seed, a tick and an entity, and it does not care who published them. What is
*not* generic is how a given simulator says those things on the bus. That is
this module, and the split is deliberate — it is what lets one measurement
model serve more than one simulator without either of them leaking into it.

A source answers two questions:

* which key to subscribe to, and
* how to turn a sample on that key into a `Fix`, or into nothing.

## There is one source today, and it is transitional

`VesselTruthRecord` reads Slipway's `vessel_truth` record: a JSON blob riding
on the `raw_json` subject under the `truth` source. That is a private contract
of one simulator, declared in that project's own rider register, and a
component named for the ecosystem rather than for Slipway should not need to
know about it.

It is here because the alternative is worse. Slipway is retiring `vessel_truth`
onto typed keelson subjects — `location_fix`, `heading_true_north_deg`,
`ScenarioStatus` and friends, all published under `/truth` — and when that
lands a `TypedTruth` source joins this module, reading the same subjects any
observation uses. Waiting for it would have meant either no repository or two
copies of the measurement model, and two copies of a noise model is strictly
worse than not having split at all.

So: this file is expected to grow a second source and then to lose its first.
"""

from __future__ import annotations

import json
from dataclasses import dataclass

import keelson
from google.protobuf.message import DecodeError
from keelson.payloads.Primitives_pb2 import TimestampedString


@dataclass(frozen=True)
class Fix:
    """One position a simulator asserts is true, and what it takes to model it.

    `timestamp_ns` is the **simulated** source time the simulator stamped, not
    transport time. In a stepped or fast-time run those diverge by design, and
    a modelled fix has to line up with the tick that produced it.

    `enabled` is false when the receiver has been failed. A real one keeps its
    power on and reports nothing usable, so the service publishes nothing at
    all — a stale fix would be the frozen-value lie that makes a recording
    unreadable after the fact.
    """

    entity_id: str
    latitude_deg: float
    longitude_deg: float
    seed: int
    tick: int
    timestamp_ns: int
    enabled: bool
    #: The fraction of ticks the simulator says the receiver gets a fix out on
    #: (Slipway's `set_gnss_duty`). 1.0 is healthy.
    duty: float = 1.0


class VesselTruthRecord:
    """Slipway's `vessel_truth` JSON record on `raw_json/truth`.

    Transitional. See this module's docstring for what replaces it and why it
    is here in the meantime.
    """

    #: The source tag truth is published under. Truth is a source, not a realm
    #: or a separate key space: the receiver reads the vessel's own entity on
    #: the one realm and picks truth out by its source.
    SOURCE = "truth"

    SUBJECT = "raw_json"

    def key(self, realm: str, entity_id: str) -> str:
        return keelson.construct_pubsub_key(realm, entity_id, self.SUBJECT, self.SOURCE)

    def fix(self, envelope: bytes, entity_id: str) -> Fix | None:
        """A `Fix`, or `None` for anything this source cannot read.

        Silent on a malformed sample rather than raising. This is a subscriber
        callback on a shared subject: a record it does not understand is not
        its business, and a receiver that died on one would take a whole run
        down over a message addressed to somebody else.
        """
        try:
            _, timestamp_ns, payload = keelson.uncover(envelope)
            message = TimestampedString()
            message.ParseFromString(payload)
            record = json.loads(message.value)
        except (DecodeError, UnicodeDecodeError, ValueError, TypeError):
            return None

        if record.get("entity_id") != entity_id:
            return None

        try:
            return Fix(
                entity_id=entity_id,
                latitude_deg=float(record["latitude_deg"]),
                longitude_deg=float(record["longitude_deg"]),
                seed=int(record["seed"]),
                tick=int(record["tick"]),
                timestamp_ns=timestamp_ns,
                enabled=bool(record.get("gnss_enabled", True)),
                duty=float(record.get("gnss_duty", 1.0)),
            )
        except (KeyError, TypeError, ValueError):
            return None
