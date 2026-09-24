"""keelson-sil-gnss: a simulated GNSS receiver.

Reads where a vessel actually is from a simulator's truth output, decides what
a receiver would have reported, and publishes that as `location_fix` under its
own source id.

    keelson-sil-gnss --realm rise --entity-id slipway-alpha \\
        --source-id gnss --sigma-m 2.5 --connect tcp/127.0.0.1:7447

**Why this is its own process, and its own repository.** A simulator's job is
to produce keelson messages describing the world. Every imperfection downstream
of that is somebody else's: sensor error here, protocol conversion in
`keelson2nmea0183`. Keeping them apart is what lets each be tested on its own
and lets a real platform run its production connectors unchanged against
simulated data. Deciding what a receiver would have reported has a different
rate of change, different validation evidence and a different owner from
deciding where a hull is — and it is the first of a family, since a compass, a
rate gyro, a radar and a camera each want the same treatment.

It reads the **truth plane**, so the operational plane never carries an exact
position at all. A consumer reading `location_fix` is therefore reading a
modelled receiver, by construction rather than by convention.

The service owns no clock and no state. `seed` and `tick` arrive with every
truth sample, and the error at a tick is a pure function of them, so two runs
of the same scenario produce identical fixes and a restored run picks up
exactly where it left off — without this process knowing a restore happened.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys

import keelson
import zenoh
from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix
from keelson.scaffolding import declare_liveliness, declare_publisher
from keelson.scaffolding.cli import add_common_arguments, create_zenoh_config
from keelson.scaffolding.signals import GracefulShutdown

from .model import GnssConfig, observed_lat_lon
from .sources import VesselTruthRecord

logger = logging.getLogger("keelson.sil.gnss")

# Windows delivers neither SIGINT nor SIGTERM to a service started as a child
# process: its SIGTERM is TerminateProcess, which runs no handler at all. A
# console CTRL_BREAK does arrive, as SIGBREAK, so it is listed here — without
# it a service run natively on Windows is killed rather than shut down, and a
# simulated pause would end as a truncated recording instead of the clean data
# gap the bus contract requires. keelson's default is SIGINT and SIGTERM only.
SHUTDOWN_SIGNALS = [signal.SIGINT, signal.SIGTERM] + (
    [signal.SIGBREAK] if hasattr(signal, "SIGBREAK") else []
)

# What this service puts on the operational plane, named here so a platform
# health config can be checked against real constants rather than against
# strings it repeats.
SOURCE_ID = "gnss"
PUBSUB_SUBJECTS = ("location_fix",)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        "keelson-sil-gnss",
        description="Simulated GNSS receiver: truth in, location_fix out.",
    )
    add_common_arguments(parser)
    parser.add_argument("-r", "--realm", default="rise")
    parser.add_argument(
        "-e", "--entity-id", required=True, help="The vessel this receiver is fitted to"
    )
    parser.add_argument(
        "-s",
        "--source-id",
        default=SOURCE_ID,
        help="Names the device, as on a real platform",
    )
    parser.add_argument(
        "--sigma-m",
        type=float,
        default=0.0,
        help="Standard deviation of the position error in metres, applied "
        "independently to east and north. 0 reports the true position, which is "
        "useful for isolating a fault but is not a receiver anyone owns.",
    )
    return parser.parse_args(argv)


class Receiver:
    """One vessel's receiver. Stateless between fixes, by design."""

    def __init__(self, session: zenoh.Session, args: argparse.Namespace):
        self._args = args
        self._config = GnssConfig(sigma_m=args.sigma_m)
        self._source = VesselTruthRecord()
        self._publisher = declare_publisher(
            session,
            keelson.construct_pubsub_key(
                args.realm, args.entity_id, "location_fix", args.source_id
            ),
        )
        self.fixes = 0
        self.blind = 0

    @property
    def truth_key(self) -> str:
        return self._source.key(self._args.realm, self._args.entity_id)

    def on_truth(self, sample) -> None:
        fix = self._source.fix(sample.payload.to_bytes(), self._args.entity_id)
        if fix is None:
            return

        # The receiver has failed. A real one keeps its power on and reports
        # nothing usable; publishing a stale fix would be the frozen-value lie
        # this project refuses everywhere else.
        if not fix.enabled:
            self.blind += 1
            return

        latitude, longitude = observed_lat_lon(
            entity_id=fix.entity_id,
            latitude_deg=fix.latitude_deg,
            longitude_deg=fix.longitude_deg,
            config=self._config,
            seed=fix.seed,
            tick=fix.tick,
        )

        message = LocationFix()
        message.timestamp.FromNanoseconds(fix.timestamp_ns)
        message.frame_id = f"{self._args.entity_id}/{self._args.source_id}"
        message.latitude = latitude
        message.longitude = longitude
        # Sim time on the envelope too: `enclose` defaults to the wall clock,
        # which a run on an imported recording's epoch is months away from.
        self._publisher.put(
            keelson.enclose(message.SerializeToString(), enclosed_at=fix.timestamp_ns)
        )
        self.fixes += 1


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=args.log_level, format="%(asctime)s %(levelname)s %(name)s %(message)s"
    )
    config = create_zenoh_config(args.mode, args.connect, args.listen)
    # Never discover a bus by accident.
    config.insert_json5("scouting/multicast/enabled", "false")

    with zenoh.open(config) as session:
        receiver = Receiver(session, args)
        session.declare_subscriber(receiver.truth_key, receiver.on_truth)
        with declare_liveliness(
            session,
            args.realm,
            args.entity_id,
            args.source_id,
            pubsub_subjects=PUBSUB_SUBJECTS,
        ):
            logger.info(
                "[GNSS] %s/%s fitted to %s, sigma %.2f m",
                args.entity_id,
                args.source_id,
                args.realm,
                args.sigma_m,
            )
            with GracefulShutdown(SHUTDOWN_SIGNALS) as shutdown:
                while not shutdown.is_requested():
                    shutdown.wait(timeout=1.0)
    logger.info("[GNSS] %d fixes, %d ticks blind", receiver.fixes, receiver.blind)


if __name__ == "__main__":
    sys.exit(main())
