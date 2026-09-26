"""keelson-sil-gnss: a simulated GNSS receiver.

Reads where a vessel actually is from a simulator's truth output, decides what
a receiver would have reported, and publishes that as `location_fix` under its
own source id.

    keelson-sil-gnss --realm rise --entity-id slipway-alpha \\
        --source-id gnss --grade professional --connect tcp/127.0.0.1:7447

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

**What it publishes.** `location_fix` at the grade's rate, with the receiver's
own covariance, and once a second the metrics a real receiver reports:
satellites visible and used, `location_fix_quality`, HDOP/VDOP/PDOP and the GST
error statistics. **What it can be told**, over keelson's `configurable/v1`: its
grade, and jamming, multipath, sky blockage, lost corrections, spoofing, lock
loss and power — see `settings.py`.

The measurement owns no clock and no state. `seed` and `tick` arrive with every
truth sample, and the error at a tick is a pure function of them, so two runs
of the same scenario produce identical fixes and a restored run picks up
exactly where it left off — without this process knowing a restore happened.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
from dataclasses import replace

import keelson
import zenoh
from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix
from keelson.payloads.LocationFixQuality_pb2 import LocationFixQuality
from keelson.payloads.Primitives_pb2 import TimestampedFloat, TimestampedInt
from keelson.scaffolding import declare_liveliness, declare_publisher, make_configurable
from keelson.scaffolding.cli import add_common_arguments, create_zenoh_config
from keelson.scaffolding.signals import GracefulShutdown

from .model import reports_fix
from .settings import Settings, merge
from .solution import GRADES, NS, Solution, custom_grade, solve
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
INT_METRICS = ("location_fix_satellites_visible", "location_fix_satellites_used")
FLOAT_METRICS = (
    "location_fix_hdop",
    "location_fix_vdop",
    "location_fix_pdop",
    "gnss_error_rms_m",
    "gnss_error_semi_major_axis_m",
    "gnss_error_semi_minor_axis_m",
    "gnss_error_orientation_deg",
    "gnss_error_latitude_m",
    "gnss_error_longitude_m",
    "gnss_error_altitude_m",
)
PUBSUB_SUBJECTS = ("location_fix", "location_fix_quality", *INT_METRICS, *FLOAT_METRICS)

#: The metrics go out once a second whatever the fix rate, as a receiver's
#: GSA/GST sentences do.
METRICS_PERIOD_NS = NS


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        "keelson-sil-gnss",
        description="Simulated GNSS receiver: truth in, location_fix and its metrics out.",
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
        "--grade",
        choices=sorted(GRADES),
        default=None,
        help="What class of receiver this is. Changeable at run time over configurable/v1.",
    )
    parser.add_argument(
        "--sigma-m",
        type=float,
        default=0.0,
        help="A receiver described only by its per-axis error in metres, split between "
        "wander and jitter as the recreational grade splits it. Ignored with --grade.",
    )
    return parser.parse_args(argv)


def _timestamped_float(value: float, t_ns: int) -> bytes:
    message = TimestampedFloat()
    message.timestamp.FromNanoseconds(t_ns)
    message.value = value
    return keelson.enclose(message.SerializeToString(), enclosed_at=t_ns)


def _timestamped_int(value: int, t_ns: int) -> bytes:
    message = TimestampedInt()
    message.timestamp.FromNanoseconds(t_ns)
    message.value = value
    return keelson.enclose(message.SerializeToString(), enclosed_at=t_ns)


class Receiver:
    """One vessel's receiver.

    The measurement is stateless; what is kept here is only what a receiver
    keeps — its settings, and which output slots it has already filled — and
    none of it moves a single metre of error.
    """

    def __init__(self, session: zenoh.Session, args: argparse.Namespace):
        self._args = args
        self._source = VesselTruthRecord()
        #: A `--sigma-m` receiver is its own grade, reachable under "custom".
        self._custom = custom_grade(args.sigma_m) if args.sigma_m > 0 and not args.grade else None
        initial = args.grade or ("custom" if self._custom else "recreational")
        self._lock = threading.Lock()
        self._settings = Settings(grade=initial)
        self._last_t_ns: int | None = None
        self._fix_slot: int | None = None
        self._metrics_slot: int | None = None

        def key(subject: str) -> str:
            return keelson.construct_pubsub_key(
                args.realm, args.entity_id, subject, args.source_id
            )

        self._publishers = {subject: declare_publisher(session, key(subject)) for subject in PUBSUB_SUBJECTS}
        self.fixes = 0
        self.blind = 0

    # --- settings, from configurable/v1 ---------------------------------------

    @property
    def settings(self) -> Settings:
        with self._lock:
            return self._settings

    def get_config(self) -> dict:
        config = self.settings.as_config()
        if self._custom is not None:
            config["grades"] = sorted([*config["grades"], "custom"])
        return config

    def set_config(self, request: dict) -> None:
        with self._lock:
            grade = request.get("grade") if isinstance(request, dict) else None
            if grade == "custom" and self._custom is not None:
                # Not one of the named grades, so `merge` would refuse it.
                request = {k: v for k, v in request.items() if k != "grade"}
                merged = merge(self._settings, request, self._last_t_ns)
                self._settings = replace(merged, grade="custom")
            else:
                self._settings = merge(self._settings, request, self._last_t_ns)
        logger.info("[GNSS] settings now %s", self.get_config())

    def _grade(self, name: str):
        if name == "custom" and self._custom is not None:
            return self._custom
        return GRADES[name]

    # --- truth in, observation out --------------------------------------------

    @property
    def truth_key(self) -> str:
        return self._source.key(self._args.realm, self._args.entity_id)

    def on_truth(self, sample) -> None:
        fix = self._source.fix(sample.payload.to_bytes(), self._args.entity_id)
        if fix is None:
            return
        t_ns = fix.timestamp_ns
        with self._lock:
            self._last_t_ns = t_ns
            settings = self._settings

        # Failed by the simulator, or switched off by an operator. A real one
        # reports nothing usable; publishing a stale fix would be the
        # frozen-value lie this component refuses everywhere else.
        if not fix.enabled or not settings.receiver_on:
            self.blind += 1
            return

        grade = self._grade(settings.grade)
        solution = solve(
            fix.entity_id,
            fix.latitude_deg,
            fix.longitude_deg,
            grade,
            settings.faults,
            fix.seed,
            fix.tick,
            t_ns,
        )

        metrics_slot = t_ns // METRICS_PERIOD_NS
        if metrics_slot != self._metrics_slot:
            self._metrics_slot = metrics_slot
            self._publish_metrics(solution, t_ns)

        fix_slot = t_ns // max(1, int(NS / grade.rate_hz))
        if fix_slot == self._fix_slot:
            return
        self._fix_slot = fix_slot

        # Losing lock — the simulator's `gnss_duty` and the operator's
        # `lock_duty` together — or too few satellites for a solution: a gap.
        if not solution.has_fix or not reports_fix(
            fix.seed, fix.entity_id, fix.tick, fix.duty * settings.lock_duty
        ):
            self.blind += 1
            return
        self._publish_fix(solution, t_ns)

    def _publish_fix(self, solution: Solution, t_ns: int) -> None:
        message = LocationFix()
        message.timestamp.FromNanoseconds(t_ns)
        message.frame_id = f"{self._args.entity_id}/{self._args.source_id}"
        message.latitude = solution.latitude_deg
        message.longitude = solution.longitude_deg
        # What the receiver believes, east-north-up, row-major 3x3.
        message.position_covariance.extend(
            [
                solution.sigma_east_m**2, 0.0, 0.0,
                0.0, solution.sigma_north_m**2, 0.0,
                0.0, 0.0, solution.sigma_up_m**2,
            ]
        )
        message.position_covariance_type = LocationFix.PositionCovarianceType.DIAGONAL_KNOWN
        # Sim time on the envelope too: `enclose` defaults to the wall clock,
        # which a run on an imported recording's epoch is months away from.
        self._publishers["location_fix"].put(
            keelson.enclose(message.SerializeToString(), enclosed_at=t_ns)
        )
        self.fixes += 1

    def _publish_metrics(self, solution: Solution, t_ns: int) -> None:
        quality = LocationFixQuality()
        quality.timestamp.FromNanoseconds(t_ns)
        quality.fix_type = solution.fix_type
        quality.pos_type = solution.pos_type
        quality.rtk_status = solution.rtk_status
        quality.integrity = solution.integrity
        self._publishers["location_fix_quality"].put(
            keelson.enclose(quality.SerializeToString(), enclosed_at=t_ns)
        )
        ints = {
            "location_fix_satellites_visible": solution.satellites_visible,
            "location_fix_satellites_used": solution.satellites_used,
        }
        for subject, value in ints.items():
            self._publishers[subject].put(_timestamped_int(value, t_ns))
        if not solution.has_fix:
            # No solution, so no DOP and no error estimate: a gap, not a number.
            return
        floats = {
            "location_fix_hdop": solution.hdop,
            "location_fix_vdop": solution.vdop,
            "location_fix_pdop": solution.pdop,
            "gnss_error_rms_m": solution.error_rms_m,
            "gnss_error_semi_major_axis_m": max(solution.sigma_east_m, solution.sigma_north_m),
            "gnss_error_semi_minor_axis_m": min(solution.sigma_east_m, solution.sigma_north_m),
            "gnss_error_orientation_deg": 0.0,
            "gnss_error_latitude_m": solution.sigma_north_m,
            "gnss_error_longitude_m": solution.sigma_east_m,
            "gnss_error_altitude_m": solution.sigma_up_m,
        }
        for subject, value in floats.items():
            self._publishers[subject].put(_timestamped_float(value, t_ns))


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
        # keelson's own interface: get_config / set_config, and every applied
        # configuration republished on `configuration_json` for the record.
        make_configurable(
            session,
            base_path=args.realm,
            entity_id=args.entity_id,
            responder_id=args.source_id,
            get_config_cb=receiver.get_config,
            set_config_cb=receiver.set_config,
        )
        with declare_liveliness(
            session,
            args.realm,
            args.entity_id,
            args.source_id,
            pubsub_subjects=PUBSUB_SUBJECTS,
        ):
            logger.info(
                "[GNSS] %s/%s fitted to %s, grade %s",
                args.entity_id,
                args.source_id,
                args.realm,
                receiver.settings.grade,
            )
            with GracefulShutdown(SHUTDOWN_SIGNALS) as shutdown:
                while not shutdown.is_requested():
                    shutdown.wait(timeout=1.0)
    logger.info("[GNSS] %d fixes, %d blind", receiver.fixes, receiver.blind)


if __name__ == "__main__":
    sys.exit(main())
