# keelson-sil-gnss

A simulated GNSS receiver for software-in-the-loop testing. It reads where a
vessel actually is, decides what a receiver of a given grade would have
reported, and publishes that, and the receiver's own metrics, under its own
source id.

```
truth  ──►  receiver model  ──►  rise/@v0/<entity>/pubsub/location_fix/gnss
                                 …/location_fix_satellites_used/gnss, …hdop, …quality, GST
```

```bash
keelson-sil-gnss --realm rise --entity-id slipway-alpha \
    --source-id gnss --grade professional --connect tcp/127.0.0.1:7447
```

One process per fitted device, as on a real platform.

## Grades

| grade | fix rate | per-axis error (wander / jitter) | wander timescale | solution | satellites in view |
|---|---|---|---|---|---|
| `recreational` | 1 Hz | 1.5 m / 0.3 m | ~3 min | SBAS, single frequency | 14–20 |
| `professional` | 10 Hz | 0.4 m / 0.08 m | ~2 min | DGNSS, dual frequency | 25–35 |
| `survey` | 10 Hz | 8 mm / 4 mm | ~1 min | RTK fixed (RTK float without corrections) | 25–35 |

The figures are nominal, spec-sheet-class values for each kind of receiver,
not measurements of a particular unit. `--sigma-m` still describes a receiver
by its total per-axis error alone, as the grade `custom`.

**The error wanders; it does not jump.** Most of a real receiver's error
changes over minutes, so consecutive fixes sit close together and the whole
cloud drifts. Each axis is a correlated part (hashed values at knots, smoothly
interpolated) plus a small per-fix jitter. Accuracy follows the satellites: HDOP
grows as fewer are used, and the error grows with it.

## What it publishes

- `location_fix` at the grade's rate, with the receiver's own covariance
- once a second: `location_fix_satellites_visible`, `…_satellites_used`,
  `location_fix_quality` (fix type, solution, RTK status, integrity),
  `location_fix_hdop` / `vdop` / `pdop`, and the GST error statistics
  (`gnss_error_rms_m`, semi-major/minor axes, latitude/longitude/altitude)

With too few satellites there is no fix and no `location_fix` at all: a gap,
never a frozen value. The error estimate is what the receiver *believes*; only
truth knows the actual error, and spoofing does not show in the estimate.

## Live settings

keelson's own `configurable/v1`, served on the receiver's entity under its
source id: `get_config` answers the current settings and `set_config` merges a
JSON object. Every applied setting is republished on `configuration_json`, so a
recording says what was done to the receiver and when.

| key | value | effect |
|---|---|---|
| `grade` | `recreational` / `professional` / `survey` | switch the class of receiver |
| `receiver_on` | bool | off is silence: no fix, no metrics |
| `lock_duty` | 0–1 | fraction of fixes that get out |
| `jamming` | 0–1 | satellites drop, error grows, no fix near 1; above 0.5 corrections are lost |
| `multipath` | 0–1 | extra short-timescale error |
| `sky_mask` | 0–1 | fraction of the sky blocked; fewer satellites, worse geometry |
| `corrections` | bool | SBAS/DGNSS → single, RTK fixed → float |
| `spoofing` | `{offset_m, bearing_deg, drift_mps}` or `null` | fix pulled away while the receiver reports it healthy |
| `clear` | `true` | every fault off, grade kept |

An unknown key or an out-of-range value is refused, not ignored. Settings take
effect on the next sample — real time. The noise is still drawn from
`(seed, instant)`, but when a slider moved is in the `configuration_json`
trail, not in any seed.

## Why it is not part of a simulator

A simulator's job is to say where things are. Deciding what an instrument would
have reported is a different job, with a different rate of change, different
validation evidence and a different owner — and it is the first of a family,
because a compass, a rate gyro, a radar and a camera each want the same
treatment.

Keeping them apart buys two things that matter more than the tidiness:

- **The operational plane carries no exact position at all.** A consumer
  reading `location_fix` is reading a modelled receiver by construction, not by
  anybody's good behaviour. A simulator that published truth under an
  operational name would let software under test read ground truth, and nothing
  would report that it had.
- **Production connectors run unchanged.** The next stage down is already
  somebody's shipped code: `keelson2nmea0183` turns these subjects into GGA,
  RMC, HDT and the rest. Building a simulator that emitted NMEA directly would
  duplicate a connector that already exists and is already tested.

## Reproducibility

The measurement owns no clock and no state. The error at an instant is a pure
function of `(seed, stream, instant)`, drawn from a hash rather than from a
generator, so:

- two runs of the same scenario produce byte-identical fixes;
- a run restored from a checkpoint picks up exactly where it left off, without
  this process knowing a restore happened;
- adding a vessel, or publishing in a different order, cannot shift an existing
  vessel's values.

The last one is the property a per-stream sequential generator loses quietly,
and it is what most of `tests/test_model.py` is about.

## What it reads, and the gap

`sources.py` holds the input adapters, kept separate from the measurement model
so that one model can serve more than one simulator.

**There is one adapter today and it is transitional.** `VesselTruthRecord`
reads Slipway's `vessel_truth` record — a JSON blob riding on `raw_json` under
the `truth` source. That is a private contract of one simulator, and a
component named for the ecosystem should not have to know about it.

It is here because the alternative was worse. Slipway is retiring
`vessel_truth` onto typed keelson subjects published under `/truth`, and when
that lands a `TypedTruth` adapter joins this module and reads the same subjects
any observation uses. Waiting for it would have meant either no repository or
two copies of the measurement model, and two copies of a noise model are
strictly worse than not having split at all.

So this component is **not yet generic in its input**, and the README says so
rather than the name implying otherwise.

## Scope

Position and its receiver metrics only. Heading, course, speed and rate of turn need compass and
rate-gyro models that do not exist here; degrading data no instrument produced
would be the same fiction this component removes.

## Development

```bash
uv sync
uv run pytest -q
uv run ruff check .
docker build -t keelson-sil-gnss .
```

The keelson pin in `pyproject.toml` is exact and matches the build Slipway
runs. The ellipsoid comes from `pymap3d`: keelson#245, which would have put it
in the SDK, was closed on the view that numerical helpers are not the SDK's to
own.
