# keelson-sil-gnss

A simulated GNSS receiver for software-in-the-loop testing. It reads where a
vessel actually is, decides what a receiver would have reported, and publishes
that as `location_fix` under its own source id.

```
truth  ──►  measurement model  ──►  rise/@v0/<entity>/pubsub/location_fix/gnss
```

```bash
keelson-sil-gnss --realm rise --entity-id slipway-alpha \
    --source-id gnss --sigma-m 2.5 --connect tcp/127.0.0.1:7447
```

One process per fitted device, as on a real platform. `--sigma-m` is the
standard deviation of the position error in metres, applied independently east
and north — a property of the hardware, not of the exercise.

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

The service owns no clock and no state. The error at a tick is a pure function
of `(seed, stream, tick)`, drawn from a hash rather than from a generator, so:

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

Position only. Heading, course, speed and rate of turn need compass and
rate-gyro models that do not exist here; degrading data no instrument produced
would be the same fiction this component removes.

## Development

```bash
uv sync
uv run pytest -q
uv run ruff check .
docker build -t keelson-sil-gnss .
```

The keelson pin in `pyproject.toml` is currently an **alpha**, cut from the
pull request that adds `keelson.geodesy`. That is a migration pin: replace it
with the `-pre` the moment one carries that module.
