# Math And Signals

## Raw Data

Each state message provides:

```text
HandState.press_sensor_state[group].pressure[cell]
```

The raw matrix is treated as `9 x 12`. On the tested right hand `214-R-T`, only
33 of those 108 slots are active tactile cells.

The exact active-slot table and sentinel evidence are recorded in
[Observations And Operating Notes](OBSERVATIONS.md).

## Validity

The visualizer treats a slot as valid when:

```python
valid = isfinite(raw)
valid &= abs(raw - invalid_value) > invalid_tolerance
valid &= raw >= invalid_below
```

Current defaults:

```text
invalid_value = 30000
invalid_tolerance = 1000
invalid_below = 0
```

`invalid_below` is intentionally zero. We do not discard low values using an
unverified floor.

## Baseline

At startup, keep the hand untouched. The visualizer stores the first
`baseline_samples` valid samples and computes:

```python
baseline[group, cell] = median(valid_raw_samples)
```

Median is used so a single startup spike does not define the baseline.

## Noise

The live visualizer uses a simple empirical idle-noise guard:

```python
noise[group, cell] = percentile_99(abs(baseline_samples - baseline))
contact_threshold = max(min_contact_delta, max(noise over mapped taxels))
```

For `214-R-T`, a free-touch session measured baseline standard deviation around
5-9 raw counts and baseline p99 absolute drift at or below 24 counts, so
`min_contact_delta=50` is above observed idle noise.

## Delta

After baseline:

```python
delta = raw - baseline
delta = max(delta, 0)
```

`delta` is an uncalibrated raw-count increase above untouched baseline. It is
useful for contact visualization and mapping validation. It is not grams,
newtons, or pascals.

## Color

If `delta < contact_threshold`, the marker is dim blue.

If `delta >= contact_threshold`:

```python
level = clamp((delta - contact_threshold) / full_scale_delta, 0, 1)
```

The marker fades from yellow to red. `full_scale_delta` changes visualization
contrast only; it is not a physical calibration.
