# Lab Protocols

This is the bench runbook. It keeps the copy-paste commands and operator steps
in one place; rationale and measured evidence live in the other docs.

## 1. Raw Sentinel Audit

Run this once for a newly connected hand:

```bash
ros2 run dex3_pressure_tools dex3_pressure_raw_audit --ros-args \
  -p seconds:=5.0 \
  -p output_file:=/home/kanth042/dex3_pressure_raw_audit.json
```

Keep the hand untouched. The audit prints the median raw table and reports how
many slots are stable near `30000`.

Expected for right hand `214-R-T`:

```text
75 slots stable near 30000
33 active tactile slots
```

## 2. Free-Touch Session

Use this to measure real noise and touch magnitude:

```bash
ros2 run dex3_pressure_tools dex3_pressure_session_recorder --ros-args \
  -p baseline_seconds:=10.0 \
  -p record_seconds:=120.0 \
  -p output_dir:=/home/kanth042
```

Flow:

1. Keep the hand untouched and press Enter.
2. Wait for baseline recording to finish.
3. Press Enter again.
4. Touch, tap, press, and slide objects over the hand for the recording window.

Outputs:

- Summary JSON with per-taxel baseline and free-touch statistics.
- Compressed `.npz` with raw time series.

For a baseline-only run, keep the hand untouched and set `record_seconds:=0.0`:

```bash
ros2 run dex3_pressure_tools dex3_pressure_session_recorder --ros-args \
  -p baseline_seconds:=20.0 \
  -p record_seconds:=0.0 \
  -p prefix:=dex3_pressure_baseline \
  -p output_dir:=/home/kanth042
```

This writes the same JSON/NPZ formats, but the summary only reports idle
baseline statistics and the noisiest taxels.

## 3. RViz Visualization

The default launch starts `robot_state_publisher`, RViz, and the pressure
visualizer using the packaged G1/Dex3 URDF. It subscribes to
`/lf/dex3/right/state` by default:

```bash
ros2 launch dex3_pressure_tools dex3_pressure_visualizer.launch.py
```

If another process already publishes robot TF and `/robot_description`, disable
the packaged model:

```bash
ros2 launch dex3_pressure_tools dex3_pressure_visualizer.launch.py \
  start_robot_state_publisher:=false \
  publish_robot_joint_state_defaults:=false
```

Keep the hand untouched until the visualizer logs that the baseline is ready.

Use the high-rate state topic for fast contact dynamics:

```bash
ros2 launch dex3_pressure_tools dex3_pressure_visualizer.launch.py \
  state_topic:=/dex3/right/state
```

## 4. Guided Mapping Check

Use this when a marker appears to be on the wrong physical taxel:

```bash
ros2 run dex3_pressure_tools dex3_pressure_calibrator --ros-args \
  -p repeats:=3 \
  -p top_n:=10
```

For each target:

1. Keep clear, press Enter, wait for the baseline window.
2. Touch the shown label, press Enter, hold for the touch window.

The output JSON records the strongest changed raw pressure slots for each
target. A mapping change is trustworthy only when repeated touches agree.

## 5. Launch Parameters

Useful visualizer parameters:

```bash
ros2 launch dex3_pressure_tools dex3_pressure_visualizer.launch.py \
  baseline_samples:=60 \
  min_contact_delta:=50.0 \
  full_scale_delta:=3000.0 \
  show_labels:=true
```

Meaning:

- `baseline_samples`: untouched samples used for median baseline.
- `min_contact_delta`: minimum raw-count increase required for contact display.
- `full_scale_delta`: color contrast range after threshold.
- `show_labels`: display `group:cell` text labels on markers.
