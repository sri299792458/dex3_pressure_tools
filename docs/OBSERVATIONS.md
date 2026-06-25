# Observations And Operating Notes

These notes document what was learned while probing the right Dex3-1 hand marked
`214-R-T`. They are intentionally separate from the code so lab members can see
which details were observed locally and which details came from external docs.

## State Topics

The right hand was reachable through:

```bash
ros2 topic echo --once --full-length /dex3/right/state unitree_hg/msg/HandState
```

The tools default to:

```text
/lf/dex3/right/state
```

`/lf` is the low-frequency state topic. On `214-R-T`, a short
`ros2 topic hz` run measured about `9.1 Hz`. This is enough for visualization,
calibration prompts, baseline collection, and free-touch logging.

The higher-rate topic is:

```text
/dex3/right/state
```

On `214-R-T`, a short `ros2 topic hz` run measured about `750-770 Hz`:

```text
/dex3/right/state     ~770 Hz
/lf/dex3/right/state  ~9 Hz
```

Use the high-rate topic when studying fast taps, slip, or transient contact
dynamics:

```bash
ros2 launch dex3_pressure_tools dex3_pressure_visualizer.launch.py \
  state_topic:=/dex3/right/state
```

Use `/lf/dex3/right/state` when you want lower bandwidth and stable live RViz
debugging.

To re-measure on a lab machine:

```bash
ros2 topic hz /dex3/right/state
ros2 topic hz /lf/dex3/right/state
```

## Message Structure

The tools read:

```text
unitree_hg/msg/HandState
```

Relevant fields:

```text
MotorState[] motor_state
PressSensorState[] press_sensor_state
```

Each pressure group contains:

```text
float32[12] pressure
float32[12] temperature
uint32 lost
uint32 reserve
```

The pressure array is treated as:

```text
press_sensor_state[group].pressure[cell]
```

with `group = 0..8` and `cell = 0..11`.

## Active Pressure Slots

Raw audit on `214-R-T`:

```text
sample_count = 45
duration = 4.87 s
active tactile slots = 33
stable 30000 slots = 75
```

Verified active slots:

```text
ID 0: pressure[0], [2], [9], [11]
ID 1: pressure[3], [6], [8]
ID 2: pressure[0], [2], [9], [11]
ID 3: pressure[3], [6], [8]
ID 4: pressure[0], [2], [9], [11]
ID 5: pressure[3], [6], [8]
ID 6: pressure[0], [2], [9], [11]
ID 7: pressure[0], [2], [9], [11]
ID 8: pressure[0], [2], [9], [11]
```

The unused slots stayed exactly at:

```text
30000
```

So `30000` is not only an article claim; it was verified on this physical hand.

## Baseline Noise

Baseline-only rerun:

```text
file = /home/kanth042/dex3_pressure_baseline_20260625_132738.json
state_topic = /lf/dex3/right/state
baseline_sample_count = 210
duration = 20.0 s
active tactile slots = 33
stable 30000 slots = 75
```

In the matching `.npz`, every inactive slot stayed exactly `30000` for all 210
samples. No active taxel drifted by `50` raw counts or more from its median.

Untouched baseline noise in that run:

```text
baseline std median = about 5.9 raw counts
baseline std max = about 7.5 raw counts
baseline p99 absolute drift median = 16 raw counts
baseline p99 absolute drift max = 24 raw counts
```

Free-touch session:

```text
sample_count = 1274
baseline_sample_count = 107
free_sample_count = 1167
duration = 139.7 s
```

Untouched baseline noise over active taxels:

```text
baseline std median = about 6 raw counts
baseline std max = about 9 raw counts
baseline p99 absolute drift median = 16 raw counts
baseline p99 absolute drift max = 24 raw counts
```

This is why the visualizer default uses:

```text
min_contact_delta = 50
```

It is above the observed idle drift but still sensitive to light contact.

## Touch Magnitudes

In the same free-touch session:

```text
free-touch max delta median = 2280 raw counts
free-touch max delta max = 13800 raw counts
22 / 33 active taxels crossed 500 raw counts
```

This means `500` is a conservative "definite touch" threshold, while `50` is a
good live visualization threshold for detecting light touches above noise.

## Joint Order

The right-hand `motor_state` order used by the visualizer was observed from live
joint motion in RViz:

```text
0: right_hand_thumb_0_joint
1: right_hand_thumb_1_joint
2: right_hand_thumb_2_joint
3: right_hand_middle_0_joint
4: right_hand_middle_1_joint
5: right_hand_index_0_joint
6: right_hand_index_1_joint
```

This order matters because an incorrect index/middle order makes the wrong
finger move in RViz.

## Mapping Source

The seed pressure layout came from the Meko G1/Dex3-1 article:

https://www.mekosrl.it/post/n-2-2-unitree-g1-umanoide-robot-tutto-ci%C3%B2-che-c-%C3%A8-da-sapere-sviluppo-applicazioni-sdk-descriz

The article states that the figure ID corresponds to `PressSensorState_`, and
the `pressure[12]` indices are marked in the figure. The seed map is useful,
but repeated live touch evidence is still the final authority for this hand.

## Safety Boundary

The tools do not publish Dex3 command messages. They subscribe to hand state and
publish only:

```text
/dex3/right/pressure_markers
/joint_states  # optional, for RViz TF
```

They do not publish:

```text
/dex3/right/cmd
```
