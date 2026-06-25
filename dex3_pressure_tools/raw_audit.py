#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from unitree_hg.msg import HandState


class Dex3PressureRawAudit(Node):
    def __init__(self):
        super().__init__("dex3_right_pressure_raw_audit")

        self.declare_parameter("state_topic", "/lf/dex3/right/state")
        self.declare_parameter("seconds", 5.0)
        self.declare_parameter("candidate_invalid_value", 30000.0)
        self.declare_parameter("candidate_invalid_tolerance", 1000.0)
        self.declare_parameter("stable_fraction", 0.95)
        self.declare_parameter("output_file", "")
        self.declare_parameter("save_samples", True)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.seconds = float(self.get_parameter("seconds").value)
        self.candidate_invalid_value = float(
            self.get_parameter("candidate_invalid_value").value
        )
        self.candidate_invalid_tolerance = float(
            self.get_parameter("candidate_invalid_tolerance").value
        )
        self.stable_fraction = float(self.get_parameter("stable_fraction").value)
        self.output_file = str(self.get_parameter("output_file").value).strip()
        self.save_samples = bool(self.get_parameter("save_samples").value)

        self.samples = []
        self.sample_times = []

        qos = QoSProfile(depth=200)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(HandState, self.state_topic, self._state_callback, qos)

    def _state_callback(self, msg):
        raw = np.full((9, 12), np.nan, dtype=float)
        for group_id, press_sensor_state in enumerate(msg.press_sensor_state[:9]):
            pressures = list(press_sensor_state.pressure[:12])
            raw[group_id, : len(pressures)] = [float(value) for value in pressures]
        self.samples.append(raw)
        self.sample_times.append(time.time())


def _default_output_file():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.abspath(f"dex3_pressure_raw_audit_{stamp}.json")


def _summarize(node):
    if not node.samples:
        raise RuntimeError(f"No samples arrived on {node.state_topic}")

    data = np.stack(node.samples, axis=0)
    with np.errstate(invalid="ignore"):
        median = np.nanmedian(data, axis=0)
        minimum = np.nanmin(data, axis=0)
        maximum = np.nanmax(data, axis=0)
        std = np.nanstd(data, axis=0)

    near_candidate = (
        np.abs(data - node.candidate_invalid_value) <= node.candidate_invalid_tolerance
    )
    near_fraction = np.mean(near_candidate, axis=0)
    stable_candidate = near_fraction >= node.stable_fraction

    slots = []
    for group_id in range(9):
        for cell_id in range(12):
            slots.append({
                "group": group_id,
                "cell": cell_id,
                "median": float(median[group_id, cell_id]),
                "min": float(minimum[group_id, cell_id]),
                "max": float(maximum[group_id, cell_id]),
                "std": float(std[group_id, cell_id]),
                "candidate_invalid_fraction": float(near_fraction[group_id, cell_id]),
                "stable_candidate_invalid": bool(stable_candidate[group_id, cell_id]),
            })

    return {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "state_topic": node.state_topic,
        "sample_count": int(data.shape[0]),
        "duration_observed_seconds": (
            float(node.sample_times[-1] - node.sample_times[0])
            if len(node.sample_times) > 1 else 0.0
        ),
        "candidate_invalid_value": node.candidate_invalid_value,
        "candidate_invalid_tolerance": node.candidate_invalid_tolerance,
        "stable_fraction": node.stable_fraction,
        "stable_candidate_invalid_count": int(np.sum(stable_candidate)),
        "median_table": median.tolist(),
        "slots": slots,
        "raw_samples": data.tolist() if node.save_samples else [],
    }


def _print_summary(summary):
    print("")
    print("Dex3 raw pressure audit")
    print(f"  state_topic: {summary['state_topic']}")
    print(f"  samples: {summary['sample_count']}")
    print(f"  observed duration: {summary['duration_observed_seconds']:.2f}s")
    print(
        "  candidate invalid sentinel: "
        f"{summary['candidate_invalid_value']:.0f} +/- "
        f"{summary['candidate_invalid_tolerance']:.0f}"
    )
    print(
        "  stable candidate-invalid slots: "
        f"{summary['stable_candidate_invalid_count']}/108"
    )
    print("")
    print("Median raw table, rows=press_sensor_state[group], cols=pressure[cell]:")
    for group_id, row in enumerate(summary["median_table"]):
        values = " ".join(f"{value:8.0f}" for value in row)
        print(f"  group {group_id}: {values}")

    stable = [slot for slot in summary["slots"] if slot["stable_candidate_invalid"]]
    print("")
    if stable:
        print("Slots stable near candidate invalid value:")
        for slot in stable:
            print(
                "  "
                f"{slot['group']}:{slot['cell']} "
                f"median={slot['median']:.0f} "
                f"range={slot['min']:.0f}-{slot['max']:.0f} "
                f"near={100.0 * slot['candidate_invalid_fraction']:.1f}%"
            )
    else:
        print("No slots were stable near the candidate invalid value.")
    print("")


def main(args=None):
    rclpy.init(args=args)
    node = Dex3PressureRawAudit()
    output_file = os.path.expanduser(node.output_file) if node.output_file else _default_output_file()
    output_file = os.path.abspath(output_file)

    print("")
    print("Recording unfiltered Dex3 pressure matrix.")
    print(f"  state_topic: {node.state_topic}")
    print(f"  seconds: {node.seconds:.1f}")
    print(f"  output_file: {output_file}")
    print("Keep the hand untouched if you are auditing invalid/no-sensor slots.")

    deadline = time.monotonic() + max(0.1, node.seconds)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        summary = _summarize(node)
        _print_summary(summary)
        with open(output_file, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2)
        print(f"Wrote {output_file}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
