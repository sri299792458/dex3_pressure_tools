#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import threading
import time
import warnings

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from unitree_hg.msg import HandState


DEFAULT_REGIONS = [
    "ID 0 pressure[11] thumb_pad",
    "ID 0 pressure[9] thumb_pad",
    "ID 0 pressure[2] thumb_pad",
    "ID 0 pressure[0] thumb_pad",
    "ID 1 pressure[3] thumb_tip",
    "ID 1 pressure[6] thumb_tip",
    "ID 1 pressure[8] thumb_tip",
    "ID 2 pressure[9] lower_finger_pad",
    "ID 2 pressure[0] lower_finger_pad",
    "ID 2 pressure[11] lower_finger_pad",
    "ID 2 pressure[2] lower_finger_pad",
    "ID 3 pressure[8] lower_finger_tip",
    "ID 3 pressure[3] lower_finger_tip",
    "ID 3 pressure[6] lower_finger_tip",
    "ID 4 pressure[9] upper_finger_pad",
    "ID 4 pressure[0] upper_finger_pad",
    "ID 4 pressure[11] upper_finger_pad",
    "ID 4 pressure[2] upper_finger_pad",
    "ID 5 pressure[8] upper_finger_tip",
    "ID 5 pressure[3] upper_finger_tip",
    "ID 5 pressure[6] upper_finger_tip",
    "ID 6 pressure[2] palm_middle_side",
    "ID 6 pressure[11] palm_middle_side",
    "ID 6 pressure[0] palm_middle_side",
    "ID 6 pressure[9] palm_middle_side",
    "ID 7 pressure[2] palm_center",
    "ID 7 pressure[11] palm_center",
    "ID 7 pressure[0] palm_center",
    "ID 7 pressure[9] palm_center",
    "ID 8 pressure[2] palm_index_side",
    "ID 8 pressure[11] palm_index_side",
    "ID 8 pressure[0] palm_index_side",
    "ID 8 pressure[9] palm_index_side",
]


class Dex3PressureCalibrator(Node):
    def __init__(self):
        super().__init__("dex3_right_pressure_calibrator")

        self.declare_parameter("state_topic", "/lf/dex3/right/state")
        self.declare_parameter("baseline_seconds", 2.0)
        self.declare_parameter("touch_seconds", 2.0)
        self.declare_parameter("release_seconds", 0.0)
        self.declare_parameter("ready_countdown_seconds", 0.0)
        self.declare_parameter("manual_step", True)
        self.declare_parameter("repeats", 3)
        self.declare_parameter("top_n", 10)
        self.declare_parameter("regions", ",".join(DEFAULT_REGIONS))
        self.declare_parameter("output_file", "")
        self.declare_parameter("invalid_value", 30000.0)
        self.declare_parameter("invalid_tolerance", 1000.0)
        self.declare_parameter("invalid_below", 0.0)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.baseline_seconds = float(self.get_parameter("baseline_seconds").value)
        self.touch_seconds = float(self.get_parameter("touch_seconds").value)
        self.release_seconds = float(self.get_parameter("release_seconds").value)
        self.ready_countdown_seconds = float(self.get_parameter("ready_countdown_seconds").value)
        self.manual_step = bool(self.get_parameter("manual_step").value)
        self.repeats = int(self.get_parameter("repeats").value)
        self.top_n = int(self.get_parameter("top_n").value)
        self.regions = [
            region.strip()
            for region in str(self.get_parameter("regions").value).split(",")
            if region.strip()
        ]
        self.output_file = str(self.get_parameter("output_file").value).strip()
        self.invalid_value = float(self.get_parameter("invalid_value").value)
        self.invalid_tolerance = float(self.get_parameter("invalid_tolerance").value)
        self.invalid_below = float(self.get_parameter("invalid_below").value)

        self._lock = threading.Lock()
        self._latest_raw = None
        self._sample_event = threading.Event()
        self._recording = False
        self._recorded_samples = []

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(HandState, self.state_topic, self._state_callback, qos)

    def _state_callback(self, msg):
        raw = np.full((9, 12), np.nan, dtype=float)
        for group_id, press_sensor_state in enumerate(msg.press_sensor_state[:9]):
            pressures = list(press_sensor_state.pressure[:12])
            raw[group_id, : len(pressures)] = [float(value) for value in pressures]

        with self._lock:
            self._latest_raw = raw
            if self._recording:
                self._recorded_samples.append(raw)
        self._sample_event.set()

    def wait_for_sample(self, timeout=5.0):
        return self._sample_event.wait(timeout)

    def record_window(self, duration_seconds):
        with self._lock:
            self._recorded_samples = []
            self._recording = True
        time.sleep(duration_seconds)
        with self._lock:
            self._recording = False
            samples = list(self._recorded_samples)
        if samples:
            return np.stack(samples, axis=0)
        with self._lock:
            latest = None if self._latest_raw is None else self._latest_raw.copy()
        if latest is None:
            return np.empty((0, 9, 12), dtype=float)
        return latest.reshape((1, 9, 12))

    def valid_mask(self, raw):
        finite = np.isfinite(raw)
        far_from_invalid_value = np.abs(raw - self.invalid_value) > self.invalid_tolerance
        above_invalid_floor = raw >= self.invalid_below
        return finite & far_from_invalid_value & above_invalid_floor


def _median_baseline(node, samples):
    if samples.size == 0:
        return np.full((9, 12), np.nan, dtype=float)
    valid = node.valid_mask(samples)
    masked = np.where(valid, samples, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmedian(masked, axis=0)


def _top_touch_cells(node, baseline, touch_samples, top_n):
    if touch_samples.size == 0:
        return []

    valid = node.valid_mask(touch_samples)
    deltas = np.where(valid, touch_samples - baseline.reshape((1, 9, 12)), np.nan)
    deltas = np.where(np.isfinite(deltas), np.maximum(deltas, 0.0), np.nan)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        peak_delta = np.nanmax(deltas, axis=0)
        peak_raw = np.nanmax(np.where(valid, touch_samples, np.nan), axis=0)

    candidates = []
    for group_id in range(peak_delta.shape[0]):
        for cell_id in range(peak_delta.shape[1]):
            delta = peak_delta[group_id, cell_id]
            raw = peak_raw[group_id, cell_id]
            if not np.isfinite(delta) or not np.isfinite(raw):
                continue
            candidates.append({
                "group": group_id,
                "cell": cell_id,
                "peak_delta": float(delta),
                "peak_raw": float(raw),
            })

    candidates.sort(key=lambda item: item["peak_delta"], reverse=True)
    return candidates[:top_n]


def _count_winners(region_result):
    counts = {}
    for repeat in region_result["repeats"]:
        if not repeat["top"]:
            continue
        winner = repeat["top"][0]
        key = f"{winner['group']}:{winner['cell']}"
        counts[key] = counts.get(key, 0) + 1
    return dict(sorted(counts.items(), key=lambda item: item[1], reverse=True))


def _default_output_file():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.abspath(f"dex3_pressure_calibration_{stamp}.json")


def _countdown(label, seconds):
    seconds = max(0.0, float(seconds))
    whole_seconds = int(seconds)
    if whole_seconds <= 0:
        return
    print(label)
    for remaining in range(whole_seconds, 0, -1):
        print(f"  {remaining}...")
        time.sleep(1.0)
    leftover = seconds - whole_seconds
    if leftover > 0.0:
        time.sleep(leftover)


def _wait_for_step(node, prompt, countdown_label):
    if node.manual_step or node.ready_countdown_seconds <= 0.0:
        input(prompt)
    else:
        _countdown(countdown_label, node.ready_countdown_seconds)


def _run_calibration(node):
    if not node.wait_for_sample():
        raise RuntimeError(f"No samples arrived on {node.state_topic}")

    output_file = os.path.expanduser(node.output_file) if node.output_file else _default_output_file()
    output_file = os.path.abspath(output_file)

    print("")
    print("Dex3 right pressure calibration")
    print(f"  state_topic: {node.state_topic}")
    print(f"  repeats: {node.repeats}")
    print(f"  baseline_seconds: {node.baseline_seconds}")
    print(f"  touch_seconds: {node.touch_seconds}")
    print(f"  release_seconds: {node.release_seconds}")
    print(f"  ready_countdown_seconds: {node.ready_countdown_seconds}")
    print(f"  manual_step: {node.manual_step}")
    print(f"  output_file: {output_file}")
    print("")
    print("Simple flow for each target:")
    print(f"  1. Keep clear, press Enter, wait {node.baseline_seconds:.1f}s.")
    print(f"  2. Touch the shown label, press Enter, hold for {node.touch_seconds:.1f}s.")
    print("Example: if it says 'ID 0 pressure[11]', touch the RViz taxel labelled '0:11'.")
    print("The tool ranks all 9 x 12 raw pressure slots; the winner tells us whether that label is correct.")
    if node.manual_step or node.ready_countdown_seconds <= 0.0:
        input("Press Enter when the hand is still, open, and untouched...")
    else:
        _countdown("automatic calibration starts after countdown", node.ready_countdown_seconds)

    results = {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "state_topic": node.state_topic,
        "baseline_seconds": node.baseline_seconds,
        "touch_seconds": node.touch_seconds,
        "release_seconds": node.release_seconds,
        "repeats": node.repeats,
        "top_n": node.top_n,
        "regions": {},
    }

    for region in node.regions:
        print("")
        print(f"=== Target: {region} ===")
        if not node.manual_step and node.ready_countdown_seconds > 0.0:
            _countdown(
                f"automatic target {region}: get ready",
                node.ready_countdown_seconds,
            )
        region_result = {"repeats": []}

        for repeat_idx in range(1, node.repeats + 1):
            print("")
            if node.manual_step:
                input(
                    f"{region} repeat {repeat_idx}/{node.repeats}: "
                    f"KEEP CLEAR, press Enter, then wait {node.baseline_seconds:.1f}s..."
                )
            else:
                _countdown(
                    f"{region} repeat {repeat_idx}/{node.repeats}: keep clear",
                    node.ready_countdown_seconds,
                )
            print(f"repeat {repeat_idx}/{node.repeats}: recording untouched baseline for {node.baseline_seconds:.1f}s")
            baseline_samples = node.record_window(node.baseline_seconds)
            baseline = _median_baseline(node, baseline_samples)

            if node.manual_step:
                input(
                    f"{region} repeat {repeat_idx}/{node.repeats}: "
                    f"TOUCH THIS LABEL, press Enter, then hold for {node.touch_seconds:.1f}s..."
                )
            else:
                _countdown(
                    f"{region} repeat {repeat_idx}/{node.repeats}: touch now",
                    node.ready_countdown_seconds,
                )
            print(f"repeat {repeat_idx}/{node.repeats}: recording TOUCH on {region} for {node.touch_seconds:.1f}s")
            touch_samples = node.record_window(node.touch_seconds)
            top = _top_touch_cells(node, baseline, touch_samples, node.top_n)

            if node.manual_step:
                print("release now; next baseline waits for Enter")
            elif node.release_seconds > 0.0:
                print(f"repeat {repeat_idx}/{node.repeats}: release contact now for {node.release_seconds:.1f}s")
                time.sleep(node.release_seconds)

            summary = ", ".join(
                f"({item['group']},{item['cell']}) d={item['peak_delta']:.0f} raw={item['peak_raw']:.0f}"
                for item in top[:5]
            )
            print("  top:", summary if summary else "no valid response")
            region_result["repeats"].append({
                "repeat": repeat_idx,
                "baseline_sample_count": int(baseline_samples.shape[0]),
                "touch_sample_count": int(touch_samples.shape[0]),
                "top": top,
            })

        region_result["winner_counts"] = _count_winners(region_result)
        print(f"winner counts for {region}: {region_result['winner_counts']}")
        results["regions"][region] = region_result

        with open(output_file, "w", encoding="utf-8") as stream:
            json.dump(results, stream, indent=2)

    print("")
    print(f"Saved calibration results to {output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = Dex3PressureCalibrator()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        _run_calibration(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
