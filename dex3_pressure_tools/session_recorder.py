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


class Dex3PressureSessionRecorder(Node):
    def __init__(self):
        super().__init__("dex3_right_pressure_session_recorder")

        self.declare_parameter("state_topic", "/lf/dex3/right/state")
        self.declare_parameter("baseline_seconds", 5.0)
        self.declare_parameter("record_seconds", 60.0)
        self.declare_parameter("interactive", True)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("prefix", "dex3_pressure_session")
        self.declare_parameter("candidate_invalid_value", 30000.0)
        self.declare_parameter("candidate_invalid_tolerance", 1000.0)
        self.declare_parameter("stable_invalid_fraction", 0.95)
        self.declare_parameter("summary_contact_delta", 500.0)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.baseline_seconds = float(self.get_parameter("baseline_seconds").value)
        self.record_seconds = float(self.get_parameter("record_seconds").value)
        self.interactive = bool(self.get_parameter("interactive").value)
        self.output_dir = str(self.get_parameter("output_dir").value).strip()
        self.prefix = str(self.get_parameter("prefix").value).strip() or "dex3_pressure_session"
        self.candidate_invalid_value = float(
            self.get_parameter("candidate_invalid_value").value
        )
        self.candidate_invalid_tolerance = float(
            self.get_parameter("candidate_invalid_tolerance").value
        )
        self.stable_invalid_fraction = float(
            self.get_parameter("stable_invalid_fraction").value
        )
        self.summary_contact_delta = float(self.get_parameter("summary_contact_delta").value)

        self.samples = []
        self.sample_times = []
        self.motor_positions = []

        qos = QoSProfile(depth=300)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(HandState, self.state_topic, self._state_callback, qos)

    def _state_callback(self, msg):
        raw = np.full((9, 12), np.nan, dtype=float)
        for group_id, press_sensor_state in enumerate(msg.press_sensor_state[:9]):
            pressures = list(press_sensor_state.pressure[:12])
            raw[group_id, : len(pressures)] = [float(value) for value in pressures]

        motor_q = [float(motor_state.q) for motor_state in msg.motor_state]

        self.samples.append(raw)
        self.sample_times.append(time.time())
        self.motor_positions.append(motor_q)


def _default_paths(output_dir, prefix):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    directory = os.path.abspath(os.path.expanduser(output_dir or "."))
    os.makedirs(directory, exist_ok=True)
    base = os.path.join(directory, f"{prefix}_{stamp}")
    return f"{base}.json", f"{base}.npz"


def _spin_for(node, seconds):
    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def _safe_quantile(values, q, axis=0):
    with np.errstate(all="ignore"):
        return np.nanquantile(values, q, axis=axis)


def _summarize_slot(group_id, cell_id, baseline, free_delta, active_mask, summary_delta):
    baseline_slot = baseline[:, group_id, cell_id]
    free_slot = free_delta[:, group_id, cell_id] if free_delta.size else np.array([])
    finite_baseline = baseline_slot[np.isfinite(baseline_slot)]
    finite_free = free_slot[np.isfinite(free_slot)]

    if finite_baseline.size:
        baseline_median = float(np.nanmedian(finite_baseline))
        baseline_min = float(np.nanmin(finite_baseline))
        baseline_max = float(np.nanmax(finite_baseline))
        baseline_std = float(np.nanstd(finite_baseline))
        baseline_noise_p99 = float(_safe_quantile(np.abs(finite_baseline - baseline_median), 0.99))
    else:
        baseline_median = baseline_min = baseline_max = baseline_std = np.nan
        baseline_noise_p99 = np.nan

    if finite_free.size:
        free_max = float(np.nanmax(finite_free))
        free_p50 = float(_safe_quantile(finite_free, 0.50))
        free_p90 = float(_safe_quantile(finite_free, 0.90))
        free_p95 = float(_safe_quantile(finite_free, 0.95))
        free_p99 = float(_safe_quantile(finite_free, 0.99))
        over_noise = int(np.sum(finite_free >= baseline_noise_p99))
        noise_fraction = float(over_noise / finite_free.size)
        over_summary = int(np.sum(finite_free >= summary_delta))
        summary_fraction = float(over_summary / finite_free.size)
    else:
        free_max = free_p50 = free_p90 = free_p95 = free_p99 = np.nan
        over_noise = 0
        noise_fraction = 0.0
        over_summary = 0
        summary_fraction = 0.0

    return {
        "group": group_id,
        "cell": cell_id,
        "active_taxel": bool(active_mask[group_id, cell_id]),
        "baseline_median": baseline_median,
        "baseline_min": baseline_min,
        "baseline_max": baseline_max,
        "baseline_std": baseline_std,
        "baseline_noise_p99_abs_delta": baseline_noise_p99,
        "free_delta_max": free_max,
        "free_delta_p50": free_p50,
        "free_delta_p90": free_p90,
        "free_delta_p95": free_p95,
        "free_delta_p99": free_p99,
        "free_samples_over_baseline_noise_p99": over_noise,
        "free_noise_fraction": noise_fraction,
        "free_samples_over_summary_contact_delta": over_summary,
        "free_summary_contact_fraction": summary_fraction,
    }


def _build_summary(node, baseline_end_index):
    if not node.samples:
        raise RuntimeError(f"No samples arrived on {node.state_topic}")

    raw = np.stack(node.samples, axis=0)
    sample_times = np.array(node.sample_times, dtype=float)
    relative_time = sample_times - sample_times[0]
    baseline = raw[:baseline_end_index]
    free = raw[baseline_end_index:]
    if baseline.size == 0:
        raise RuntimeError("No baseline samples were recorded.")

    candidate_invalid = (
        np.abs(baseline - node.candidate_invalid_value) <= node.candidate_invalid_tolerance
    )
    candidate_fraction = np.mean(candidate_invalid, axis=0)
    stable_invalid_mask = candidate_fraction >= node.stable_invalid_fraction
    active_mask = ~stable_invalid_mask

    masked_baseline = np.where(active_mask.reshape((1, 9, 12)), baseline, np.nan)
    baseline_median = np.nanmedian(masked_baseline, axis=0)
    baseline_std = np.nanstd(masked_baseline, axis=0)
    baseline_abs_delta = np.abs(masked_baseline - baseline_median.reshape((1, 9, 12)))
    baseline_noise_p99 = _safe_quantile(baseline_abs_delta, 0.99, axis=0)

    if free.size:
        free_delta = free - baseline_median.reshape((1, 9, 12))
        free_delta = np.where(np.isfinite(free_delta), np.maximum(free_delta, 0.0), np.nan)
        free_delta = np.where(active_mask.reshape((1, 9, 12)), free_delta, np.nan)
    else:
        free_delta = np.empty((0, 9, 12), dtype=float)

    slots = []
    for group_id in range(9):
        for cell_id in range(12):
            slots.append(
                _summarize_slot(
                    group_id,
                    cell_id,
                    baseline,
                    free_delta,
                    active_mask,
                    node.summary_contact_delta,
                )
            )

    active_slots = [slot for slot in slots if slot["active_taxel"]]
    stable_invalid_slots = [slot for slot in slots if not slot["active_taxel"]]
    strongest_by_max = sorted(
        active_slots,
        key=lambda slot: (
            -1.0 if not np.isfinite(slot["free_delta_max"]) else slot["free_delta_max"]
        ),
        reverse=True,
    )[:15]
    strongest_by_p95 = sorted(
        active_slots,
        key=lambda slot: (
            -1.0 if not np.isfinite(slot["free_delta_p95"]) else slot["free_delta_p95"]
        ),
        reverse=True,
    )[:15]

    summary_threshold_hits = []
    for slot in active_slots:
        if slot["free_delta_max"] >= node.summary_contact_delta:
            summary_threshold_hits.append(slot)
    summary_threshold_hits.sort(key=lambda slot: slot["free_delta_max"], reverse=True)

    return {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "state_topic": node.state_topic,
        "sample_count": int(raw.shape[0]),
        "baseline_sample_count": int(baseline.shape[0]),
        "free_sample_count": int(free.shape[0]),
        "duration_observed_seconds": float(relative_time[-1]) if relative_time.size else 0.0,
        "baseline_seconds_requested": node.baseline_seconds,
        "record_seconds_requested": node.record_seconds,
        "candidate_invalid_value": node.candidate_invalid_value,
        "candidate_invalid_tolerance": node.candidate_invalid_tolerance,
        "stable_invalid_fraction": node.stable_invalid_fraction,
        "stable_invalid_count": int(np.sum(stable_invalid_mask)),
        "active_taxel_count": int(np.sum(active_mask)),
        "summary_contact_delta": node.summary_contact_delta,
        "baseline_median_table": baseline_median.tolist(),
        "baseline_std_table": baseline_std.tolist(),
        "baseline_noise_p99_abs_delta_table": baseline_noise_p99.tolist(),
        "slots": slots,
        "strongest_by_max_delta": strongest_by_max,
        "strongest_by_p95_delta": strongest_by_p95,
        "hits_above_summary_contact_delta": summary_threshold_hits,
    }, raw, relative_time, np.array(node.motor_positions, dtype=object), baseline_end_index


def _print_slot(slot):
    return (
        f"{slot['group']}:{slot['cell']} "
        f"max={slot['free_delta_max']:.0f} "
        f"p95={slot['free_delta_p95']:.0f} "
        f"noise_p99={slot['baseline_noise_p99_abs_delta']:.1f} "
        f"summary_frac={100.0 * slot['free_summary_contact_fraction']:.1f}%"
    )


def _print_baseline_slot(slot):
    return (
        f"{slot['group']}:{slot['cell']} "
        f"median={slot['baseline_median']:.0f} "
        f"std={slot['baseline_std']:.1f} "
        f"noise_p99={slot['baseline_noise_p99_abs_delta']:.1f}"
    )


def _print_summary(summary, json_path, npz_path):
    print("")
    print("Dex3 pressure session summary")
    print(f"  samples: {summary['sample_count']}")
    print(f"  baseline samples: {summary['baseline_sample_count']}")
    print(f"  free-touch samples: {summary['free_sample_count']}")
    print(f"  observed duration: {summary['duration_observed_seconds']:.2f}s")
    print(f"  active taxels: {summary['active_taxel_count']}")
    print(f"  stable invalid slots: {summary['stable_invalid_count']}")
    print("")
    if summary["free_sample_count"] == 0:
        noisiest = sorted(
            [slot for slot in summary["slots"] if slot["active_taxel"]],
            key=lambda slot: (
                -1.0
                if not np.isfinite(slot["baseline_noise_p99_abs_delta"])
                else slot["baseline_noise_p99_abs_delta"]
            ),
            reverse=True,
        )[:10]
        print("Baseline-only run; no free-touch samples recorded.")
        print("Noisiest baseline taxels by p99 absolute delta:")
        for slot in noisiest:
            print("  " + _print_baseline_slot(slot))
        print(f"Wrote {json_path}")
        print(f"Wrote {npz_path}")
        return

    print("Strongest free-touch taxels by max delta:")
    for slot in summary["strongest_by_max_delta"][:10]:
        print("  " + _print_slot(slot))
    print("")
    print("Strongest free-touch taxels by p95 delta:")
    for slot in summary["strongest_by_p95_delta"][:10]:
        print("  " + _print_slot(slot))
    print("")
    print(
        "Taxels with max delta above "
        f"{summary['summary_contact_delta']:.0f}: "
        f"{len(summary['hits_above_summary_contact_delta'])}"
    )
    print(f"Wrote {json_path}")
    print(f"Wrote {npz_path}")


def main(args=None):
    rclpy.init(args=args)
    node = Dex3PressureSessionRecorder()
    json_path, npz_path = _default_paths(node.output_dir, node.prefix)
    baseline_only = node.record_seconds <= 0.0

    try:
        print("")
        title = "baseline recorder" if baseline_only else "free-touch recorder"
        print(f"Dex3 right pressure {title}")
        print(f"  state_topic: {node.state_topic}")
        print(f"  baseline_seconds: {node.baseline_seconds:.1f}")
        print(f"  record_seconds: {node.record_seconds:.1f}")
        print(f"  json: {json_path}")
        print(f"  npz: {npz_path}")
        print("")

        if node.interactive:
            input("Keep the hand untouched. Press Enter to record baseline...")
        baseline_start = len(node.samples)
        _spin_for(node, node.baseline_seconds)
        baseline_end = len(node.samples)
        baseline_count = baseline_end - baseline_start
        print(f"Recorded {baseline_count} baseline samples.")

        if not baseline_only and node.interactive:
            input("Now freely touch/tap/press the hand with objects. Press Enter to start...")
        if not baseline_only:
            _spin_for(node, node.record_seconds)

        summary, raw, relative_time, motor_positions, baseline_end_index = _build_summary(
            node, baseline_end
        )
        with open(json_path, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2)
        np.savez_compressed(
            npz_path,
            raw=raw,
            relative_time=relative_time,
            motor_positions=motor_positions,
            baseline_end_index=np.array([baseline_end_index], dtype=int),
        )
        _print_summary(summary, json_path, npz_path)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
