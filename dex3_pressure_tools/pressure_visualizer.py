#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
import math
import os
import time
import warnings

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

try:
    import yaml
except ImportError:  # pragma: no cover - package.xml declares python3-yaml
    yaml = None

from unitree_hg.msg import HandState


# Live 214-R-T HandState.motor_state order observed from RViz/joint motion:
# thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1.
RIGHT_HAND_JOINT_NAMES = [
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
]

G1_BODY_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

LEFT_HAND_JOINT_NAMES = [
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
]

@dataclass(frozen=True)
class TaxelSpec:
    group_id: int
    cell_id: int
    name: str
    frame_id: str
    position: tuple
    orientation: tuple
    scale: tuple
    label_offset: tuple

    @property
    def label(self):
        return f"{self.group_id}:{self.cell_id}"


def _rpy_to_quaternion(rpy):
    roll, pitch, yaw = rpy
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _float_tuple(values, length, field_name):
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"{field_name} must have {length} values. Got {values!r}")
    return result


def _build_explicit_taxel_specs(taxels):
    specs = []
    for taxel in taxels:
        group_id = int(taxel.get("group", taxel.get("group_id")))
        cell_id = int(taxel.get("cell", taxel.get("cell_id")))
        name = str(taxel.get("name", f"group_{group_id}"))
        frame_id = str(taxel["frame_id"])
        position = _float_tuple(taxel["xyz"], 3, f"{name} xyz")
        scale = _float_tuple(taxel["scale"], 3, f"{name} scale")
        label_offset = _float_tuple(
            taxel.get("label_offset", [0.0, 0.0, 0.008]),
            3,
            f"{name} label_offset",
        )
        if "orientation" in taxel:
            orientation = _float_tuple(taxel["orientation"], 4, f"{name} orientation")
        else:
            orientation = _rpy_to_quaternion(
                _float_tuple(taxel.get("rpy", [0.0, 0.0, 0.0]), 3, f"{name} rpy")
            )
        specs.append(
            TaxelSpec(
                group_id=group_id,
                cell_id=cell_id,
                name=name,
                frame_id=frame_id,
                position=position,
                orientation=orientation,
                scale=scale,
                label_offset=label_offset,
            )
        )
    return specs


def _load_taxel_specs(mapping_file):
    if not mapping_file:
        raise ValueError("A Dex3 pressure mapping_file with a non-empty 'taxels' list is required.")
    if mapping_file:
        if yaml is None:
            raise RuntimeError("python3-yaml is required to load a Dex3 pressure mapping file.")
        with open(os.path.expanduser(mapping_file), "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        taxels = data.get("taxels", [])
        if taxels:
            return _build_explicit_taxel_specs(taxels)
        raise ValueError(f"Mapping file {mapping_file!r} does not contain a non-empty 'taxels' list.")


def _clamp(value, low, high):
    return max(low, min(high, value))


class Dex3PressureVisualizer(Node):
    def __init__(self):
        super().__init__("dex3_right_pressure_visualizer")

        self.declare_parameter("state_topic", "/lf/dex3/right/state")
        self.declare_parameter("marker_topic", "/dex3/right/pressure_markers")
        self.declare_parameter("mapping_file", "")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("baseline_samples", 60)
        self.declare_parameter("min_contact_delta", 50.0)
        self.declare_parameter("full_scale_delta", 3000.0)
        self.declare_parameter("invalid_below", 0.0)
        self.declare_parameter("invalid_value", 30000.0)
        self.declare_parameter("invalid_tolerance", 1000.0)
        self.declare_parameter("show_invalid", True)
        self.declare_parameter("show_labels", True)
        self.declare_parameter("enable_heatmap", False)
        self.declare_parameter("heatmap_mode", "delta")
        self.declare_parameter("publish_hand_joint_states", True)
        self.declare_parameter("publish_robot_joint_state_defaults", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("probe_log_period_s", 1.0)
        self.declare_parameter("top_n_log", 8)

        self.state_topic = str(self.get_parameter("state_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        mapping_file = str(self.get_parameter("mapping_file").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.baseline_samples = int(self.get_parameter("baseline_samples").value)
        self.min_contact_delta = float(self.get_parameter("min_contact_delta").value)
        self.full_scale_delta = max(1.0, float(self.get_parameter("full_scale_delta").value))
        self.invalid_below = float(self.get_parameter("invalid_below").value)
        self.invalid_value = float(self.get_parameter("invalid_value").value)
        self.invalid_tolerance = float(self.get_parameter("invalid_tolerance").value)
        self.show_invalid = bool(self.get_parameter("show_invalid").value)
        self.show_labels = bool(self.get_parameter("show_labels").value)
        self.enable_heatmap = bool(self.get_parameter("enable_heatmap").value)
        self.heatmap_mode = str(self.get_parameter("heatmap_mode").value).strip().lower()
        self.publish_hand_joint_states = bool(self.get_parameter("publish_hand_joint_states").value)
        self.publish_robot_joint_state_defaults = bool(
            self.get_parameter("publish_robot_joint_state_defaults").value
        )
        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.probe_log_period_s = float(self.get_parameter("probe_log_period_s").value)
        self.top_n_log = int(self.get_parameter("top_n_log").value)

        if publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0. Got {publish_rate_hz}")
        if self.baseline_samples < 1:
            raise ValueError(f"baseline_samples must be >= 1. Got {self.baseline_samples}")

        self.taxels = _load_taxel_specs(mapping_file)
        self.taxel_lookup = {(taxel.group_id, taxel.cell_id): taxel for taxel in self.taxels}

        self.latest_raw = np.full((9, 12), np.nan, dtype=float)
        self.latest_valid = np.zeros((9, 12), dtype=bool)
        self.latest_delta = np.zeros((9, 12), dtype=float)
        self.contact_threshold = self.min_contact_delta
        self.baseline_noise_p99_abs_delta = np.full((9, 12), np.nan, dtype=float)
        self.latest_motor_positions = None
        self.baseline = None
        self._baseline_buffer = []
        self._last_probe_log = 0.0
        self._sent_delete_all = False

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.VOLATILE

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, marker_qos)
        self.joint_state_pub = None
        if self.publish_hand_joint_states:
            self.joint_state_pub = self.create_publisher(JointState, joint_state_topic, 10)

        self.create_subscription(HandState, self.state_topic, self._state_callback, state_qos)
        self.create_timer(1.0 / publish_rate_hz, self._publish_timer_callback)

        self._heatmap = None
        if self.enable_heatmap:
            self._init_heatmap()

        support_mode = "plus neutral support joints" if self.publish_robot_joint_state_defaults else "only"
        self.get_logger().info(
            f"Listening to {self.state_topic}; publishing {len(self.taxels)} pressure taxels "
            f"on {marker_topic}. Publishing right-hand joint states {support_mode}. "
            "No Dex3 command publishers are created."
        )

    def _state_callback(self, msg):
        raw = np.full((9, 12), np.nan, dtype=float)
        for group_id, press_sensor_state in enumerate(msg.press_sensor_state[:9]):
            pressures = list(press_sensor_state.pressure[:12])
            raw[group_id, : len(pressures)] = [float(value) for value in pressures]

        valid = self._valid_pressure_mask(raw)
        raw_for_baseline = np.where(valid, raw, np.nan)
        self.latest_raw = raw
        self.latest_valid = valid

        motor_positions = []
        for motor_state in msg.motor_state[: len(RIGHT_HAND_JOINT_NAMES)]:
            motor_positions.append(float(motor_state.q))
        self.latest_motor_positions = motor_positions

        if self.baseline is None:
            self._baseline_buffer.append(raw_for_baseline)
            if len(self._baseline_buffer) >= self.baseline_samples:
                baseline_samples = np.stack(self._baseline_buffer, axis=0)
                self._finish_baseline(baseline_samples, raw)
                self._baseline_buffer.clear()
            return

        delta = np.where(valid, raw - self.baseline, 0.0)
        delta = np.where(np.isfinite(delta), np.maximum(delta, 0.0), 0.0)
        self.latest_delta = delta
        self._maybe_log_probe_candidates()

    def _finish_baseline(self, baseline_samples, fallback_raw):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            self.baseline = np.nanmedian(baseline_samples, axis=0)

        finite_baseline = np.isfinite(self.baseline)
        self.baseline = np.where(finite_baseline, self.baseline, fallback_raw)

        abs_delta = np.abs(baseline_samples - self.baseline.reshape((1, 9, 12)))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            self.baseline_noise_p99_abs_delta = np.nanquantile(abs_delta, 0.99, axis=0)

        active_noise_thresholds = [
            float(self.baseline_noise_p99_abs_delta[taxel.group_id, taxel.cell_id])
            for taxel in self.taxels
            if finite_baseline[taxel.group_id, taxel.cell_id]
            and np.isfinite(self.baseline_noise_p99_abs_delta[taxel.group_id, taxel.cell_id])
        ]
        if active_noise_thresholds:
            median_noise_threshold = float(np.median(active_noise_thresholds))
            max_noise_threshold = float(np.max(active_noise_thresholds))
            self.contact_threshold = max(self.min_contact_delta, max_noise_threshold)
            self.get_logger().info(
                "Dex3 pressure baseline ready from "
                f"{self.baseline_samples} samples. "
                f"Noise threshold median={median_noise_threshold:.1f}, "
                f"max={max_noise_threshold:.1f}; "
                f"using contact_threshold={self.contact_threshold:.1f} raw counts."
            )
        else:
            self.get_logger().warn(
                f"Dex3 pressure baseline ready from {self.baseline_samples} samples, "
                "but no mapped taxels had finite valid baselines."
            )

    def _valid_pressure_mask(self, raw):
        finite = np.isfinite(raw)
        far_from_invalid_value = np.abs(raw - self.invalid_value) > self.invalid_tolerance
        above_invalid_floor = raw >= self.invalid_below
        return finite & far_from_invalid_value & above_invalid_floor

    def _publish_timer_callback(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        if not self._sent_delete_all:
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            marker_array.markers.append(delete_all)
            self._sent_delete_all = True

        for marker_id, taxel in enumerate(self.taxels):
            raw = self.latest_raw[taxel.group_id, taxel.cell_id]
            valid = bool(self.latest_valid[taxel.group_id, taxel.cell_id])
            delta = float(self.latest_delta[taxel.group_id, taxel.cell_id])
            marker_array.markers.append(
                self._make_taxel_marker(marker_id, taxel, now, raw, valid, delta)
            )
            if self.show_labels:
                marker_array.markers.append(self._make_label_marker(marker_id, taxel, now))

        self.marker_pub.publish(marker_array)

        if self.joint_state_pub is not None:
            self._publish_hand_joint_state(now)

        if self._heatmap is not None:
            self._update_heatmap()

    def _make_taxel_marker(self, marker_id, taxel, stamp, raw, valid, delta):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = taxel.frame_id
        marker.ns = "dex3_right_pressure_taxels"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.x = taxel.orientation[0]
        marker.pose.orientation.y = taxel.orientation[1]
        marker.pose.orientation.z = taxel.orientation[2]
        marker.pose.orientation.w = taxel.orientation[3]
        marker.pose.position.x = taxel.position[0]
        marker.pose.position.y = taxel.position[1]
        marker.pose.position.z = taxel.position[2]
        marker.scale.x = max(0.0005, taxel.scale[0])
        marker.scale.y = max(0.0005, taxel.scale[1])
        marker.scale.z = max(0.0005, taxel.scale[2])
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self._color_for_taxel(
            valid, delta
        )
        return marker

    def _make_label_marker(self, marker_id, taxel, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = taxel.frame_id
        marker.ns = "dex3_right_pressure_labels"
        marker.id = marker_id + 1000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.x = taxel.orientation[0]
        marker.pose.orientation.y = taxel.orientation[1]
        marker.pose.orientation.z = taxel.orientation[2]
        marker.pose.orientation.w = taxel.orientation[3]
        marker.pose.position.x = taxel.position[0] + taxel.label_offset[0]
        marker.pose.position.y = taxel.position[1] + taxel.label_offset[1]
        marker.pose.position.z = taxel.position[2] + taxel.label_offset[2]
        marker.scale.z = 0.007
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.85
        marker.text = taxel.label
        return marker

    def _color_for_taxel(self, valid, delta):
        if not valid:
            if self.show_invalid:
                return (0.35, 0.35, 0.35, 0.20)
            return (0.35, 0.35, 0.35, 0.0)

        if self.baseline is None or delta < self.contact_threshold:
            return (0.0, 0.42, 1.0, 0.50)

        level = _clamp((delta - self.contact_threshold) / self.full_scale_delta, 0.0, 1.0)
        red = 1.0
        green = 0.85 * (1.0 - level)
        blue = 0.05
        alpha = 0.45 + 0.45 * level
        return (red, green, blue, alpha)

    def _publish_hand_joint_state(self, stamp):
        if not self.latest_motor_positions:
            return

        count = min(len(self.latest_motor_positions), len(RIGHT_HAND_JOINT_NAMES))
        if count == 0:
            return

        msg = JointState()
        msg.header.stamp = stamp
        if self.publish_robot_joint_state_defaults:
            msg.name = G1_BODY_JOINT_NAMES + LEFT_HAND_JOINT_NAMES + RIGHT_HAND_JOINT_NAMES[:count]
            msg.position = (
                [0.0] * len(G1_BODY_JOINT_NAMES)
                + [0.0] * len(LEFT_HAND_JOINT_NAMES)
                + self.latest_motor_positions[:count]
            )
        else:
            msg.name = RIGHT_HAND_JOINT_NAMES[:count]
            msg.position = self.latest_motor_positions[:count]
        self.joint_state_pub.publish(msg)

    def _maybe_log_probe_candidates(self):
        if self.probe_log_period_s <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_probe_log < self.probe_log_period_s:
            return
        self._last_probe_log = now

        candidates = []
        for taxel in self.taxels:
            delta = float(self.latest_delta[taxel.group_id, taxel.cell_id])
            if delta >= self.contact_threshold:
                raw = float(self.latest_raw[taxel.group_id, taxel.cell_id])
                candidates.append((delta, raw, taxel))

        if not candidates:
            return

        candidates.sort(key=lambda item: item[0], reverse=True)
        chunks = []
        for delta, raw, taxel in candidates[: self.top_n_log]:
            chunks.append(f"{taxel.name}({taxel.group_id},{taxel.cell_id}) d={delta:.0f} raw={raw:.0f}")
        self.get_logger().info("Dex3 pressure probe top changes: " + "; ".join(chunks))

    def _init_heatmap(self):
        try:
            import matplotlib

            if not os.environ.get("DISPLAY"):
                matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover - depends on desktop packages
            self.get_logger().warn(f"Could not start Dex3 pressure heatmap: {exc}")
            self._heatmap = None
            return

        plt.ion()
        fig, ax = plt.subplots(num="Dex3 Right Pressure")
        cmap = plt.get_cmap("inferno").copy()
        cmap.set_bad(color="#30343b")
        data = np.ma.masked_invalid(np.zeros((9, 12), dtype=float))
        image = ax.imshow(data, interpolation="nearest", aspect="auto", cmap=cmap)
        ax.set_xlabel("pressure[cell]")
        ax.set_ylabel("press_sensor_state[group]")
        ax.set_xticks(range(12))
        ax.set_yticks(range(9))
        fig.colorbar(image, ax=ax)
        fig.tight_layout()
        self._heatmap = (plt, fig, ax, image)

    def _update_heatmap(self):
        plt, fig, ax, image = self._heatmap
        if self.heatmap_mode == "raw":
            data = np.where(self.latest_valid, self.latest_raw, np.nan)
            finite = data[np.isfinite(data)]
            if finite.size:
                image.set_clim(float(np.nanmin(finite)), float(np.nanmax(finite)))
            ax.set_title("Dex3 right pressure raw")
        else:
            data = np.where(self.latest_valid, self.latest_delta, np.nan)
            image.set_clim(0.0, self.full_scale_delta)
            ax.set_title("Dex3 right pressure delta")

        image.set_data(np.ma.masked_invalid(data))
        fig.canvas.draw_idle()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = Dex3PressureVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
