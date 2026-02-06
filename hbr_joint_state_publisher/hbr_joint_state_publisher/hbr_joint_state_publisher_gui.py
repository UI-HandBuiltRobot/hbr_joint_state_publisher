#!/usr/bin/env python3
import math
import tkinter as tk
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState


@dataclass
class JointSpec:
    name: str
    jtype: str
    lower: float
    upper: float


def parse_urdf_joints(urdf_xml: str,
                     continuous_limits: Tuple[float, float] = (-math.pi, math.pi)) -> List[JointSpec]:
    """
    Parse joint names/types/limits from a URDF XML string.

    - Includes: revolute, prismatic, continuous
    - Excludes: fixed
    - For continuous: uses continuous_limits
    - For missing limits on revolute/prismatic: falls back to continuous_limits (safe default)
    """
    root = ET.fromstring(urdf_xml)
    joints: List[JointSpec] = []

    for j in root.findall("joint"):
        name = j.get("name")
        jtype = j.get("type")

        if not name or not jtype:
            continue
        if jtype == "fixed":
            continue
        if jtype not in ("revolute", "prismatic", "continuous"):
            # You can decide to ignore or include others; ignoring is simplest for labs.
            continue

        if jtype == "continuous":
            lo, hi = continuous_limits
        else:
            limit = j.find("limit")
            if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
                try:
                    lo = float(limit.get("lower"))
                    hi = float(limit.get("upper"))
                except ValueError:
                    lo, hi = continuous_limits
            else:
                # Some URDFs omit limits; use safe lab default
                lo, hi = continuous_limits

        # Ensure sane order
        if hi < lo:
            lo, hi = hi, lo

        joints.append(JointSpec(name=name, jtype=jtype, lower=lo, upper=hi))

    return joints


class HBRJointStatePublisherGUI(Node):
    def __init__(self):
        super().__init__("hbr_joint_state_publisher_gui")

        # Parameters (students usually won't touch these)
        self.declare_parameter("robot_description_topic", "robot_description")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("continuous_lower", -math.pi)
        self.declare_parameter("continuous_upper", math.pi)

        self.robot_description_topic = self.get_parameter("robot_description_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        cont_lo = float(self.get_parameter("continuous_lower").value)
        cont_hi = float(self.get_parameter("continuous_upper").value)
        self.continuous_limits = (cont_lo, cont_hi)

        self.pub = self.create_publisher(JointState, "joint_states", 10)

        # QoS to receive latched /robot_description
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.sub = self.create_subscription(
            String,
            self.robot_description_topic,
            self.on_robot_description,
            qos
        )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self._have_model = False
        self.joints: List[JointSpec] = []
        self.positions: List[float] = []

        # Tkinter UI
        self.root = tk.Tk()
        self.root.title("HBR Joint State Publisher GUI")

        self.header_label = tk.Label(self.root, text="Waiting for /robot_description ...")
        self.header_label.pack(padx=10, pady=(10, 6), anchor="w")

        btn_frame = tk.Frame(self.root)
        btn_frame.pack(fill="x", padx=10, pady=(0, 10))

        self.reset_btn = tk.Button(btn_frame, text="Reset to zero", command=self.reset_all, state=tk.DISABLED)
        self.reset_btn.pack(side=tk.LEFT)

        self.copy_btn = tk.Button(btn_frame, text="Copy joint vector", command=self.copy_vector, state=tk.DISABLED)
        self.copy_btn.pack(side=tk.LEFT, padx=(8, 0))

        self.slider_frame = tk.Frame(self.root)
        self.slider_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.scales: List[tk.Scale] = []

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_robot_description(self, msg: String):
        if self._have_model:
            return  # Keep first model to avoid UI rebuild surprises

        urdf_xml = msg.data.strip()
        if not urdf_xml:
            return

        try:
            joints = parse_urdf_joints(urdf_xml, continuous_limits=self.continuous_limits)
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return

        if not joints:
            self.get_logger().warn("No movable joints found in URDF (revolute/prismatic/continuous).")
            return

        self.joints = joints
        self.positions = [0.0] * len(self.joints)

        # Build sliders
        for w in self.slider_frame.winfo_children():
            w.destroy()
        self.scales.clear()

        for idx, js in enumerate(self.joints):
            row = tk.Frame(self.slider_frame)
            row.pack(fill="x", pady=6)

            label = tk.Label(row, text=f"{js.name} ({js.jtype})")
            label.pack(anchor="w")

            # Slider resolution: finer for radians
            res = 0.01

            scale = tk.Scale(
                row,
                from_=js.lower,
                to=js.upper,
                resolution=res,
                orient=tk.HORIZONTAL,
                length=520,
                command=lambda val, i=idx: self.set_position(i, float(val)),
            )
            scale.set(0.0)
            scale.pack(fill="x")
            self.scales.append(scale)

        self.header_label.config(
            text=f"Loaded {len(self.joints)} joints from /{self.robot_description_topic}. Publishing /joint_states."
        )
        self.reset_btn.config(state=tk.NORMAL)
        self.copy_btn.config(state=tk.NORMAL)

        self._have_model = True
        self.get_logger().info(f"Loaded {len(self.joints)} joints from /{self.robot_description_topic}")

    def set_position(self, idx: int, value: float):
        if 0 <= idx < len(self.positions):
            self.positions[idx] = value

    def reset_all(self):
        for i, scale in enumerate(self.scales):
            scale.set(0.0)
            self.positions[i] = 0.0

    def copy_vector(self):
        # Copies a python-ish list like [0.0, 1.57, ...]
        vec_str = "[" + ", ".join(f"{p:.4f}" for p in self.positions) + "]"
        self.root.clipboard_clear()
        self.root.clipboard_append(vec_str)

    def on_timer(self):
        # Publish even if UI hasn't loaded yet? Usually no; but harmless.
        if self._have_model:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [j.name for j in self.joints]
            msg.position = list(self.positions)
            self.pub.publish(msg)

        # Keep Tk alive without blocking ROS
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            # Window closed
            pass

    def on_close(self):
        self.destroy_node()
        rclpy.shutdown()
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main():
    rclpy.init()
    node = HBRJointStatePublisherGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
