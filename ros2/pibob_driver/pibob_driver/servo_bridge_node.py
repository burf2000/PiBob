"""Bridge ROS 2 /joint_states to the PiBob Flask servo API.

Subscribes to sensor_msgs/JointState on `joint_commands` (remap as needed) and
POSTs each known joint to `<base_url>/set` as a servo angle in degrees.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import requests


@dataclass
class JointCfg:
    channel: int
    sign: int
    home_deg: float
    min_deg: float
    max_deg: float
    last_sent_deg: float = float('nan')
    last_sent_time: float = 0.0


class ServoBridge(Node):
    def __init__(self):
        # The joints.* params are nested in the YAML and not explicitly declared;
        # auto-declaring from overrides makes them (and the scalars below) readable
        # via get_parameter / get_parameters_by_prefix. Without this the bridge loads
        # zero joints ("No joints configured") even with a valid --params-file.
        super().__init__('servo_bridge',
                         automatically_declare_parameters_from_overrides=True)

        self.base_url = self.get_parameter('base_url').get_parameter_value().string_value
        self.timeout = float(self.get_parameter('request_timeout_s').value)
        self.rate_limit_hz = float(self.get_parameter('rate_limit_hz').value)
        self.deadband_deg = float(self.get_parameter('deadband_deg').value)

        self.joints: dict[str, JointCfg] = self._load_joint_map()
        if not self.joints:
            self.get_logger().error(
                'No joints configured. Pass joint_map.yaml via --params-file.')
        else:
            self.get_logger().info(
                f'Bridging {len(self.joints)} joints to {self.base_url}')

        self._session = requests.Session()
        self._lock = threading.Lock()

        self.create_subscription(
            JointState, 'joint_commands', self._on_joint_state, 10)

    def _load_joint_map(self) -> dict[str, JointCfg]:
        """Read the nested `joints.<name>.<field>` parameters into JointCfg objects."""
        prefix = 'joints'
        params = self.get_parameters_by_prefix(prefix)
        # params keys look like "head_pan_joint.channel", "head_pan_joint.sign", ...
        grouped: dict[str, dict[str, float]] = {}
        for key, param in params.items():
            joint, _, field = key.partition('.')
            if not field:
                continue
            grouped.setdefault(joint, {})[field] = param.value

        out: dict[str, JointCfg] = {}
        for name, fields in grouped.items():
            try:
                out[name] = JointCfg(
                    channel=int(fields['channel']),
                    sign=int(fields.get('sign', 1)),
                    home_deg=float(fields.get('home_deg', 90.0)),
                    min_deg=float(fields.get('min_deg', 0.0)),
                    max_deg=float(fields.get('max_deg', 180.0)),
                )
            except KeyError as e:
                self.get_logger().warn(f'Joint {name} missing field {e}; skipped')
        return out

    def _on_joint_state(self, msg: JointState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        min_dt = 1.0 / self.rate_limit_hz if self.rate_limit_hz > 0 else 0.0

        for name, position in zip(msg.name, msg.position):
            cfg = self.joints.get(name)
            if cfg is None:
                continue

            deg = cfg.home_deg + cfg.sign * math.degrees(position)
            deg = max(cfg.min_deg, min(cfg.max_deg, deg))

            if not math.isnan(cfg.last_sent_deg):
                if abs(deg - cfg.last_sent_deg) < self.deadband_deg:
                    continue
                if (now - cfg.last_sent_time) < min_dt:
                    continue

            self._send(cfg, deg)
            cfg.last_sent_deg = deg
            cfg.last_sent_time = now

    def _send(self, cfg: JointCfg, deg: float) -> None:
        url = f'{self.base_url}/set'
        payload = {'channel': cfg.channel, 'value': int(round(deg)), 'mode': 'servo'}
        try:
            r = self._session.post(url, json=payload, timeout=self.timeout)
            if r.status_code != 200:
                self.get_logger().warn(
                    f'CH{cfg.channel} -> {deg:.1f}: HTTP {r.status_code} {r.text[:80]}')
        except requests.RequestException as e:
            self.get_logger().warn(f'CH{cfg.channel} POST failed: {e}')


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
