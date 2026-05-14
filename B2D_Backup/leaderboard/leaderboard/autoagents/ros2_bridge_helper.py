#!/usr/bin/env python3
"""
Runs under Python 3.10 (system ROS2). Subscribes to
/carla/hero/vehicle_control_cmd and forwards each message as
newline-delimited JSON over a local TCP socket to the Python 3.7
leaderboard process.

carla_ackermann_control (if used) converts AckermannDrive →
vehicle_control_cmd before this helper sees it, so this helper
always reads the final VehicleControl regardless of control mode.
"""

import json
import os
import socket as _socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from carla_msgs.msg import CarlaEgoVehicleControl

VEHICLE_TOPIC = "/carla/hero/vehicle_control_cmd"
PORT = int(os.environ.get("BRIDGE_PORT", "55055"))


class BridgeNode(Node):
    def __init__(self, conn):
        super().__init__("ros2_bridge_helper")
        self._conn = conn
        self._lock = threading.Lock()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            CarlaEgoVehicleControl, VEHICLE_TOPIC, self._callback, qos
        )
        self.get_logger().info(
            f"[bridge_helper] subscribed to {VEHICLE_TOPIC}, forwarding on :{PORT}"
        )

    def _callback(self, msg):
        data = {
            "header": {
                "stamp": {
                    "secs": msg.header.stamp.sec,
                    "nsecs": msg.header.stamp.nanosec,
                }
            },
            "throttle": float(msg.throttle),
            "steer": float(msg.steer),
            "brake": float(msg.brake),
            "hand_brake": bool(msg.hand_brake),
            "reverse": bool(msg.reverse),
            "manual_gear_shift": bool(msg.manual_gear_shift),
            "gear": int(msg.gear),
        }
        payload = (json.dumps(data) + "\n").encode()
        with self._lock:
            try:
                self._conn.sendall(payload)
            except OSError:
                self.get_logger().warn("[bridge_helper] socket send failed")


def main():
    srv = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
    srv.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(1)
    print(f"[bridge_helper] listening on 127.0.0.1:{PORT}", flush=True)
    conn, _ = srv.accept()
    print("[bridge_helper] agent connected", flush=True)

    rclpy.init()
    node = BridgeNode(conn)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        conn.close()
        srv.close()


if __name__ == "__main__":
    main()
