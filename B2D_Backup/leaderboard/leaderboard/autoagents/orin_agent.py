#!/usr/bin/env python3
"""
orin_agent.py — Bench2Drive leaderboard agent for Orin-in-the-loop evaluation.

Startup sequence (handled here, in __init__ / setup):
  1. Launch carla_ros_bridge (passive, synchronous) — publishes CARLA sensor
     data as ROS2 topics so the Orin can subscribe to them.
  2. Subscribe to /carla/hero/vehicle_control_cmd where the Orin publishes
     CarlaEgoVehicleControl after running its AI model.
  3. run_step() returns the received control to the leaderboard evaluator,
     which applies it to the CARLA ego vehicle directly via Python API.

The Orin never touches CARLA directly — it only speaks ROS2.
"""

import queue
import threading

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from carla_msgs.msg import CarlaEgoVehicleControl

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from leaderboard.autoagents.ros_base_agent import ROSLauncher

CONTROL_TOPIC = "/carla/hero/vehicle_control_cmd"
CONTROL_TIMEOUT_S = 1.0


def get_entry_point():
    return 'OrinAgent'


class OrinAgent(AutonomousAgent):
    """
    Leaderboard agent that:
      - launches carla_ros_bridge so the Orin sees sensor topics
      - receives CarlaEgoVehicleControl from the Orin
      - returns that control to CARLA each step
    """

    def __init__(self, carla_host, carla_port, debug=False):
        super().__init__(carla_host, carla_port, debug)

        # --- 1. Launch carla_ros_bridge (passive, no ego registration) ---
        self._bridge = ROSLauncher("bridge", ros_version=2, debug=debug)
        self._bridge.run(
            package="carla_ros_bridge",
            launch_file="carla_ros_bridge.launch.py",
            parameters={
                "host": carla_host,
                "port": carla_port,
                "timeout": 60,
                "synchronous_mode": True,
                "passive": True,
                "register_all_sensors": True,    # publish all spawned sensors
                "ego_vehicle_role_name": "hero",
            },
        )

        # --- 2. ROS2 node: subscribe to control from Orin ---
        rclpy.init(args=None)
        self._node = rclpy.create_node('orin_agent_node')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._node.create_subscription(
            CarlaEgoVehicleControl,
            CONTROL_TOPIC,
            self._control_callback,
            qos,
        )

        self._control_queue = queue.Queue(maxsize=1)
        self._last_control = carla.VehicleControl()  # default: stopped
        self._received_any = False

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._spin_thread.start()

        self._node.get_logger().info(
            f'OrinAgent ready — carla_ros_bridge launched, '
            f'waiting for control on {CONTROL_TOPIC}'
        )

    def _control_callback(self, msg: CarlaEgoVehicleControl):
        control = carla.VehicleControl(
            throttle=float(msg.throttle),
            steer=float(msg.steer),
            brake=float(msg.brake),
            hand_brake=bool(msg.hand_brake),
            reverse=bool(msg.reverse),
            manual_gear_shift=bool(msg.manual_gear_shift),
            gear=int(msg.gear),
        )
        # Always keep the freshest command — drop stale if queue is full
        try:
            self._control_queue.get_nowait()
        except queue.Empty:
            pass
        self._control_queue.put_nowait(control)
        self._received_any = True

    def setup(self, path_to_conf_file):
        self.track = Track.SENSORS

    def sensors(self):
        """
        Sensors spawned on the ego vehicle in CARLA.
        carla_ros_bridge publishes these as ROS2 topics; the Orin subscribes.
          /carla/hero/GPS/fix    → sensor_msgs/NavSatFix
          /carla/hero/IMU/imu   → sensor_msgs/Imu
        """
        return [
            {
                'type': 'sensor.other.gnss',
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'id': 'GPS',
            },
            {
                'type': 'sensor.other.imu',
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'id': 'IMU',
            },
        ]

    def run_step(self, input_data, timestamp):
        assert self._bridge.is_alive(), "carla_ros_bridge process died"

        try:
            control = self._control_queue.get(block=True, timeout=CONTROL_TIMEOUT_S)
            self._last_control = control
        except queue.Empty:
            if not self._received_any:
                self._node.get_logger().warn(
                    f'No control from Orin on {CONTROL_TOPIC} — holding brake',
                    throttle_duration_sec=5.0,
                )
            control = self._last_control

        return control

    def destroy(self):
        self._bridge.terminate()
        self._node.destroy_node()
        rclpy.shutdown()
        self._spin_thread.join(timeout=2.0)
        super().destroy()
