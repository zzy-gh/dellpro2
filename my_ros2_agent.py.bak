"""
A machine leaderboard agent (ROS gateway) compatible with simlingo I/O.

Sensor list mirrors simlingo/team_code/agent_simlingo.py + config_simlingo.GlobalConfig:
  - 1x front RGB camera (rgb_0): pos=(-1.5, 0, 2.0), rot=(0,0,0), 1024x512, fov=110
  - sensor.other.imu  id=imu
  - sensor.other.gnss id=gps
  - sensor.speedometer id=speed   (leaderboard pseudo-sensor; ROSAgentWrapper auto-remaps to
                                   sensor.pseudo.speedometer so carla_ros_bridge publishes
                                   /carla/hero/speed as std_msgs/Float32)

Topics published by carla_ros_bridge (ego role_name = hero):
  /carla/hero/rgb_0/image          sensor_msgs/Image
  /carla/hero/rgb_0/camera_info    sensor_msgs/CameraInfo
  /carla/hero/imu                  sensor_msgs/Imu
  /carla/hero/gps                  sensor_msgs/NavSatFix
  /carla/hero/speed                std_msgs/Float32
  /carla/hero/odometry             nav_msgs/Odometry     (auto-published; used by carla_ackermann_control PID)
  /carla/hero/global_plan          carla_msgs/CarlaRoute
  /carla/hero/global_plan_gnss     carla_msgs/CarlaGnssRoute
  /carla/hero/status               std_msgs/Bool

Control mode (set via CONTROL_MODE env var):

  CONTROL_MODE=vehicle  (default)
    B machine publishes: /carla/hero/vehicle_control_cmd  carla_msgs/CarlaEgoVehicleControl
    Fields: throttle(0-1), brake(0-1), steer(-1~1), hand_brake, reverse

  CONTROL_MODE=ackermann
    B machine publishes: /carla/hero/ackermann_control  ackermann_msgs/AckermannDrive
    Fields: speed(m/s), steering_angle(rad)
    carla_ackermann_control node is auto-started to do PID conversion to vehicle_control_cmd
"""

import os

from leaderboard.autoagents.ros2_agent import ROS2Agent


CARLA_FPS = 20
CAMERA_TICK = 1.0 / CARLA_FPS

CAMERA_POS  = [1.5, 0.0, 2.0]
CAMERA_ROT  = [0.0, 0.0, 0.0]
CAMERA_W    = 1024
CAMERA_H    = 512
CAMERA_FOV  = 110

CONTROL_MODE = os.environ.get("CONTROL_MODE", "ackermann")  # "vehicle" or "ackermann"


def get_entry_point():
    return "MyROS2Agent"


class MyROS2Agent(ROS2Agent):

    def setup(self, path_to_conf_file):
        self._ackermann_proc = None
        print("[MyROS2Agent] expecting ackermann_control node to be running externally")

    def sensors(self):
        return [
            {
                "type": "sensor.camera.rgb",
                "id": "rgb_0",
                "x": CAMERA_POS[0], "y": CAMERA_POS[1], "z": CAMERA_POS[2],
                "roll":  CAMERA_ROT[0], "pitch": CAMERA_ROT[1], "yaw": CAMERA_ROT[2],
                "width":  CAMERA_W, "height": CAMERA_H, "fov": CAMERA_FOV,
            },
            {
                "type": "sensor.other.imu",
                "id": "imu",
                "x": 0.0, "y": 0.0, "z": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "sensor_tick": CAMERA_TICK,
            },
            {
                "type": "sensor.other.gnss",
                "id": "gps",
                "x": 0.0, "y": 0.0, "z": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "sensor_tick": 0.01,
            },
            {
                "type": "sensor.speedometer",
                "id": "speed",
                "reading_frequency": CARLA_FPS,
            },
            {
                "type": "sensor.pseudo.odom",
                "id": "odometry",
                "x": 0.0, "y": 0.0, "z": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            },
        ]

    def get_ros_entrypoint(self):
        return {
            "package": "rosbridge_server",
            "launch_file": "rosbridge_websocket_launch.xml",
            "parameters": {},
        }

    def destroy(self):
        if self._ackermann_proc is not None:
            self._ackermann_proc.terminate()
            self._ackermann_proc.wait()
        super().destroy()
