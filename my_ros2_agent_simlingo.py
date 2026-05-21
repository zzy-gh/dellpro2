"""
CARLA leaderboard agent (ROS gateway) for SimLingo inference.

Sensor config matches simlingo/team_code/agent_simlingo.py + config_simlingo.GlobalConfig:
  - 1x front RGB camera (rgb_0): pos=(-1.5, 0, 2.0), rot=(0,0,0), 1024x512, fov=110
  - sensor.other.imu  id=imu
  - sensor.other.gnss id=gps
  - sensor.speedometer id=speed  (leaderboard pseudo-sensor; ROSAgentWrapper auto-remaps to
                                  sensor.pseudo.speedometer so carla_ros_bridge publishes
                                  /carla/hero/speed as std_msgs/Float32)
  - sensor.pseudo.odom id=odometry

Topics published by carla_ros_bridge (role_name = hero):
  /carla/hero/rgb_0/image          sensor_msgs/Image      (RGBA, 1024x512)
  /carla/hero/rgb_0/camera_info    sensor_msgs/CameraInfo
  /carla/hero/imu                  sensor_msgs/Imu
  /carla/hero/gps                  sensor_msgs/NavSatFix
  /carla/hero/speed                std_msgs/Float32
  /carla/hero/odometry             nav_msgs/Odometry
  /carla/hero/global_plan          carla_msgs/CarlaRoute         (TRANSIENT_LOCAL)
  /carla/hero/global_plan_gnss     carla_msgs/CarlaGnssRoute     (TRANSIENT_LOCAL)
  /carla/hero/status               std_msgs/Bool

Control (CONTROL_MODE=ackermann, default):
  Inference machine publishes: /carla/hero/ackermann_control  ackermann_msgs/AckermannDrive
  carla_ackermann_control node converts → /carla/hero/vehicle_control_cmd
  This agent reads:           /carla/hero/vehicle_control_cmd  carla_msgs/CarlaEgoVehicleControl

Control (CONTROL_MODE=vehicle):
  Inference machine publishes: /carla/hero/vehicle_control_cmd  carla_msgs/CarlaEgoVehicleControl
"""

import os

from leaderboard.autoagents.ros2_agent import ROS2Agent


CARLA_FPS = 20
CAMERA_TICK = 1.0 / CARLA_FPS

# Camera config matches SimLingo training (config_simlingo.GlobalConfig):
#   camera_pos_0 = [-1.5, 0.0, 2.0]  (1.5 m behind centre, 2 m high)
#   camera_fov_0 = 110
CAMERA_POS = [-1.5, 0.0, 2.0]
CAMERA_ROT = [0.0, 0.0, 0.0]
CAMERA_W   = 1024
CAMERA_H   = 512
CAMERA_FOV = 110

CONTROL_MODE = os.environ.get("CONTROL_MODE", "ackermann")  # "vehicle" or "ackermann"


def get_entry_point():
    return "MyROS2AgentSimLingo"


class MyROS2AgentSimLingo(ROS2Agent):

    def setup(self, path_to_conf_file):
        self._ackermann_proc = None
        print(
            f"[MyROS2AgentSimLingo] CONTROL_MODE={CONTROL_MODE}. "
            "Expecting carla_ackermann_control node to be running externally (ackermann mode)."
        )

    def sensors(self):
        return [
            # Single front camera — matches SimLingo training configuration exactly
            {
                "type":   "sensor.camera.rgb",
                "id":     "rgb_0",
                "x":      CAMERA_POS[0],
                "y":      CAMERA_POS[1],
                "z":      CAMERA_POS[2],
                "roll":   CAMERA_ROT[0],
                "pitch":  CAMERA_ROT[1],
                "yaw":    CAMERA_ROT[2],
                "width":  CAMERA_W,
                "height": CAMERA_H,
                "fov":    CAMERA_FOV,
            },
            {
                "type": "sensor.other.imu",
                "id":   "imu",
                "x": 0.0, "y": 0.0, "z": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "sensor_tick": CAMERA_TICK,
            },
            {
                "type": "sensor.other.gnss",
                "id":   "gps",
                "x": 0.0, "y": 0.0, "z": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "sensor_tick": 0.01,
            },
            {
                "type": "sensor.speedometer",
                "id":   "speed",
                "reading_frequency": CARLA_FPS,
            },
            {
                "type": "sensor.pseudo.odom",
                "id":   "odometry",
                "x": 0.0, "y": 0.0, "z": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            },
        ]

    def get_ros_entrypoint(self):
        return {
            "package":    "rosbridge_server",
            "launch_file": "rosbridge_websocket_launch.xml",
            "parameters": {},
        }

    def destroy(self):
        if self._ackermann_proc is not None:
            self._ackermann_proc.terminate()
            self._ackermann_proc.wait()
        super().destroy()
