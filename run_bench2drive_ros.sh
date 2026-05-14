#!/bin/bash
# Launch script for CARLA PC: Bench2Drive leaderboard + custom ROS2 agent
# The agent internally starts carla_ros_bridge + rosbridge_websocket on port 9090
# The inference machine (Orin) connects via ws://<CARLA_PC_IP>:9090
#
# Control mode (set via env var before running):
#   Default (vehicle):  ./run_bench2drive_ros.sh
#   Ackermann mode:     CONTROL_MODE=ackermann ./run_bench2drive_ros.sh
#
#   vehicle mode:  Orin publishes /carla/hero/vehicle_control_cmd (CarlaEgoVehicleControl)
#                  Fields: throttle(0-1), brake(0-1), steer(-1~1)
#   ackermann mode: Orin publishes /carla/hero/ackermann_control (AckermannDrive)
#                  Fields: speed(m/s), steering_angle(rad)
#                  carla_ackermann_control node must be started separately

set -e

# ---------- Paths ----------
CARLA_ROOT=/home/avsaw1/yuheng/simulator/CARLA_0.9.15

LEADERBOARD_ROOT=/home/avsaw1/Karmishth/Masterarbeit/Bench2Drive/leaderboard
SCENARIO_RUNNER_ROOT=/home/avsaw1/Karmishth/Masterarbeit/Bench2Drive/scenario_runner
MY_AGENT=/home/avsaw1/Karmishth/Masterarbeit/my_ros2_agent.py
PYTHON=/home/avsaw1/miniconda3/envs/bench2drive310/bin/python3

# ---------- ROS2 environment ----------
# Must be sourced before launch; leaderboard internally spawns ros2 launch / carla_ros_bridge
source /opt/ros/humble/setup.bash
source /home/avsaw1/Karmishth/Masterarbeit/Zhiyuan_ws/install/setup.bash
# ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
# Ensure system Python 3.10 takes precedence over conda base Python 3.12
# for all ros2 launch subprocesses (rosbridge, carla_ros_bridge, etc.)
export PATH=/usr/bin:$PATH

# ---------- Python path ----------
export CARLA_ROOT
CONDA_SITE=/home/avsaw1/miniconda3/envs/bench2drive310/lib/python3.10/site-packages
export PYTHONPATH=${LEADERBOARD_ROOT}:${SCENARIO_RUNNER_ROOT}:${CARLA_ROOT}/PythonAPI/carla:${CONDA_SITE}:${PYTHONPATH}

# Use conda's OpenSSL instead of system libcrypto (system version lacks OPENSSL_3.3.0)
export LD_LIBRARY_PATH=/home/avsaw1/miniconda3/envs/bench2drive310/lib:${LD_LIBRARY_PATH}

# ---------- rosbridge port (Orin connects to this) ----------
export ROSBRIDGE_PORT=9090

# ---------- Launch ----------
${PYTHON} ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
    --routes ${LEADERBOARD_ROOT}/data/small_town_2routes.xml \
    --repetitions 1 \
    --track SENSORS \
    --checkpoint /home/avsaw1/Karmishth/Masterarbeit/results.json \
    --agent ${MY_AGENT} \
    --agent-config "" \
    --port 2000 \
    --traffic-manager-port 8000 \
    --gpu-rank 0
