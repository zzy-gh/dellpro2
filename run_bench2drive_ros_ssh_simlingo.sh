#!/bin/bash
# Launch script for CARLA PC: Bench2Drive leaderboard + SimLingo ROS2 agent
# SSH-compatible version: sets up display and graphics environment for headless SSH sessions
#
# The agent internally starts carla_ros_bridge + rosbridge_websocket on port 9090
# The inference machine (Orin) connects via ws://<CARLA_PC_IP>:9090
#
# Control mode (set via env var before running):
#   Default (vehicle):  ./run_bench2drive_ros_ssh_simlingo.sh
#
#   vehicle mode:  Orin publishes /carla/hero/vehicle_control_cmd (CarlaEgoVehicleControl)
#                  Fields: throttle(0-1), brake(0-1), steer(-1~1)

set -e

# ---------- SSH / Display fix ----------
# SSH sessions don't inherit the desktop graphics environment.
# These settings allow CARLA to find and use the local X display and GPU.
export DISPLAY=:1
unset SDL_VIDEODRIVER
xhost +local: > /dev/null 2>&1 || true

# Restore LD_LIBRARY_PATH entries that are present in local desktop sessions
# but missing in SSH sessions (needed for CARLA/UE4 to find graphics libraries)
export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/usr/local/cuda-12.8/lib64:/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH}

# ---------- Paths ----------
CARLA_ROOT=/home/dellpro2/Zhiyuan/carla_0.9.15

LEADERBOARD_ROOT=/home/dellpro2/Karmishth/Bench2Drive/leaderboard
SCENARIO_RUNNER_ROOT=/home/dellpro2/Karmishth/Bench2Drive/scenario_runner
MY_AGENT=/home/dellpro2/Karmishth/my_ros2_agent_simlingo.py
PYTHON=/home/dellpro2/miniconda3/envs/zhiyuan_b2d/bin/python3

# ---------- ROS2 environment ----------
# Must be sourced before launch; leaderboard internally spawns ros2 launch / carla_ros_bridge
source /opt/ros/humble/setup.bash
source /home/dellpro2/Zhiyuan_ws/install/setup.bash

# Ensure system Python 3.10 takes precedence over conda base Python 3.12
# for all ros2 launch subprocesses (rosbridge, carla_ros_bridge, etc.)
export PATH=/usr/bin:$PATH

# ---------- Python path ----------
export CARLA_ROOT
CONDA_SITE=/home/dellpro2/miniconda3/envs/zhiyuan_b2d/lib/python3.10/site-packages
export PYTHONPATH=${LEADERBOARD_ROOT}:${SCENARIO_RUNNER_ROOT}:${CARLA_ROOT}/PythonAPI/carla:${CONDA_SITE}:${PYTHONPATH}

# Use conda's OpenSSL instead of system libcrypto (system version lacks OPENSSL_3.3.0)
export LD_LIBRARY_PATH=/home/dellpro2/miniconda3/envs/zhiyuan_b2d/lib:${LD_LIBRARY_PATH}

# ---------- rosbridge port (Orin connects to this) ----------
export ROSBRIDGE_PORT=9090

# ---------- Launch ----------
${PYTHON} ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
    --routes ${LEADERBOARD_ROOT}/data/small_town_2routes.xml \
    --repetitions 1 \
    --track SENSORS \
    --checkpoint /home/dellpro2/Karmishth/results.json \
    --agent ${MY_AGENT} \
    --agent-config "" \
    --port 2000 \
    --traffic-manager-port 8000 \
    --gpu-rank 0
