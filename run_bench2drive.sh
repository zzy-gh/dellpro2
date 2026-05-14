#!/bin/bash
# Bench2Drive 最简评测启动脚本
# 用 my_agent.py 控制 ego 车跑 dev10（10 条路线）

CARLA_ROOT=/home/avsaw1/yuheng/simulator/CARLA_0.9.15

LEADERBOARD_ROOT=/home/avsaw1/Karmishth/Masterarbeit/Bench2Drive/leaderboard
SCENARIO_RUNNER_ROOT=/home/avsaw1/Karmishth/Masterarbeit/Bench2Drive/scenario_runner
MY_AGENT=/home/avsaw1/Karmishth/Masterarbeit/my_agent.py
# PYTHON=/home/dellpro2/miniconda3/envs/zhiyuan_b2d/bin/python

export CARLA_ROOT
export PYTHONPATH=${LEADERBOARD_ROOT}:${SCENARIO_RUNNER_ROOT}:${CARLA_ROOT}/PythonAPI/carla

${PYTHON} ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
    --routes ${LEADERBOARD_ROOT}/data/small_town_2routes.xml \
    --repetitions 1 \
    --track SENSORS \
    --checkpoint /home/dellpro2/Zhiyuan/results.json \
    --agent ${MY_AGENT} \
    --agent-config "" \
    --port 2000 \
    --traffic-manager-port 8000 \
    --gpu-rank 0
