#!/bin/bash
# Run SimpleAgent end-to-end: eval → json → video
# Usage: bash leaderboard/scripts/run_simple_agent.sh
# Run from the Bench2Drive root directory with zhiyuan_b2d conda env active.

# ── Paths ────────────────────────────────────────────────────────────────────
export CARLA_ROOT=/home/dellpro2/CARLA_0.9.15
export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:leaderboard
export PYTHONPATH=$PYTHONPATH:leaderboard/team_code
export PYTHONPATH=$PYTHONPATH:scenario_runner
export SCENARIO_RUNNER_ROOT=scenario_runner
export LEADERBOARD_ROOT=leaderboard

# ── Ports & GPU ──────────────────────────────────────────────────────────────
PORT=20000
TM_PORT=20500
GPU_RANK=0           # change if GPU 0 is occupied

# ── Eval settings ────────────────────────────────────────────────────────────
ROUTES=leaderboard/data/routes_devtest.xml          # 2 routes, quick test
TEAM_AGENT=leaderboard/team_code/simple_agent.py
TEAM_CONFIG=none                                     # no model checkpoint needed
CHECKPOINT_ENDPOINT=simple_agent_eval.json
export SAVE_PATH=./simple_agent_output/             # images + meta saved here
export IS_BENCH2DRIVE=True
export PLANNER_TYPE=only_traj

export CHALLENGE_TRACK_CODENAME=SENSORS
export DEBUG_CHALLENGE=0
export REPETITIONS=1
export RESUME=True

# ── Step 1: Run evaluation ────────────────────────────────────────────────────
echo "====== Step 1: Running evaluation ======"
CUDA_VISIBLE_DEVICES=${GPU_RANK} python ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
    --routes=${ROUTES} \
    --repetitions=${REPETITIONS} \
    --track=${CHALLENGE_TRACK_CODENAME} \
    --checkpoint=${CHECKPOINT_ENDPOINT} \
    --agent=${TEAM_AGENT} \
    --agent-config=${TEAM_CONFIG} \
    --debug=${DEBUG_CHALLENGE} \
    --resume=${RESUME} \
    --port=${PORT} \
    --traffic-manager-port=${TM_PORT} \
    --gpu-rank=${GPU_RANK}

# ── Step 2: Compute metrics ──────────────────────────────────────────────────
echo ""
echo "====== Step 2: Merging result JSON ======"
python tools/merge_route_json.py -f ./

echo ""
echo "====== Step 3: Visualize (generate videos) ======"
for route_dir in ${SAVE_PATH}*/; do
    if [ -d "${route_dir}/rgb_front" ]; then
        out_video="${route_dir}output.mp4"
        echo "  Generating: ${out_video}"
        python - <<PYEOF
import cv2, os, json
from tqdm import trange

images_folder = "${route_dir}"
output_video  = "${out_video}"
fps = 15

images = sorted([f for f in os.listdir(os.path.join(images_folder, 'rgb_front'))
                 if f.endswith('.jpg') or f.endswith('.png')])
if len(images) < 2:
    print("Not enough frames, skipping.")
    exit()

frame0 = cv2.imread(os.path.join(images_folder, 'rgb_front', images[0]))
h, w = frame0.shape[:2]
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video  = cv2.VideoWriter(output_video, fourcc, fps, (w, h))

for i in trange(1, len(images)):
    img_path  = os.path.join(images_folder, 'rgb_front', images[i])
    meta_path = os.path.join(images_folder, 'meta', f'{i:04d}.json')
    img = cv2.imread(img_path)
    try:
        with open(meta_path) as f:
            meta = json.load(f)
        text = (f"speed:{meta['speed']:.2f}  steer:{meta['steer']:.2f}  "
                f"throttle:{meta['throttle']:.2f}  brake:{meta['brake']:.2f}")
        cv2.putText(img, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 255, 255), 2, cv2.LINE_AA)
    except FileNotFoundError:
        pass
    video.write(img)
video.release()
print(f"Saved: {output_video}")
PYEOF
    fi
done

echo ""
echo "====== Done! ======"
echo "  Eval JSON  : ${CHECKPOINT_ENDPOINT}"
echo "  Images/meta: ${SAVE_PATH}"
echo "  Videos      : ${SAVE_PATH}*/output.mp4"
