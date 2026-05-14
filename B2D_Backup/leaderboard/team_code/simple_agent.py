#!/usr/bin/env python
"""
SimpleAgent: outputs fixed throttle/steer/brake for quick pipeline testing.
Saves rgb_front images and meta JSON for generate_video.py visualization.
"""

import os
import json
import cv2
import carla

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track


def get_entry_point():
    return 'SimpleAgent'


class SimpleAgent(AutonomousAgent):

    def setup(self, path_to_conf_file):
        self.track = Track.SENSORS
        self._step = 0
        self._save_path = None

        save_path_base = os.environ.get('SAVE_PATH', None)
        if save_path_base and '+' in path_to_conf_file:
            save_name = path_to_conf_file.split('+')[-1]
            self._save_path = os.path.join(save_path_base, save_name)
            os.makedirs(os.path.join(self._save_path, 'rgb_front'), exist_ok=True)
            os.makedirs(os.path.join(self._save_path, 'meta'), exist_ok=True)
            print(f'[SimpleAgent] Saving to: {self._save_path}')

    def sensors(self):
        return [
            {
                'type': 'sensor.camera.rgb',
                'x': 1.3, 'y': 0.0, 'z': 2.3,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'width': 800, 'height': 450, 'fov': 100,
                'id': 'CAM_FRONT',
            },
            {
                'type': 'sensor.speedometer',
                'reading_frequency': 20,
                'id': 'SPEED',
            },
        ]

    def run_step(self, input_data, timestamp):
        # ── Control ──────────────────────────────────────────────
        # Replace these values with your own model output later.
        control = carla.VehicleControl()
        control.throttle = 0.5
        control.steer = 0.0
        control.brake = 0.0
        control.hand_brake = False

        # ── Save data ────────────────────────────────────────────
        if self._save_path:
            # RGB image: CARLA returns BGRA, drop alpha
            rgb = input_data['CAM_FRONT'][1][:, :, :3]
            img_path = os.path.join(
                self._save_path, 'rgb_front', f'{self._step:04d}.jpg'
            )
            cv2.imwrite(img_path, rgb)

            # Meta JSON (format expected by tools/generate_video.py)
            speed = input_data['SPEED'][1]['speed']
            meta = {
                'steer': control.steer,
                'throttle': control.throttle,
                'brake': control.brake,
                'speed': speed,
            }
            meta_path = os.path.join(
                self._save_path, 'meta', f'{self._step:04d}.json'
            )
            with open(meta_path, 'w') as f:
                json.dump(meta, f)

        self._step += 1
        return control
