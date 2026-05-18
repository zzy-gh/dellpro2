#!/usr/bin/env python3
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Trajectory GIF visualizer for CARLA + ROS 2.

Subscribes to:
  /carla/hero/global_plan            (carla_msgs/CarlaRoute)
  /carla/hero/vehicle_control_cmd    (carla_msgs/CarlaEgoVehicleControl)
  /carla/hero/odometry               (nav_msgs/Odometry)

Renders a two-panel figure each tick:
  Left : top-down map with the planned route (colour-coded by RoadOption)
         and the ego trajectory history
  Right: command HUD — steering wheel needle, throttle/brake bars, gear, speed

On Ctrl-C or SIGTERM the accumulated frames are written as an animated GIF.

ROS 2 parameters
----------------
output  str  (default 'trajectory_scenario.gif')  output file path
fps     int  (default 4)                           capture + GIF playback rate
history int  (default 1000)                        max ego positions kept in memory

Quick start
-----------
  python3 tools/trajectory_gif_visualizer.py
  ros2 run leaderboard trajectory_gif_visualizer \
      --ros-args -p output:=run1.gif -p fps:=4

Dependencies (pip install if missing)
--------------------------------------
  imageio   pillow   matplotlib   rclpy   carla_msgs   nav_msgs
"""

import io
import signal
import sys
from threading import Lock

import imageio.v2 as imageio
import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaRoute
from nav_msgs.msg import Odometry


# RoadOption int codes → (colour, label)
# Mirrors srunner.scenariomanager.carla_data_provider.RoadOption
_ROAD_OPT_STYLE = {
    -1: ('#888888', 'Void'),
     1: ('#4fc3f7', 'Left'),
     2: ('#ffb74d', 'Right'),
     3: ('#aed581', 'Straight'),
     4: ('#80cbc4', 'Follow lane'),
     5: ('#ce93d8', 'Change left'),
     6: ('#fff176', 'Change right'),
}
_DEFAULT_WP_COLOR = '#80cbc4'

# Dark-theme palette
_BG   = '#0d0d1a'
_PANEL = '#111122'
_GRID  = '#1e1e33'


class TrajectoryGifVisualizer(Node):
    """ROS 2 node that captures frames and saves them as an animated GIF."""

    def __init__(self) -> None:
        super().__init__('trajectory_gif_visualizer')

        self.declare_parameter('output', 'trajectory_scenario.gif')
        self.declare_parameter('fps', 4)
        self.declare_parameter('history', 1000)

        self._output      = self.get_parameter('output').get_parameter_value().string_value
        self._fps         = self.get_parameter('fps').get_parameter_value().integer_value
        self._max_history = self.get_parameter('history').get_parameter_value().integer_value

        self._lock             = Lock()
        self._plan_poses: list  = []   # [(x, y), ...]  ROS frame
        self._plan_opts:  list  = []   # [int(RoadOption), ...]
        self._ego_history: list = []   # [(x, y, yaw), ...]
        self._ctrl:  dict | None = None
        self._speed_mps: float  = 0.0
        self._frames:    list   = []

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        default = QoSProfile(depth=10)

        self.create_subscription(CarlaRoute,
                                 '/carla/hero/global_plan',
                                 self._plan_cb, latched)
        self.create_subscription(CarlaEgoVehicleControl,
                                 '/carla/hero/vehicle_control_cmd',
                                 self._ctrl_cb, default)
        self.create_subscription(Odometry,
                                 '/carla/hero/odometry',
                                 self._odom_cb, default)

        self.create_timer(1.0 / max(1, self._fps), self._capture_frame)
        self.get_logger().info(
            f'Visualizer ready — output={self._output}  fps={self._fps}')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _plan_cb(self, msg: CarlaRoute) -> None:
        poses = [(p.position.x, p.position.y) for p in msg.poses]
        opts  = list(msg.road_options)
        with self._lock:
            self._plan_poses = poses
            self._plan_opts  = opts
        self.get_logger().info(f'Global plan received: {len(poses)} waypoints')

    def _ctrl_cb(self, msg: CarlaEgoVehicleControl) -> None:
        with self._lock:
            self._ctrl = {
                'throttle':   float(msg.throttle),
                'steer':      float(msg.steer),
                'brake':      float(msg.brake),
                'reverse':    bool(msg.reverse),
                'hand_brake': bool(msg.hand_brake),
            }

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = 2.0 * np.arctan2(ori.z, ori.w)          # quaternion → yaw
        spd = np.hypot(msg.twist.twist.linear.x,
                       msg.twist.twist.linear.y)
        with self._lock:
            self._ego_history.append((pos.x, pos.y, yaw))
            if len(self._ego_history) > self._max_history:
                self._ego_history = self._ego_history[-self._max_history:]
            self._speed_mps = spd

    # ── Frame rendering ────────────────────────────────────────────────────────

    def _capture_frame(self) -> None:
        with self._lock:
            plan_poses = list(self._plan_poses)
            plan_opts  = list(self._plan_opts)
            history    = list(self._ego_history)
            ctrl       = dict(self._ctrl) if self._ctrl else None
            speed      = self._speed_mps

        if not plan_poses and not history:
            return

        fig = plt.figure(figsize=(16, 8), facecolor=_BG)
        gs  = fig.add_gridspec(1, 2, width_ratios=[3, 1], wspace=0.02)
        ax_map = fig.add_subplot(gs[0])
        ax_hud = fig.add_subplot(gs[1])

        self._render_map(ax_map, plan_poses, plan_opts, history)
        self._render_hud(ax_hud, ctrl, speed)

        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=100,
                    bbox_inches='tight', facecolor=fig.get_facecolor())
        buf.seek(0)
        self._frames.append(imageio.imread(buf))
        plt.close(fig)

    # ── Map panel ──────────────────────────────────────────────────────────────

    def _render_map(self, ax, plan_poses, plan_opts, history) -> None:
        ax.set_facecolor(_PANEL)

        # Thin connector line for the full route
        if plan_poses:
            xs, ys = zip(*plan_poses)
            ax.plot(xs, ys, '-', color='#2a2a4a', linewidth=1.2, zorder=1)

        # Waypoints coloured by RoadOption
        seen_opts: set = set()
        for i, (x, y) in enumerate(plan_poses):
            opt   = plan_opts[i] if i < len(plan_opts) else 4
            color = _ROAD_OPT_STYLE.get(opt, (_DEFAULT_WP_COLOR, ''))[0]
            ax.plot(x, y, 'o', color=color, markersize=4, alpha=0.7, zorder=2)
            seen_opts.add(opt)

        # Ego trajectory history — fades from dim to bright
        n = len(history)
        if n > 1:
            hx = [p[0] for p in history]
            hy = [p[1] for p in history]
            for i in range(1, n):
                alpha = 0.15 + 0.85 * (i / n)
                ax.plot(hx[i-1:i+1], hy[i-1:i+1],
                        '-', color='#ff4444', linewidth=2.0,
                        alpha=alpha, zorder=3)

        # Current position + heading arrow
        if history:
            ex, ey, eyaw = history[-1]
            half = self._half_span(plan_poses, history)
            arrow_len = max(3.0, half * 0.06)
            dx, dy = arrow_len * np.cos(eyaw), arrow_len * np.sin(eyaw)
            ax.annotate('',
                        xy=(ex + dx, ey + dy), xytext=(ex, ey),
                        arrowprops=dict(arrowstyle='->', color='#ffd700',
                                        lw=2.5, mutation_scale=18),
                        zorder=6)
            ax.plot(ex, ey, 's', color='#ffd700', markersize=11,
                    markeredgecolor='white', markeredgewidth=1, zorder=6)

        # Viewport
        cx, cy, half = self._viewport(plan_poses, history)
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)

        # RoadOption legend (only options that appear in this plan)
        handles = [
            mpatches.Patch(color=_ROAD_OPT_STYLE[o][0],
                           label=_ROAD_OPT_STYLE[o][1])
            for o in sorted(seen_opts)
            if o in _ROAD_OPT_STYLE
        ]
        if handles:
            ax.legend(handles=handles, loc='upper left',
                      facecolor='#1a1a33', edgecolor='#555',
                      labelcolor='white', fontsize=7, framealpha=0.85)

        ax.set_title('CARLA Scenario — Trajectory View',
                     color='white', fontsize=13, pad=8)
        ax.set_xlabel('X (m)', color='#aaa', fontsize=9)
        ax.set_ylabel('Y (m)', color='#aaa', fontsize=9)
        ax.tick_params(colors='#666')
        for sp in ax.spines.values():
            sp.set_edgecolor('#333')
        ax.grid(True, color=_GRID, linewidth=0.6)

    # ── HUD panel ──────────────────────────────────────────────────────────────

    def _render_hud(self, ax, ctrl, speed) -> None:
        ax.set_facecolor(_PANEL)
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title('Vehicle Commands', color='white', fontsize=11, pad=8)
        for sp in ax.spines.values():
            sp.set_edgecolor('#333')

        if ctrl is None:
            ax.text(0.5, 0.5, 'Waiting for\ncommands…',
                    ha='center', va='center', color='#555', fontsize=11)
            return

        throttle   = ctrl['throttle']
        steer      = ctrl['steer']
        brake      = ctrl['brake']
        reverse    = ctrl['reverse']
        hand_brake = ctrl['hand_brake']

        # Steering wheel arc
        theta = np.linspace(np.deg2rad(200), np.deg2rad(340), 120)
        ax.plot(0.5 + 0.28 * np.cos(theta),
                0.80 + 0.18 * np.sin(theta),
                color='#444', lw=3, solid_capstyle='round')
        # Needle: steer 0 → up, ±1 → ±70°
        needle_angle = np.deg2rad(90 + steer * 70)
        nx = 0.5 + 0.22 * np.cos(needle_angle)
        ny = 0.80 + 0.14 * np.sin(needle_angle)
        ax.plot([0.5, nx], [0.80, ny], color='#ffd700', lw=3,
                solid_capstyle='round', zorder=5)
        ax.plot(0.5, 0.80, 'o', color='#ffd700', markersize=6, zorder=6)
        ax.text(0.5, 0.615, f'Steer  {steer:+.3f}',
                ha='center', color='#ccc', fontsize=9)

        # Throttle bar
        bx, by, bw, bh = 0.07, 0.46, 0.39, 0.06
        ax.add_patch(mpatches.Rectangle((bx, by), bw, bh,
                                         linewidth=1, edgecolor='#333',
                                         facecolor='#1a2a1a'))
        ax.add_patch(mpatches.Rectangle((bx, by), bw * throttle, bh,
                                         facecolor='#00e676', linewidth=0))
        ax.text(bx + bw / 2, by + bh + 0.025,
                f'Throttle  {throttle:.2f}',
                ha='center', color='#aaa', fontsize=8)

        # Brake bar
        bx2 = 0.54
        ax.add_patch(mpatches.Rectangle((bx2, by), bw, bh,
                                         linewidth=1, edgecolor='#333',
                                         facecolor='#2a1a1a'))
        ax.add_patch(mpatches.Rectangle((bx2, by), bw * brake, bh,
                                         facecolor='#ff5252', linewidth=0))
        ax.text(bx2 + bw / 2, by + bh + 0.025,
                f'Brake  {brake:.2f}',
                ha='center', color='#aaa', fontsize=8)

        # Gear indicator
        gear_txt = 'R' if reverse else 'D'
        gear_col = '#ff6b6b' if reverse else '#69f0ae'
        ax.text(0.30, 0.275, gear_txt, ha='center', va='center',
                color=gear_col, fontsize=28, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#0f3460',
                          edgecolor=gear_col, linewidth=2))
        ax.text(0.30, 0.145, 'Gear', ha='center', color='#555', fontsize=8)

        # Handbrake indicator
        hb_col = '#ff5252' if hand_brake else '#2a2a3a'
        hb_ec  = '#ff5252' if hand_brake else '#444'
        ax.text(0.70, 0.275, 'P', ha='center', va='center',
                color=hb_col, fontsize=28, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#0f3460',
                          edgecolor=hb_ec, linewidth=2))
        ax.text(0.70, 0.145, 'Handbrake', ha='center',
                color='#555', fontsize=8)

        # Speed
        ax.text(0.5, 0.055, f'{speed * 3.6:.1f} km/h',
                ha='center', va='center',
                color='white', fontsize=14, fontweight='bold')

    # ── Viewport helpers ───────────────────────────────────────────────────────

    def _half_span(self, plan_poses, history) -> float:
        all_pts = list(plan_poses) + [(p[0], p[1]) for p in history]
        if len(all_pts) < 2:
            return 50.0
        xs = [p[0] for p in all_pts]
        ys = [p[1] for p in all_pts]
        return max(max(xs) - min(xs), max(ys) - min(ys), 30.0)

    def _viewport(self, plan_poses, history):
        """Return (cx, cy, half_span) for the map viewport."""
        if history:
            ex, ey, _ = history[-1]
            # Include a lookahead window of the plan ahead of the ego
            lookahead: list = []
            if plan_poses:
                dists = [(np.hypot(p[0] - ex, p[1] - ey), i)
                         for i, p in enumerate(plan_poses)]
                closest = min(dists, key=lambda d: d[0])[1]
                lookahead = plan_poses[closest: closest + 60]
            pts = [(ex, ey)] + lookahead + [
                (p[0], p[1]) for p in history[-30:]]
        elif plan_poses:
            pts = list(plan_poses)
        else:
            return 0.0, 0.0, 50.0

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
        span = max(max(xs) - min(xs), max(ys) - min(ys), 30.0)
        return cx, cy, span * 0.65

    # ── GIF export ─────────────────────────────────────────────────────────────

    def save_gif(self) -> None:
        if not self._frames:
            self.get_logger().warn('No frames captured — nothing to save.')
            return
        self.get_logger().info(
            f'Saving {len(self._frames)} frames → {self._output}')
        imageio.mimsave(self._output, self._frames,
                        fps=self._fps, loop=0)
        self.get_logger().info(f'GIF saved: {self._output}')


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = TrajectoryGifVisualizer()

    def _on_shutdown(sig, frame):
        node.save_gif()
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _on_shutdown)
    signal.signal(signal.SIGTERM, _on_shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_gif()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
