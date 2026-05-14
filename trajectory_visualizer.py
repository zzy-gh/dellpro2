import carla
import rclpy
from rclpy.node import Node
from autoware_planning_msgs.msg import Trajectory

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        self.debug = carla.Client('localhost', 2000).get_world().debug
        self.create_subscription(Trajectory, '/alpamayo/predicted_trajectory', self.callback, 10)

    def callback(self, msg):
        for point in msg.points:
            self.debug.draw_point(
                carla.Location(x=point.pose.position.x, y=point.pose.position.y, z=point.pose.position.z + 0.5),
                size=0.15, color=carla.Color(r=0, g=255, b=0), life_time=0.5)

rclpy.init()
rclpy.spin(TrajectoryVisualizer())
