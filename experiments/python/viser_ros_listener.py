"""Viser viewer that listens on ROS2 topics for trajectories from viser_dynamic_obstacle_avoid.py
and publishes an interactive obstacle pose. Requires ROS2 (rclpy) and viser.

Run: python experiments/python/viser_ros_listener.py
The panda_marker URDF is read from $CRICKET_ROOT/resources/panda_marker (default: ../cricket); adjust if absent.
"""
import os
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
from viser_utils import setup_viser_with_robot

def add_constraint_plane(server, position=(0.29276255, -0.55347496, 0.00607783), wxyz=(0, 1, 0, 0)):
    print(position, wxyz)
    box_handle = server.scene.add_box(
        name="my_cuboid",
        dimensions=(10.02, 10.5, 0.06),  # (width, height, depth)
        position=position,
        wxyz = wxyz,
        color=(0, 255, 0),  # Red color
        opacity=0.5,
    )

def add_json_cuboids(server, filename, color=(12, 89, 178)):
    """
    Add cuboids from a JSON file. The JSON should be a list of objects, each with keys:
    'name', 'x', 'y', 'z', 'dx', 'dy', 'dz' and optional 'roll','pitch','yaw'.

    This function reads the JSON, iterates over entries, and adds a box for each item.
    Dimensions are used as provided in the JSON (assumed full extents).
    """
    with open(filename, "r") as f:
        data = json.load(f)

    for i, obj in enumerate(data):
        name = obj.get("name", f"cuboid_{i}")
        x = obj.get("x", 0.0)
        y = obj.get("y", 0.0)
        z = obj.get("z", 0.0)
        dx = obj.get("dx", 0.0)
        dy = obj.get("dy", 0.0)
        dz = obj.get("dz", 0.0)

        # Position and dimensions expected as (x, y, z) and (dx, dy, dz)
        server.scene.add_box(
            name=f"/{name}",
            dimensions=(dx, dy, dz),
            color=color,
            position=(x, y, z),
        )


class ViserWaypointListener(Node):
	def __init__(self) -> None:
		super().__init__("viser_waypoint_listener")

		self.declare_parameter("waypoint_topic", "planner_waypoint")
		self.declare_parameter("frame_pose_topic", "obstacle_pose_and_dim")
		self.declare_parameter("ee_trajectory_topic", "ee_trajectory")
		self.declare_parameter("frame_publish_rate_hz", 10.0)

		waypoint_topic = self.get_parameter("waypoint_topic").get_parameter_value().string_value
		frame_pose_topic = self.get_parameter("frame_pose_topic").get_parameter_value().string_value
		ee_trajectory_topic = self.get_parameter("ee_trajectory_topic").get_parameter_value().string_value
		frame_publish_rate_hz = self.get_parameter("frame_publish_rate_hz").get_parameter_value().double_value

		# robot_dir = Path(__file__).parents[1] / "resources" / "panda"
		robot_dir = Path(os.environ.get("CRICKET_ROOT", Path(__file__).parents[2] / "cricket")) / "resources" / "panda_marker"
		self.server, self.robot = setup_viser_with_robot(robot_dir, "panda_expo.urdf")

		self.robot_dof = 7
		self.current_waypoint = np.array(
			[-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294],
			dtype=np.float64,
		)
		self.robot.update_cfg(self.current_waypoint.tolist())
		
		# Track waypoint spheres for trajectory visualization
		self.waypoint_sphere_handles = []

		self.frame_pose_pub = self.create_publisher(Float32MultiArray, frame_pose_topic, 10)
		self.waypoint_sub = self.create_subscription(
			Float32MultiArray,
			waypoint_topic,
			self.on_waypoint,
			10,
		)
		self.ee_trajectory_sub = self.create_subscription(
			Float32MultiArray,
			ee_trajectory_topic,
			self.on_ee_trajectory,
			10,
		)

		# self.hardcoded_frame_pose = np.array(
		# 	[0.30, -0.20, 0.50, 0.0, 0.0, 0.0, 1.0],
		# 	dtype=np.float64,
		# )
		self.frame_pub_timer = self.create_timer(
			1.0 / frame_publish_rate_hz,
			self.publish_hardcoded_frame_pose,
		)

		# Create interactive controller and mesh for the sphere obstacle.
		self.sphere_handle = self.server.scene.add_transform_controls(
			"/obstacle", scale=0.2, position=(0.4, 0.3, 0.4)
		)
		self.server.scene.add_icosphere(
            name="/obstacle/mesh",
            radius=0.05,
            position=(0.0, 0.0, 0.0),
            color=(1.0, 0.0, 0.0),
        )

		self.timing_handle = self.server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)


		add_constraint_plane(self.server)  # Add a constraint plane to the scene
		add_json_cuboids(self.server, str(Path(__file__).parents[1] / "assets" / "environments" / "maze" / "cuboids.json"))  # Add cuboids from JSON file


		self.get_logger().info(
			f"Viser listener ready. Subscribed to '{waypoint_topic}', publishing frame pose on '{frame_pose_topic}'."
		)

	def on_waypoint(self, msg: Float32MultiArray) -> None:
		waypoint = np.array(msg.data, dtype=np.float64)
		if waypoint.size != self.robot_dof:
			self.get_logger().warn(
				f"Received waypoint with {waypoint.size} values, expected {self.robot_dof}. Ignoring."
			)
			return

		self.current_waypoint = waypoint
		self.robot.update_cfg(self.current_waypoint.tolist())

	def on_ee_trajectory(self, msg: Float32MultiArray) -> None:
		"""Receive EE trajectory and visualize as waypoint spheres."""
		# Remove existing waypoint spheres
		for handle in self.waypoint_sphere_handles:
			handle.remove()
		self.waypoint_sphere_handles.clear()
		
		# Parse EE positions (message contains flattened [x1, y1, z1, x2, y2, z2, ...])
		if len(msg.data) % 3 != 0:
			self.get_logger().warn(f"EE trajectory data length {len(msg.data)} is not divisible by 3")
			return
		
		num_waypoints = len(msg.data) // 3
		for i in range(num_waypoints):
			x = msg.data[3*i]
			y = msg.data[3*i + 1]
			z = msg.data[3*i + 2]
			
			# Add a small sphere at each waypoint position
			handle = self.server.scene.add_icosphere(
				name=f"/trajectory_waypoint_{i}",
				radius=0.0075,
				position=(x, y, z-0.18),
				color=(0.0, 1.0, 0.0),  # Green color for waypoints
			)
			self.waypoint_sphere_handles.append(handle)
		

	def publish_hardcoded_frame_pose(self) -> None:
		out = Float32MultiArray()
		wxyz=np.array(self.sphere_handle.wxyz)
		position=np.array(self.sphere_handle.position)
		out.data = [float(v) for v in np.concatenate((position, wxyz, np.array([0.05])))]
		self.frame_pose_pub.publish(out)




def main(args=None) -> None:
	rclpy.init(args=args)
	node = ViserWaypointListener()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
