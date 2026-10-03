"""ROS2 node: replans a constrained Panda trajectory around a moving spherical obstacle.

Pairs with viser_ros_listener.py (which shows the robot, lets you drag the obstacle, and
publishes its pose). The end-effector is held level by a TaskSpaceConstraint passed as
`constraints=[...]` to the rrtc planner; a stick is attached to the end-effector and the
maze from assets/environments/maze/cuboids.json is the static environment. Waypoints are published as
Float32MultiArray messages. Planned paths are already dense on the constraint manifold, so
they are NOT re-interpolated (linear interpolation would leave the manifold).

Requires ROS2 (rclpy). Run:
    python experiments/python/viser_dynamic_obstacle_avoid.py
"""

from pathlib import Path
from typing import Optional, List
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
import numpy as np
import vamp
import json

from sensor_msgs.msg import JointState


def add_json_cuboids(filename, color=(12, 89, 178)):
    """
    Add cuboids from a JSON file. The JSON should be a list of objects, each with keys:
    'name', 'x', 'y', 'z', 'dx', 'dy', 'dz' and optional 'roll','pitch','yaw'.

    This function reads the JSON, iterates over entries, and adds a box for each item.
    Dimensions are used as provided in the JSON (assumed full extents).
    """
    with open(filename, "r") as f:
        data = json.load(f)

    vamp_cuboids = []

    for i, obj in enumerate(data):
        name = obj.get("name", f"cuboid_{i}")
        x = obj.get("x", 0.0)
        y = obj.get("y", 0.0)
        z = obj.get("z", 0.0)
        dx = obj.get("dx", 0.0)
        dy = obj.get("dy", 0.0)
        dz = obj.get("dz", 0.0)

        vamp_cuboids.append(vamp.Cuboid([x, y, z], [0.0, 0.0, 0.0], [dx / 2, dy / 2, dz / 2]))

    return vamp_cuboids




class ConstrainedVampPubSub(Node):
    """ROS2 node for constrained planner trajectory planning and replanning using VAMP."""

    def setup_planner_and_constraints(self, **kwargs) -> None:
        """Initialize the VAMP planner and configure constraints."""


        (self.vamp_module, self.planner_func, self.plan_settings, self.simp_settings) = (
            vamp.configure_robot_and_planner_with_kwargs("panda", "rrtc", **kwargs)
        )
        tsr = self.vamp_module.TaskSpaceConstraint(
            [1, 0, 0, 0, 0, 0, 0],
            [0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783],
            [-10.01, -10.01, -0.01, -0.01, -0.01, -10.01],
            [10.01, 10.01, 0.01, 0.01, 0.01, 10.01]
        )
        self.constraints = [tsr]
        self.constraint_settings = vamp.ConstraintSettings()
        self.project_settings = vamp.ConstraintSettings()
        self.project_settings.max_iterations = 50
        
        self.sampler = self.vamp_module.halton()

        self.goals = [
            [-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294],
            [1.40490, 0.35201, -0.22762, -1.90963, 0.10796, 2.26183, 0.22238]
        ]
        self.plan_settings.max_iterations = 100000
        self.plan_settings.range = 0.2

        self.vamp_cuboids = add_json_cuboids(str(Path(__file__).parents[1] / "assets" / "environments" / "maze" / "cuboids.json"))

        self.current_goal = 1

        # For the maze
        tf = np.identity(4)
        self.attachment = vamp.Attachment(tf, 0)
        attach_positions = np.zeros((10, 3))
        attach_positions[:, 2] = np.linspace(0, 0.16, len(attach_positions))

        # Add a single sphere to the attachment - spheres are added in the attachment's local frame
        self.attachment.add_spheres(
            [
                vamp.Sphere(pos, 0.01) for pos in attach_positions
            ]
        )


    def plan(self, start_config, goal_config):
        # both are given in the format that vamp likes so no worries here
        
        t1 = time.perf_counter_ns()
        env = vamp.Environment()
        env.add_sphere(vamp.Sphere(self.obstacle_pose_and_dim[:3], self.obstacle_pose_and_dim[-1]))
        for cuboid in self.vamp_cuboids:
            env.add_cuboid(cuboid)
        env.attach(self.attachment)


        # Start (and goal) must lie on the manifold: project the start when the arm has drifted off it.
        start_config = np.asarray(start_config, dtype=np.float32)
        if not self.vamp_module.satisfied(start_config, self.constraints, self.constraint_settings):
            start_config = self.vamp_module.project(start_config, self.constraints, self.project_settings)

        # now first validate if existing self.trajectory is itself valid
        # we do that by connecting start_config to the currently executing config in the trajectory, and check if all lines are valid. If not, we trigger a replan. This is a bit hacky but it works for now.
        curr_waypoint_index = self.current_time / self.waypoint_dt
        all_waypoints_valid = True
        if self.trajectory is not None and self.trajectory.shape[0] > 0 and (np.linalg.norm(self.trajectory[-1] - goal_config) < 1e-3):  # Only check if we have a trajectory and we're not already at the goal
            for i in range(int(curr_waypoint_index) - 1, self.trajectory.shape[0] - 2):
                if i == int(curr_waypoint_index) - 1:
                    start_config_to_check = start_config
                else:
                    start_config_to_check = self.trajectory[i]
                if (self.vamp_module.validate_motion(start_config_to_check, self.trajectory[i+1], env) == False):
                    self.get_logger().warn("Failed to validate motion from start_config_to_check to trajectory waypoint %d" % (i+1))
                    all_waypoints_valid = False
                    break
            if all_waypoints_valid:
                print("No change, computed in %.2f ms" % ((time.perf_counter_ns() - t1) / 1e6))
                return
        else:
            if self.trajectory is not None and self.trajectory.shape[0] > 0:
                print("end traj and goal are different : ", self.trajectory[-1], goal_config, np.linalg.norm(self.trajectory[-1] - goal_config))


        goal_config = self.vamp_module.project(
            np.asarray(goal_config, dtype=np.float32), self.constraints, self.project_settings)
        result = self.planner_func(
            start_config, goal_config, env, self.plan_settings, self.sampler,
            constraints=self.constraints, constraint_settings=self.constraint_settings)
        if (result.solved == False):
            self.get_logger().warn("Failed to find a solution! start_config=%s, goal_config=%s, cur_obstacle=%s" % (start_config, goal_config, self.obstacle_pose_and_dim))
            self.trajectory = np.array([start_config])
            return
        
        if (len(result.path) > 3):
            simple = self.vamp_module.simplify(
                result.path, env, self.simp_settings, self.sampler,
                constraints=self.constraints, constraint_settings=self.constraint_settings)
        else:
            simple = result           
        # simple = self.vamp_module.simplify(result.path, env, self.simp_settings, self.sampler)
        self.trajectory = simple.path.numpy()
        self.current_time = 0.0
        self.trajectory_finished = False
        print("Planned trajectory with %d waypoints in %.2f mseconds" % (self.trajectory.shape[0], (time.perf_counter_ns() - t1) / 1e6))




    def __init__(self):
        super().__init__('humanoid_vamp_pubsub_py')

        # Parameters
        self.declare_parameter('publish_rate_hz', 100.0)  # Publishing frequency in Hz
        self.declare_parameter('waypoint_dt', 0.1)  # Time between waypoints (in the txt file) in seconds (we can reduce this to be smaller than 1 to increase the roll out speed.)
        self.declare_parameter('planner_rate_hz', 10.0)  # Publishing frequency in Hz


        self.setup_planner_and_constraints()
        self.current_waypoint = self.goals[0]
        
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.waypoint_dt = self.get_parameter('waypoint_dt').get_parameter_value().double_value
        self.planner_rate_hz = self.get_parameter('planner_rate_hz').get_parameter_value().double_value

        self.trajectory: Optional[np.ndarray] = None
        self.is_playing = True
        self.current_time = 0.0
        self.trajectory_finished = False
        
        # Obstacle pose and dimensions (8-D array: x, y, z, qw, qx, qy, qz, scale or similar)
        self.obstacle_pose_and_dim: np.ndarray = np.zeros(8)

        self.publisher = self.create_publisher(Float32MultiArray, 'planner_waypoint', 10)
        self.ee_trajectory_publisher = self.create_publisher(Float32MultiArray, 'ee_trajectory', 10)
        self.subscription = self.create_subscription(Int32, 'get_next_waypoint', self.on_get_next_waypoint, 10)
        self.obstacle_subscription = self.create_subscription(Float32MultiArray, '/obstacle_pose_and_dim', self.on_obstacle_pose_and_dim, 10)
        
        # Create timer for high-frequency publishing
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.planner_timer = self.create_timer(1.0 / self.planner_rate_hz, self.planner_callback)


    def on_get_next_waypoint(self, msg: Int32) -> None:
        """Callback to start trajectory playback."""
        if msg.data != 1:
            return

        if self.trajectory is None or self.trajectory.shape[0] == 0:
            self.get_logger().warn("No trajectory loaded; cannot start publishing.")
            return

        if not self.is_playing:
            self.get_logger().info("Starting trajectory playback.")
            self.is_playing = True
            self.current_time = 0.0
            self.trajectory_finished = False

    def on_obstacle_pose_and_dim(self, msg: Float32MultiArray) -> None:
        """Callback to receive obstacle pose and dimensions."""
        if len(msg.data) != 8:
            self.get_logger().warn(f"Expected 8 values for obstacle_pose_and_dim, got {len(msg.data)}")
            return
        
        self.obstacle_pose_and_dim = np.array(msg.data, dtype=np.float32)
        # self.get_logger().info(f"Updated obstacle pose and dimensions: {self.obstacle_pose_and_dim}")

    def planner_callback(self):
        # get the current waypoint and plan to the next one
        start_config = self.current_waypoint
        # start_config = self.goals[self.current_goal - 1]
        goal_config = self.goals[self.current_goal]
        self.plan(start_config, goal_config)
        
        # Publish end-effector trajectory
        if self.trajectory is not None and self.trajectory.shape[0] > 0:
            ee_positions = []
            for config in self.trajectory:
                ee_pos = self.vamp_module.eefk(config)[0][:3, 3].tolist()
                ee_positions.extend(ee_pos)
            
            ee_msg = Float32MultiArray()
            ee_msg.data = [float(v) for v in ee_positions]
            self.ee_trajectory_publisher.publish(ee_msg)

    def timer_callback(self):
        if not self.is_playing or self.trajectory is None or self.trajectory.shape[0] == 0:
            return

        # Calculate which waypoint we're at
        total_duration = (self.trajectory.shape[0] - 1) * self.waypoint_dt
        
        if self.current_time >= total_duration:
            # We've reached the end - keep publishing the last waypoint
            if not self.trajectory_finished:
                self.get_logger().info("Reached end of trajectory. Publishing last waypoint continuously.")
                self.trajectory_finished = True
            
            wp = self.trajectory[-1, :].tolist()
            waypoint_num = self.trajectory.shape[0] - 1
        else:
            # Interpolate between waypoints
            waypoint_index = self.current_time / self.waypoint_dt
            idx_low = int(np.floor(waypoint_index))
            idx_high = min(idx_low + 1, self.trajectory.shape[0] - 1)
            alpha = waypoint_index - idx_low  # Interpolation factor [0, 1]
            
            # Linear interpolation
            wp_low = self.trajectory[idx_low, :]
            wp_high = self.trajectory[idx_high, :]
            wp = ((1 - alpha) * wp_low + alpha * wp_high).tolist()
            
            waypoint_num = waypoint_index
            
            # Advance time
            self.current_time += 1.0 / self.publish_rate_hz

        # Publish the waypoint
        out = Float32MultiArray()
        out.data = [float(v) for v in wp]
        self.publisher.publish(out)
        self.current_waypoint = wp

        if(np.linalg.norm(np.array(wp) - np.array(self.goals[self.current_goal])) < 1e-3):
            self.get_logger().info("Reached goal.")
            #  move to next goal, and circle back if reached end
            self.current_goal = (self.current_goal + 1) % len(self.goals)



        # Print waypoint info
        # self.get_logger().info(f"Publishing waypoint {waypoint_num:.2f} at time {self.current_time:.4f}s")

def main(args=None):
    rclpy.init(args=args)
    node = ConstrainedVampPubSub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
