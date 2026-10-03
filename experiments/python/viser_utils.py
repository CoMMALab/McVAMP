from viser.extras import ViserUrdf
import viser
import yourdfpy
from typing import Sequence, Union
import time
import numpy as np


def setup_viser_with_robot(robot_dir, robot_urdf_name):
    server = viser.ViserServer()
    # change the robot here
    urdf = yourdfpy.URDF.load(str(robot_dir / robot_urdf_name))
    robot = ViserUrdf(
        server,
        urdf,
        load_meshes = True,
        load_collision_meshes = False,
        root_node_name = "/robot",
        )

    return server, robot


def add_point_cloud(
    server: viser.ViserServer,
    point_cloud: np.ndarray,
    colors: Union[Sequence[int], Sequence[Sequence[int]]] = [],
    point_size: float = 0.01,
    prefix: str = "my_point_cloud",
    ):
    point_cloud_handle = server.scene.add_point_cloud(
        name = prefix,
        points = point_cloud,
        colors = colors,
        point_size = point_size,
        )
    return point_cloud_handle


def add_spheres(
    server: viser.ViserServer,
    sphere_positions: Sequence,
    sphere_radii: Sequence,
    colors: Union[Sequence[int], Sequence[Sequence[int]]] = [],
    prefix: str = "my_sphere",
    ):
    """
    Add spheres to the env/
    Sphere positions are (N,3) and sphere radii are (N)
    """
    sphere_handles = [None] * len(sphere_positions)
    if len(colors) == 0:
        colors = [[255, 0, 0]] * len(sphere_positions)
    elif len(colors) == 1:
        colors = colors * len(sphere_positions)
    else:
        assert len(colors) == len(sphere_positions)
    for i, (sphere_pos, sphere_rad) in enumerate(zip(sphere_positions, sphere_radii)):
        sphere_handles[i] = server.scene.add_icosphere(
            name = f"{prefix}_{i}",
            radius = sphere_rad,
            position = tuple(sphere_pos[:3]),
            color = tuple(colors[i]),
            )
    return sphere_handles


def add_trajectory(server, waypoints, robot, attachment_handles, attachment_positions):
    """
    Adds a slider to step through waypoints of a trajectory also allows for auto step through
    using play/pause button

    Args:
        server (ViserServer): ViserServer instance
        waypoints (numpy.array): A 2D numpy array (shape: (N,7)) with N waypoints of joint poses
        robot (ViserUrdf): ViserUrdf instance of the robot

        attachment_handles (numpy.array) - this is a P element list of attachment handles, spheres here
        attachment_positions (numpy.array) - this is a (N, P, 3) array of the position of each attachment handle at each waypoint pos.

    Returns:
        return_type: None.
    """
    if len(waypoints) < 1:
        return None
    assert len(attachment_handles) == len(attachment_positions[0])
    traj_slider = server.gui.add_slider(
        "Current Waypoint", min = 0, max = len(waypoints) - 1, step = 1, initial_value = 0
        )

    @traj_slider.on_update
    def update_robot_pose(event):
        waypoint_idx = int(event.target.value)
        joint_config = waypoints[waypoint_idx]
        robot.update_cfg(joint_config)

        for attach_idx, attachment_handle in enumerate(attachment_handles):
            attachment_handle.position = attachment_positions[waypoint_idx][attach_idx]

    return traj_slider


def add_cuboids(server, cuboids, prefix="/environment/cuboid", color=(160, 160, 160)):
    """Draw axis-aligned cuboids given as [(center xyz, half extents xyz), ...]."""
    for i, (center, half) in enumerate(cuboids):
        server.scene.add_box(
            f"{prefix}_{i}",
            color=color,
            dimensions=tuple(float(2.0 * h) for h in half),
            position=tuple(float(c) for c in center),
        )


def add_tsr_region(server, pose, lower, upper, extent=0.6, prefix="/constraint"):
    """Draw a task-space region: `pose` is (qw, qx, qy, qz, x, y, z) and `lower`/`upper` are its position bounds.
    Axes of the pose frame with a wide bound (more than 1 m) are free: one free axis is drawn as a line, two as
    a thin plane, each `extent` meters to either side of the pose."""
    from viser import transforms as tf

    wxyz = np.array(pose[:4], dtype=float)
    origin = np.array(pose[4:7], dtype=float)
    rotation = tf.SO3(wxyz).as_matrix()
    free = [i for i in range(3) if upper[i] - lower[i] > 1.0]

    if len(free) == 1:
        axis = rotation[:, free[0]]
        server.scene.add_line_segments(
            f"{prefix}/line",
            points=np.array([[origin - extent * axis, origin + extent * axis]]),
            colors=(255, 140, 0),
            line_width=4.0,
        )
    elif len(free) == 2:
        dimensions = [0.002, 0.002, 0.002]
        for i in free:
            dimensions[i] = 2.0 * extent
        server.scene.add_box(
            f"{prefix}/plane",
            color=(255, 140, 0),
            dimensions=tuple(dimensions),
            opacity=0.25,
            wxyz=wxyz,
            position=origin,
        )

    server.scene.add_frame(
        f"{prefix}/reference", wxyz=wxyz, position=origin, axes_length=0.1, axes_radius=0.004)


def run_viewer(server, trajectory, show, rate_hz=20.0):
    """Step through `trajectory` with a slider and an autoplay checkbox. `show(q, index)` poses the scene.
    Blocks until interrupted."""
    show(trajectory[0], 0)
    slider = server.gui.add_slider(
        "Current Waypoint", min=0, max=len(trajectory) - 1, step=1, initial_value=0)
    autoplay = server.gui.add_checkbox("Autoplay", initial_value=True)
    rate = server.gui.add_slider("Playback Rate (Hz)", min=1.0, max=60.0, step=1.0, initial_value=rate_hz)

    @slider.on_update
    def _(event):
        index = int(event.target.value)
        show(trajectory[index], index)

    print(f"visualization at http://localhost:{server.get_port()}; ctrl-c to exit")
    while True:
        if autoplay.value:
            slider.value = (slider.value + 1) % len(trajectory)
        time.sleep(1.0 / rate.value)
