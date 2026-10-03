"""Meshcat (pinocchio) playback of Digit trajectories saved as CSV, e.g. with
`python vamp/scripts/digit_example.py --output traj.csv` (vamp repo).

Run: python experiments/python/meshcat_viz.py traj.csv   (needs pinocchio, meshcat; viewer at tcp://127.0.0.1:6000)
Digit URDF/meshes come from $VAMP_ROOT/resources/digit (default: ../vamp).
"""
import time
import numpy as np
import pinocchio
from pinocchio.visualize import MeshcatVisualizer

import meshcat.geometry as mg
import matplotlib.pyplot as plt

from meshcat.transformations import translation_matrix


import tempfile

import os
from pathlib import Path

VAMP_ROOT = Path(os.environ.get("VAMP_ROOT", Path(__file__).parents[2] / "vamp"))
DIGIT_DIR = VAMP_ROOT / "resources" / "digit"


def replace_package_path(urdf_path, mesh_dir=None):
    '''
    Take in a urdf path, search for instances of package://franka_description,
    and then replace with the working directory path.

    Args:
        urdf_path (str): The path to the URDF file.

    Returns:
        str: The modified URDF path with package paths replaced.
    '''
    import re

    # Get all parents in the path
    parents = os.path.dirname(urdf_path).split(os.sep)
    replace_with = os.path.join(os.getcwd(), *parents)

    # Read the URDF file
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    # Replace package://franka_description with the current working directory
    modified_content = re.sub(r'meshes/', str(mesh_dir if mesh_dir is not None else Path(urdf_path).parent / "meshes") + "/", urdf_content)
    # modified_content = re.sub(r'package://urdf', replace_with, urdf_content)

    # Write to a temp file
    tmp = tempfile.NamedTemporaryFile(delete=False)
    with open(tmp.name, 'w') as f:
        f.write(modified_content)

    return tmp.name




# roll pitch yaw
axis_directions = np.pi/2 * np.array([[1, 0, 0], [0, 0, 0], [0, 0, 1]])

axis_colors = [[80, 230, 100],   # Green
               [255, 140, 0],    # Orange
               [190, 80, 250]]   # Purple

def color_to_hex(color):
    '''
    convert RGB color in [0, 255] to int hex
    '''
    return (color[0] << 16) + (color[1] << 8) + color[2]


viz = None
model = None
axs = None
eef_frame_name = "left_arm_eef"
eef_frame = None
data = None

def init_viz():
    # Load robot model
    _model, coll_model, vis_model = pinocchio.buildModelsFromUrdf(
        replace_package_path(str(DIGIT_DIR / "digit_model_trace_collision_spherized.urdf")),
        str(DIGIT_DIR / "meshes"),
    )

    # Set up visualizer
    _viz = MeshcatVisualizer(_model, coll_model, vis_model)
    _viz.initViewer(zmq_url="tcp://127.0.0.1:6000")
    _viz.loadViewerModel()

    fig, _axs = plt.subplots(2, 3, sharex=True, sharey='row', figsize=(12, 8))
    plt.tight_layout(pad=1.0)
    plt.ion()

    # Set viz, model, axs for callback
    global viz, model, axs, eef_frame
    viz = _viz
    model = _model
    axs = _axs
    eef_frame = model.getFrameId(eef_frame_name, pinocchio.BODY)
    global data
    data = model.createData()


    return viz

def clear_all_waypoints():
    '''
    Clear all waypoints from both sets.
    '''
    global viz
    viz.viewer["waypoints_a"].delete()
    viz.viewer["waypoints_b"].delete()
    # viz.viewer["given_waypoints"].delete()
    viz.viewer["waypoints"].delete()

waypoints_set = 'a'

def render_eefs(eeposes):
    """
    Renders the trajectory of a specific frame as a series of coordinate axes.

    Parameters
    ----------
    eeposes
    """
    global waypoints_set, viz
    visualizer = viz
    model = viz.model
    clear_all_waypoints()

    # Draw waypoints as spheres
    for i, eepose in enumerate(eeposes):
        visualizer.viewer[f"waypoints_{waypoints_set}/waypoint_{i}"].set_object(
            mg.Sphere(0.005),
            mg.MeshLambertMaterial(color=color_to_hex([255, 140, 0]))
        )
        visualizer.viewer[f"waypoints_{waypoints_set}/waypoint_{i}"].set_transform(eepose)

    # Toggle the other set of waypoints and delete it
    # (We do this so there isn't a flash when updating)
    waypoints_set = 'b' if waypoints_set == 'a' else 'a'
    visualizer.viewer[f"waypoints_{waypoints_set}"].delete()

def set_attach_object_to_robot(spheres):
    visualizer = viz
    for i, (x, y, z, r) in enumerate(spheres):
        sphere_name = f"/robot/left_ee_tool/sphere_{i}"
        visualizer.viewer[sphere_name].set_object(mg.Sphere(r), mg.MeshLambertMaterial(color=0xff0000))

        # Optional: initial offset relative to end-effector
        T_offset = np.eye(4)
        T_offset[:3, 3] = [x, y, z]
        visualizer.viewer[sphere_name].set_transform(T_offset)

def add_cuboids(cuboids, colors=(12, 89, 178)):
    '''
    cuboids: list of dicts with [x, y, z, yaw, pitch, roll, dx, dy, dz]
    colors: list of RGB tuples in [0, 255] or a single RGB tuple
    '''
    visualizer = viz

    for idx, cuboid in enumerate(cuboids):
        x, y, z = cuboid[0], cuboid[1], cuboid[2]
        yaw, pitch, roll = cuboid[3], cuboid[4], cuboid[5]
        dx, dy, dz = cuboid[6], cuboid[7], cuboid[8]

        T = translation_matrix([x, y, z])

        visualizer.viewer[f"cuboids/cuboid_{idx}"].set_object(
            mg.Box([dx, dy, dz]),
            mg.MeshLambertMaterial(color=color_to_hex(colors[idx] if isinstance(colors, list) else colors))
        )
        visualizer.viewer[f"cuboids/cuboid_{idx}"].set_transform(T)
        print(idx, x, y, z, dx, dy, dz)

def animate(positions, all_times, attachment_spheres=None, attachment_masks=None, rate=1.0, loop=False):
    '''
    Generate a nice interpolated trajectory animation.
    '''
    global viz
    # Get interpolated trajectory
    # all_times, positions = interpolate_trajectory(qs, vs, dts, time_step=0.01)

    # Assert all_times is strictly increasing
    assert np.all(np.diff(all_times) > 0), "all_times must be strictly increasing"

    # Animate
    start_time = time.perf_counter()
    i = 0
    while True:
        # Move to the right time
        elapsed = (time.perf_counter() - start_time) * rate
        if elapsed > all_times[-1]:
            start_time = time.perf_counter()
            if loop: continue
            else: break

        # Bisect find i
        i = np.searchsorted(all_times, elapsed, side="right") - 1
        i = np.clip(i, 0, positions.shape[0] - 1)

        viz.display(positions[i]) # assume 7

        # if attachment_spheres is not None and (attachment_masks is None or attachment_masks[i]):
        #     pinocchio.forwardKinematics(model, data, positions[i])
        #     pinocchio.updateFramePlacements(model, data)
        #     print(eef_frame)
        #     ee_pose = data.oMf[eef_frame].homogeneous
        #     for j, (x, y, z, r) in enumerate(attachment_spheres):
        #         # print(f"Animating sphere {j} at position ({x}, {y}, {z}) with radius {r}")
        #         sphere_name = f"/robot/left_ee_tool/sphere_{j}"
        #         sphere_poses = ee_pose @ translation_matrix([x, y, z])  # apply offset relative to EE
        #         viz.viewer[sphere_name].set_transform(sphere_poses)

    viz.viewer["waypoints"].delete()

if __name__ == '__main__':
    import sys
    traj = np.loadtxt(sys.argv[1] if len(sys.argv) > 1 else "trajectory.txt", delimiter = ",")
    init_viz()
    clear_all_waypoints()
    print(traj.shape)
    animate(traj, np.arange(0, len(traj), dtype=np.float64) / 100, loop=True)
