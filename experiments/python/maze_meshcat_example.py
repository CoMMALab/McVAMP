"""Panda carrying a stick through a narrow maze: end-effector held level (constrained), meshcat playback.

The TaskSpaceConstraint keeps the end-effector orientation and height fixed (free in x, y and
yaw) while a rod of small spheres attached to the end-effector is threaded through the maze
cuboids from assets/environments/maze/cuboids.json. The simplified path is played back in meshcat with
meshcat_viz_utils.MeshcatViz (needs pinocchio + meshcat).

Port of the old scripts/visualize_meshcat_example.py (crrtc + Composable_* API).

Run:
    python experiments/python/maze_meshcat_example.py [--planner rrtc] [--no_visualize]
"""

import json
import os
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

ASSETS = Path(__file__).parents[1] / "assets"
VAMP_ROOT = Path(os.environ.get("VAMP_ROOT", Path(__file__).parents[2] / "vamp"))

START = [-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294]
GOAL = [1.40490, 0.35201, -0.22762, -1.90963, 0.10796, 2.26183, 0.22238]


def add_json_cuboids(e, filename):
    """Add cuboids from a JSON list of {name, x, y, z, dx, dy, dz} (full extents) to `e`.
    Returns (cuboids as [x, y, z, r, p, y, dx, dy, dz], colors) for visualization."""
    with open(filename) as f:
        data = json.load(f)

    cuboids, colors = [], []
    for i, obj in enumerate(data):
        name = obj.get("name", f"cuboid_{i}")
        x, y, z = (obj.get(k, 0.0) for k in ("x", "y", "z"))
        dx, dy, dz = (obj.get(k, 0.0) for k in ("dx", "dy", "dz"))
        e.add_cuboid(vamp.Cuboid([x, y, z], [0.0, 0.0, 0.0], [dx / 2, dy / 2, dz / 2]))
        cuboids.append([x, y, z, 0.0, 0.0, 0.0, dx, dy, dz])
        colors.append((101, 67, 33) if "wall" in name.lower() else (210, 180, 140))
    return cuboids, colors


def main(
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    range_: float = 0.5,
    attachment_radius: float = 0.01,
    visualize: bool = True,
    **kwargs,
):
    module, planner_func, plan_settings, simp_settings = (
        vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))
    if planner == "aorrtc":
        plan_settings.rrtc.range = range_
    else:
        plan_settings.range = range_

    # Stick along the end-effector z axis.
    attachment = vamp.Attachment(np.identity(4), 0)
    attach_positions = np.zeros((10, 3))
    attach_positions[:, 2] = np.linspace(0, 0.16, len(attach_positions))
    attachment.add_spheres([vamp.Sphere(p, attachment_radius) for p in attach_positions])

    tsr = module.TaskSpaceConstraint(
        [1, 0, 0, 0, 0, 0, 0],
        [0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783],
        [-10.01, -10.01, -0.001, -0.01, -0.01, -10.01],
        [10.01, 10.01, 0.001, 0.01, 0.01, 10.01])
    constraints = [tsr]
    constraint_settings = vamp.ConstraintSettings()

    e = vamp.Environment()
    env_cuboids, env_colors = add_json_cuboids(e, ASSETS / "environments" / "maze" / "cuboids.json")
    e.attach(attachment)

    sampler = module.halton()
    seed_settings = vamp.ConstraintSettings()
    seed_settings.max_iterations = 100
    start = module.project(np.array(START, dtype=np.float32), constraints, seed_settings)
    goal = module.project(np.array(GOAL, dtype=np.float32), constraints, seed_settings)

    result = planner_func(
        start, goal, e, plan_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings)
    if not result.solved:
        raise RuntimeError("planning failed")
    simple = module.simplify(
        result.path, e, simp_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings)
    print(f"Planned path with {len(result.path)} waypoints, simplified to {len(simple.path)}, "
          f"in {result.nanoseconds / 1e6:.1f} ms")

    if visualize:
        import meshcat_viz_utils as viz

        # Paths are already dense on the manifold (planners emit whole projected chains).
        waypoints = simple.path.numpy()
        viz_instance = viz.MeshcatViz()
        panda_dir = VAMP_ROOT / "resources" / "panda"
        viz_instance.init_viz(str(panda_dir / "panda_spherized.urdf"), str(panda_dir / "meshes") + "/")
        viz_instance.clear_all_waypoints()
        viz_instance.add_cuboids(env_cuboids, colors=env_colors)
        eef_poses = np.array([np.array(module.eefk(w)) for w in waypoints])
        viz_instance.render_eefs(eef_poses)
        viz_instance.animate(waypoints, np.arange(len(waypoints), dtype=np.float64) / 50, loop=True)


if __name__ == "__main__":
    Fire(main)
