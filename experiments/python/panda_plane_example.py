"""Panda end-effector sliding in a fixed-orientation plane through a sphere cage.

The TaskSpaceConstraint pins the end-effector's z-translation and orientation in the
frame of a reference pose, leaving x/y free; the plan is found with a regular planner
(rrtc / aorrtc / grrtstar) given `constraints=[...]`.

Port of the old scripts/test_constraints.py (crrtc + Composable_TaskSpaceConstraint).

Run:
    python experiments/python/panda_plane_example.py [--planner rrtc] [--repeat 20] [--visualize]
"""

from pathlib import Path
import os
import time

import numpy as np
import vamp
from fire import Fire

VAMP_ROOT = Path(os.environ.get("VAMP_ROOT", Path(__file__).parents[2] / "vamp"))

CAGE = [
    [0.56, 0, 0.450], [-0.55, 0, 0.25], [-0.35, -0.35, 0.25], [0.35, 0.35, 0.8],
    [0, 0.55, 0.8], [-0.35, 0.35, 0.8], [-0.55, 0, 0.8], [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8], [0.35, -0.35, 0.8],
]

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Reference frame (qw, qx, qy, qz, x, y, z): the end-effector slides in its local x-y plane.
PLANE_POSE = [0.0, 0.707107, 0.0, 0.707107, 0.354, 0.7, 0.243]

START = [-1.053, -1.39, 1.878, -1.434, -0.531, 2.386, 2.761]
GOAL = [-2.132, 1.558, 1.406, -1.452, 0.228, 2.444, -1.034]


def main(
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    range_: float = 0.5,
    obstacle_radius: float = 0.15,
    repeat: int = 20,  # Timing trials (the original ran 20).
    visualize: bool = False,
    **kwargs,
):
    module, planner_func, plan_settings, simp_settings = (
        vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))
    if planner == "aorrtc":
        plan_settings.rrtc.range = range_
    else:
        plan_settings.range = range_

    tsr = module.TaskSpaceConstraint(
        IDENTITY, PLANE_POSE,
        [-10.01, -10.01, -0.01, -0.01, -0.01, -0.01],
        [10.01, 10.01, 0.01, 0.01, 0.01, 0.01])
    constraints = [tsr]
    constraint_settings = vamp.ConstraintSettings()

    e = vamp.Environment()
    for sphere in CAGE:
        e.add_sphere(vamp.Sphere(sphere, obstacle_radius))
    sampler = module.halton()

    seed_settings = vamp.ConstraintSettings()
    seed_settings.max_iterations = 100
    start = module.project(np.array(START, dtype=np.float32), constraints, seed_settings)
    goal = module.project(np.array(GOAL, dtype=np.float32), constraints, seed_settings)

    np.set_printoptions(precision=4, suppress=True)
    print("start eef:\n", np.array(module.eefk(start)))
    print("goal eef:\n", np.array(module.eefk(goal)))

    times = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        result = planner_func(
            start, goal, e, plan_settings, sampler,
            constraints=constraints, constraint_settings=constraint_settings)
        times.append((time.perf_counter() - t0) * 1e3)
        if not result.solved:
            raise RuntimeError("planning failed")

    simple = module.simplify(
        result.path, e, simp_settings, sampler,
        constraints=constraints, constraint_settings=constraint_settings)
    path = simple.path
    print(f"planning time: mean {np.mean(times):.2f} ms, std {np.std(times):.2f} ms over {repeat} run(s)")
    print(f"path: {len(result.path)} -> {len(path)} states, cost {result.path.cost():.4f} -> {path.cost():.4f}")
    print("all waypoints on manifold:",
          all(module.satisfied(path[i], constraints, constraint_settings) for i in range(len(path))))

    if visualize:
        from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory

        server, robot = setup_viser_with_robot(VAMP_ROOT / "resources" / "panda", "panda_spherized.urdf")
        robot.update_cfg(np.asarray(start))
        server.scene.add_grid("/floor_grid", width=10.0, height=10.0, plane="xy")
        add_spheres(server, CAGE, [obstacle_radius] * len(CAGE), prefix="/environment/sphere")
        server.scene.add_box("/constraint/plane", color=(255, 140, 0), dimensions=(1.4, 1.4, 0.002),
                             opacity=0.25, wxyz=np.array(PLANE_POSE[:4]), position=np.array(PLANE_POSE[4:]))
        add_trajectory(server, path.numpy().copy(), robot, [], [[]])
        print("visualization at http://localhost:8080; ctrl-c to exit")
        while True:
            time.sleep(1.0)


if __name__ == "__main__":
    Fire(main)
