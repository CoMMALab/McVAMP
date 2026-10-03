"""Bimanual Panda holding a rigid object: relative-pose constraint through a shelf.

The right end-effector keeps a fixed transform relative to the left one (as if both arms
grasp one rigid object), enforced with BimanualTaskSpaceConstraint passed as
`constraints=[...]` to a regular planner (rrtc / aorrtc / grrtstar). Start and goal are
projected onto the constraint manifold first (the planners reject off-manifold ones).

Port of the old scripts/bimanual_constraint_example.py (crrtc + Composable_* API).

Run:
    python experiments/python/bimanual_panda_example.py [--planner rrtc] [--repeat 20] [--visualize]
"""

from pathlib import Path
import time

import numpy as np
import vamp
from fire import Fire

ASSETS = Path(__file__).parents[1] / "assets"
VAMP_ROOT = Path(__import__("os").environ.get("VAMP_ROOT", Path(__file__).parents[2] / "vamp"))

# Pose (qw, qx, qy, qz, x, y, z) of the right end-effector in the left end-effector frame.
RIGHT_IN_LEFT = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.221814]

START = [-1.362, 1.319, 1.064, -2.486, 0.518, 2.481, -1.459,
         1.327, 1.260, -1.048, -2.481, -0.644, 2.444, -0.011]
GOAL = [-2.143, 0.395, 2.249, -2.043, 1.320, 1.772, -0.697,
        1.359, 1.320, -2.092, -2.138, -0.140, 2.437, -1.547]


def main(
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    range_: float = 1.0,  # Planner range.
    tolerance: float = 0.001,  # Half-width of the relative-pose bounds.
    shelf: str = str(ASSETS / "environments" / "shelf" / "drake.txt"),  # CSV rows: x,y,z,dx,dy,dz (full extents).
    repeat: int = 20,  # Timing trials (the original ran 20).
    visualize: bool = False,
    **kwargs,
):
    if "bimanual_panda" not in vamp.robots:
        raise SystemExit(
            "This vamp build does not include the 'bimanual_panda' robot (it has: " + ", ".join(vamp.robots) + ").\n"
            "Rebuild vamp with it, for example:\n"
            "    pip install . -C cmake.define.VAMP_ROBOTS=\"ur5;panda;fetch;bimanual_panda\"\n"
            "See the README, section 6.")

    module, planner_func, plan_settings, simp_settings = (
        vamp.configure_robot_and_planner_with_kwargs("bimanual_panda", planner, **kwargs))

    rrtc_settings = plan_settings.rrtc if planner == "aorrtc" else plan_settings
    rrtc_settings.range = range_
    if planner != "grrtstar":
        rrtc_settings.dynamic_domain = False

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.max_iterations = 15

    constraints = [
        module.BimanualTaskSpaceConstraint(RIGHT_IN_LEFT, [-tolerance] * 6, [tolerance] * 6)
    ]

    e = vamp.Environment()
    for cuboid in np.loadtxt(shelf, delimiter=","):
        e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    sampler = module.halton()

    seed_settings = vamp.ConstraintSettings()
    seed_settings.max_iterations = 100
    start = module.project(np.array(START, dtype=np.float32), constraints, seed_settings)
    goal = module.project(np.array(GOAL, dtype=np.float32), constraints, seed_settings)

    times = []
    result = None
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

    on_manifold = all(module.satisfied(path[i], constraints, constraint_settings) for i in range(len(path)))
    print(f"planning time: mean {np.mean(times):.2f} ms, std {np.std(times):.2f} ms over {repeat} run(s)")
    print(f"path: {len(result.path)} -> {len(path)} states, cost {result.path.cost():.4f} -> {path.cost():.4f}")
    print(f"all waypoints on manifold: {on_manifold}")

    if visualize:
        from viser_utils import setup_viser_with_robot, add_trajectory

        server, robot = setup_viser_with_robot(VAMP_ROOT / "resources" / "panda", "bipanda_spherized.urdf")
        robot.update_cfg(np.asarray(start))
        server.scene.add_grid("/floor_grid", width=10.0, height=10.0, plane="xy")
        for i, c in enumerate(np.loadtxt(shelf, delimiter=",")):
            server.scene.add_box(f"/environment/cuboid_{i}", color=(160, 160, 160),
                                 dimensions=tuple(c[3:6]), position=tuple(c[:3]))
        # numpy() is a read-only view and the viser helper wants a writable array.
        add_trajectory(server, path.numpy().copy(), robot, [], [[]])
        print("visualization at http://localhost:8080; ctrl-c to exit")
        while True:
            time.sleep(1.0)


if __name__ == "__main__":
    Fire(main)
