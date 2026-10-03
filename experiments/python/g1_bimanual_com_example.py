"""Unitree G1 humanoid: feet + CoM + bimanual constraints (REQUIRES A GENERATED ROBOT).

The G1 is not built into the `mcvamp` branch of vamp. Generate it with cricket
(https://github.com/CoMMALab/cricket, or the JIT via `vamp.jit.load_robot(...)`; see the
experiments README) so that a robot named `g1_unitree` shows up in `vamp.robots`, with
end-effectors ordered: left hand, right hand, left foot, right foot, and a center-of-mass
kernel. This script exits with a message if `vamp.g1_unitree` is absent.

Port of the old scripts/g1_test.py (crrtc + Composable_* API). Constraint stack:
  - TaskSpaceConstraint: feet pinned at their stance offsets (hands free)
  - CoMConstraint: CoM inside the support polygon
  - BimanualTaskSpaceConstraint: hands hold a fixed relative transform (carry an object)

Run:
    python experiments/python/g1_bimanual_com_example.py [--planner rrtc] [--trials 10] [--quick] [--visualize]

--visualize plays the last simplified path in viser, with the robot drawn as its collision spheres (the G1's visual
meshes are not part of this tree), the shelf, and the support polygon. Needs viser.
"""

from pathlib import Path
import time

import itertools

import numpy as np
import vamp
from fire import Fire

ASSETS = Path(__file__).parents[1] / "assets"

FREE = [10.0] * 6
FOOT_BOUND = [0.001, 0.001, 0.001, 0.005, 0.005, 0.005]
IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

SUPPORT_POLYGON = [(0.08, -0.045), (0.08, 0.045), (0.06, 0.045), (0.06, -0.045)]

START = [-0.005940199, -0.00012779236, 0.5253687, -0.0005400387, 0.013260534, -0.00089769263, -0.8943771,
         -0.007457934, 0.015044728, 1.758194, -0.87267, 0.017328043, -0.8946327, 0.010154848, -0.015723394,
         1.7575748, -0.87267, -0.01812963, 1.5915728, 0.13870776, 0.37364945, 0.0016204158, -0.007157203,
         -0.0032037592, 0.0036042016, 0.002070119, -0.008524217, 0.0055902405, -0.0023131033, -0.0070516965,
         -0.0035007126, -0.0020867267, 0.0020755264, 0.0010021132, 0.005703019]
GOAL = [0.042186737, 4.2915344e-06, 0.72114706, -3.3705605e-06, 0.03055413, -2.003611e-05, -0.30831593,
        -0.011244297, 0.0050789025, 0.6614378, -0.3893154, 0.012345497, -0.30834094, 0.011252999,
        -0.0050491523, 0.66147137, -0.38932344, -0.0123702455, -2.5435329e-05, 8.368492e-06, -0.24609257,
        0.008559517, -0.0059906836, -0.0037853387, -0.00089317013, 0.0042787157, -0.0032093797, 0.008020755,
        0.002362628, 0.0043031597, -0.005259495, 0.0026970499, -0.0058816765, -0.00028618058, 0.014119654]


def visualize_path(module, shelf_rows, waypoints):
    import viser
    from viser_utils import add_cuboids, run_viewer

    server = viser.ViserServer()
    server.scene.add_grid("/floor_grid", width=10.0, height=10.0, plane="xy")
    add_cuboids(server, [(row[:3], row[3:6] / 2.0) for row in shelf_rows], color=(160, 110, 60))

    # The support polygon is in world axes; draw it on the ground.
    corners = [[vx, vy, 0.0] for vx, vy in SUPPORT_POLYGON]
    server.scene.add_line_segments(
        "/constraint/support_polygon",
        points=np.array([[corners[i], corners[(i + 1) % len(corners)]] for i in range(len(corners))]),
        colors=(255, 140, 0),
        line_width=4.0)

    spheres = module.fk(np.asarray(waypoints[0], dtype=np.float32))
    handles = [server.scene.add_icosphere(f"/robot/sphere_{k}", radius=float(sphere.r),
                                          position=tuple(sphere.position), color=(90, 140, 200))
               for k, sphere in enumerate(spheres)]

    def show(q, _):
        for handle, sphere in zip(handles, module.fk(np.asarray(q, dtype=np.float32))):
            handle.position = tuple(sphere.position)

    run_viewer(server, waypoints, show)


def main(
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    shelf: str = str(ASSETS / "environments" / "shelf" / "humanoid.txt"),  # CSV rows: x,y,z,dx,dy,dz (full extents).
    trials: int = 10,  # Rounds over the whole grid (the original ran 10).
    quick: bool = False,  # One configuration (range 0.5), one round.
    visualize: bool = False,  # Play the last simplified path in viser.
    **kwargs,
):
    """Original study: planner range in {0.2, 0.5, 0.75, 1.0, 1.5}, dynamic domain off, with
    insert-all-waypoints on (`emit_all_waypoints`), perturbation scale 0.1, 15 projection
    iterations and descend rate 1.0, `trials` rounds sharing one sampler; prints
    mean/std/min/max/median planning time per combination."""
    if not hasattr(vamp, "g1_unitree"):
        raise SystemExit(
            "vamp.g1_unitree not found: the G1 robot is not part of the mcvamp branch. Generate it "
            "with cricket (or the vamp JIT) first; see the README, section 5.")

    module, planner_func, plan_settings, simp_settings = (
        vamp.configure_robot_and_planner_with_kwargs("g1_unitree", planner, **kwargs))
    rrtc_settings = plan_settings.rrtc if planner == "aorrtc" else plan_settings

    feet = module.TaskSpaceConstraint(
        [IDENTITY, IDENTITY, [1, 0, 0, 0, 0.12, 0.11, 0.0], [1, 0, 0, 0, 0.12, -0.11, 0.0]],
        [IDENTITY] * 4,
        [[-b for b in FREE], [-b for b in FREE], [-b for b in FOOT_BOUND], [-b for b in FOOT_BOUND]],
        [FREE, FREE, FOOT_BOUND, FOOT_BOUND])
    com = module.CoMConstraint(SUPPORT_POLYGON)
    hands = module.BimanualTaskSpaceConstraint(
        [1.0, 0.0005, 0.0005, 0.0005, 0.0, -0.303, 0.0], [-0.001] * 6, [0.001] * 6)
    constraints = [feet, com, hands]

    # Old constraint_settings -> ConstraintSettings: insert_all_to_tree -> emit_all_waypoints,
    # std_dev_scaling_factor -> perturbation_scale, num_projection_iterations -> max_iterations.
    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.emit_all_waypoints = True
    constraint_settings.perturbation_scale = 0.1
    constraint_settings.max_iterations = 15
    constraint_settings.descend_rate = 1.0

    e = vamp.Environment()
    shelf_rows = np.atleast_2d(np.loadtxt(shelf, delimiter=","))
    for c in shelf_rows:
        e.add_cuboid(vamp.Cuboid(c[:3], [0, 0, 0], c[3:6] / 2))
    sampler = module.halton()

    seed_settings = vamp.ConstraintSettings()
    seed_settings.max_iterations = 50
    seed_settings.descend_rate = 0.75
    start = module.project(np.array(START, dtype=np.float32), constraints, seed_settings)
    goal = module.project(np.array(GOAL, dtype=np.float32), constraints, seed_settings)

    ranges = [0.5] if quick else [0.2, 0.5, 0.75, 1.0, 1.5]
    dyndoms = [False]
    combinations = list(itertools.product(ranges, dyndoms))
    planning_times = {combination: [] for combination in combinations}
    for _ in range(1 if quick else trials):
        for combination in combinations:
            rrtc_settings.range = combination[0]
            rrtc_settings.dynamic_domain = combination[1]
            result = planner_func(
                start, goal, e, plan_settings, sampler,
                constraints=constraints, constraint_settings=constraint_settings)
            planning_times[combination].append(result.nanoseconds / 1e6)

    print("Execution completed")
    print("Planning times for each combination:")
    for combination, times in planning_times.items():
        print(f"Combination {combination}: {np.mean(times)} ms {np.std(times)} ms {np.min(times)} ms "
              f"{np.max(times)} ms {np.median(times)} ms  (mean std min max median)")

    if result.solved:
        path = module.simplify(
            result.path, e, simp_settings, sampler,
            constraints=constraints, constraint_settings=constraint_settings).path
        print(f"last path: {len(result.path)} -> {len(path)} states")
        if visualize:
            visualize_path(module, shelf_rows, path.numpy().copy())
    elif visualize:
        print("nothing to visualize: the last attempt did not solve")


if __name__ == "__main__":
    Fire(main)
