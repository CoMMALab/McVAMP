"""Shared runner for the benchmark problem sets (assets/problems/benchmark/).

Many independent problems, one fixed set of planner and projection settings per problem, as in the
original constrained benchmarker. Reports the solve rate and the time and iteration statistics of the
solved problems. The C++ counterpart is experiments/cpp/benchmark_suite.hh.
"""

import json
import os
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

import numpy as np
import vamp

ASSETS = Path(__file__).resolve().parents[1] / "assets"
VAMP_ROOT = Path(os.environ.get("VAMP_ROOT", Path(__file__).resolve().parents[2] / "vamp"))


@dataclass
class Knobs:
    range_: float = 1.0
    projection_iterations: int = 15
    descend_rate: float = 1.0
    method: str = "OuterLM"
    emit_all_waypoints: bool = True
    perturbation_scale: float = 0.1
    max_iterations: int = 200000
    dd_radius: float = 1.0


@dataclass
class Case:
    start: list
    goal: list
    make_environment: Callable[[], "vamp.Environment"]
    make_constraints: Callable[[object], list]  # module -> [constraint]
    num_obstacles: int = 0

    # Only used by --visualize: the cuboids [(center, half extents)], the task-space region
    # (pose, lower, upper position bounds) and the attached spheres [(position in the hand frame, radius)].
    cuboids: list = field(default_factory=list)
    tsr: Optional[tuple] = None
    attachment: list = field(default_factory=list)


def cuboid_environment(cuboids, attachment=None):
    """Environment from [(center, half_extents)] with an optional attachment."""
    e = vamp.Environment()
    for center, half in cuboids:
        e.add_cuboid(vamp.Cuboid(center, [0.0, 0.0, 0.0], half))
    if attachment is not None:
        e.attach(attachment)
    return e


def run_suite(name, cases: List[Case], knobs: Knobs, planner="rrtc", shared_rng=False, first=0,
              limit: Optional[int] = None, results: str = "", tolerance=None, hold_satisfied_rows=None,
              raw_seeds=False, visualize=False, **kwargs):
    module, planner_func, plan_settings, simp_settings = (
        vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))

    rrtc = plan_settings.rrtc if planner == "aorrtc" else plan_settings
    rrtc.range = knobs.range_
    rrtc.dynamic_domain = False
    rrtc.radius = knobs.dd_radius
    plan_settings.max_iterations = knobs.max_iterations
    if hasattr(plan_settings, "max_samples"):
        plan_settings.max_samples = 1000000

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.method = getattr(vamp.ProjMethod, knobs.method)
    constraint_settings.descend_rate = knobs.descend_rate
    constraint_settings.max_iterations = knobs.projection_iterations
    constraint_settings.emit_all_waypoints = knobs.emit_all_waypoints
    constraint_settings.perturbation_scale = knobs.perturbation_scale
    if tolerance is not None:
        constraint_settings.tolerance = float(tolerance)
    if hold_satisfied_rows is not None:
        constraint_settings.hold_satisfied_rows = bool(hold_satisfied_rows)

    seed_settings = vamp.ConstraintSettings()
    seed_settings.method = constraint_settings.method
    seed_settings.descend_rate = constraint_settings.descend_rate
    seed_settings.max_iterations = 200
    if tolerance is not None:
        seed_settings.tolerance = float(tolerance)
    if hold_satisfied_rows is not None:
        seed_settings.hold_satisfied_rows = bool(hold_satisfied_rows)

    end = len(cases) if limit is None else min(len(cases), first + int(limit))
    print(f"{name}: {planner}, problems {first}..{end} of {len(cases)}")

    shared = module.halton() if shared_rng else None
    times_ns, iterations, costs, records = [], [], [], []
    attempted = off_manifold = 0
    shown = None  # (index, case, waypoints) of the first solved problem, for --visualize

    for i in range(first, end):
        case = cases[i]
        e = case.make_environment()
        constraints = case.make_constraints(module)
        sampler = shared if shared_rng else module.halton()
        attempted += 1
        record = {"problem_number": i, "success": False, "solve_time_ns": 0, "iterations": 0,
                  "num_cuboid_obstacles": case.num_obstacles, "path_cost": 0.0, "on_manifold": False}
        records.append(record)

        start = np.array(case.start, dtype=np.float32)
        goal = np.array(case.goal, dtype=np.float32)
        if not raw_seeds:
            try:
                start = module.project(start, constraints, seed_settings)
                goal = module.project(goal, constraints, seed_settings)
            except ValueError:
                print(f"problem {i} ({case.num_obstacles} obstacles): not planned (seed projection failed)")
                continue

        if not (module.validate(start, e) and module.validate(goal, e)):
            print(f"problem {i} ({case.num_obstacles} obstacles): not planned (seed in collision)")
            continue

        result = planner_func(start, goal, e, plan_settings, sampler,
                              constraints=constraints, constraint_settings=constraint_settings)
        record["iterations"] = int(result.iterations)
        if not result.solved:
            print(f"problem {i} ({case.num_obstacles} obstacles): no path after {result.iterations} iterations")
            continue

        simple = module.simplify(result.path, e, simp_settings, sampler,
                                 constraints=constraints, constraint_settings=constraint_settings)
        path = simple.path
        on_manifold = all(module.satisfied(path[k], constraints, constraint_settings) for k in range(len(path)))

        times_ns.append(result.nanoseconds)
        iterations.append(int(result.iterations))
        costs.append(float(path.cost()))
        off_manifold += 0 if on_manifold else 1
        record.update(success=True, solve_time_ns=int(result.nanoseconds), path_cost=float(path.cost()),
                      on_manifold=bool(on_manifold))
        print(f"problem {i} ({case.num_obstacles} obstacles): solved in {result.nanoseconds / 1e6:.3f} ms, "
              f"{result.iterations} iterations, simplified cost {path.cost():.4f}")
        if visualize and shown is None:
            shown = (i, case, path.numpy().copy())

    solved = len(times_ns)
    print("------ Summary ------")
    print(f"Total problems: {attempted}")
    print(f"Successful problems: {solved}")
    print(f"Success rate: {100.0 * solved / attempted if attempted else 0.0:g}%")
    if solved:
        print(f"Average time (ms) for successful problems: {statistics.mean(times_ns) / 1e6:.4f}")
        print(f"Average iterations for successful problems: {int(statistics.mean(iterations))}")
        print(f"Median time (ms) for successful problems: {statistics.median(times_ns) / 1e6:.4f}")
        print(f"Median iterations for successful problems: {int(statistics.median(iterations))}")
        print(f"Mean simplified path cost: {statistics.mean(costs):.4f}")
        print(f"Paths with a waypoint off the manifold: {off_manifold}")

    if results:
        Path(results).write_text(json.dumps(records, indent=1))
        print(f"Wrote {len(records)} records to {results}")

    if visualize:
        if shown is None:
            print("nothing to visualize: no problem was solved")
        else:
            show_problem(module, *shown)


def show_problem(module, index, case, waypoints):
    """Play one solved problem in viser: robot, obstacles, task-space region and attached spheres."""
    from viser_utils import setup_viser_with_robot, add_cuboids, add_tsr_region, run_viewer

    print(f"visualizing problem {index} (the first one solved; pick another with --first and --limit)")
    server, robot = setup_viser_with_robot(VAMP_ROOT / "resources" / "panda", "panda_spherized.urdf")
    server.scene.add_grid("/floor_grid", width=10.0, height=10.0, plane="xy")
    add_cuboids(server, case.cuboids)
    if case.tsr is not None:
        add_tsr_region(server, *case.tsr)

    handles = [server.scene.add_icosphere(f"/attachment/sphere_{k}", radius=radius, position=tuple(position),
                                          color=(220, 60, 60))
               for k, (position, radius) in enumerate(case.attachment)]

    def show(q, _):
        robot.update_cfg(np.asarray(q))
        if handles:
            transform = np.array(module.eefk(np.asarray(q, dtype=np.float32)))
            for handle, (position, _radius) in zip(handles, case.attachment):
                handle.position = tuple(transform[:3, :3] @ np.asarray(position) + transform[:3, 3])

    # Paths are already dense on the manifold (planners emit whole projected chains).
    run_viewer(server, waypoints, show)
