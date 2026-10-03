"""Panda end effector constrained to a line or a plane, through random cuboid obstacles.

The line/plane problem sets of the constrained benchmarker (assets/problems/benchmark/):

    plane_problems.json   1000 problems, end effector free in a plane (x, y), 100 per obstacle-count bin
    line_problems.json     766 problems, end effector free along a line

Each problem has its own cuboid obstacles (center and half extents), a task-space region (reference frame,
end-effector pose, bounds) and a start and goal. As in the original, the orientation bounds are opened up
(+-1000) unless --keep_orientation is given, and every problem gets a fresh Halton sampler. Settings of the
original: range 1, 200000 iterations, no dynamic domain, OuterLM, descend rate 1, 15 projection iterations,
insert-all waypoints, perturbation 0.1. Counterpart of cpp/benchmark_line_plane.cc.

    python benchmark_line_plane.py [--problems=plane|line] [--file=PATH] [--keep_orientation]
                                   [--range_=R] [--planner=rrtc] [--first=I --limit=N] [--results=FILE]
                                   [--tolerance=T] [--hold_satisfied_rows=False] [--raw_seeds] [--visualize]

--visualize plays the first solved problem of the run in viser (its obstacles, the line or plane, and the path);
use --first=I --limit=1 to look at problem I. Needs viser and yourdfpy.
"""

import json

from fire import Fire

import benchmark_common as bc


def main(problems: str = "plane", file: str = "", keep_orientation: bool = False, range_: float = 1.0,
         planner: str = "rrtc", first: int = 0, limit: int = None, results: str = "", visualize: bool = False,
         **kwargs):
    if not file and problems not in ("plane", "line"):
        raise SystemExit("--problems must be plane or line (or pass --file=PATH)")

    path = file or str(bc.ASSETS / "problems" / "benchmark" / f"{problems}_problems.json")
    with open(path) as f:
        data = json.load(f)

    cases = []
    for item in data:
        lower, upper = list(item["tsr_lower_bound"]), list(item["tsr_upper_bound"])
        if not keep_orientation:
            lower[3:], upper[3:] = [-1000.0] * 3, [1000.0] * 3

        # Cuboid entries are center xyz followed by half extents.
        cuboids = [(c[:3], c[3:]) for c in item["cuboid_obstacles"]]
        reference, pose = item["eef_transforms_ref_frame_w_world"], item["eef_transforms"]

        cases.append(bc.Case(
            start=item["problem_start"],
            goal=item["problem_end"],
            make_environment=lambda cuboids=cuboids: bc.cuboid_environment(cuboids),
            make_constraints=lambda module, r=reference, p=pose, lo=lower, up=upper:
                [module.TaskSpaceConstraint(r, p, lo, up)],
            num_obstacles=len(cuboids),
            cuboids=cuboids,
            tsr=(pose, lower[:3], upper[:3]),
        ))

    knobs = bc.Knobs(range_=range_, projection_iterations=15)
    bc.run_suite(f"line/plane problems ({path})", cases, knobs, planner=planner, first=first,
                 limit=limit, results=results, visualize=visualize, **kwargs)


if __name__ == "__main__":
    Fire(main)
