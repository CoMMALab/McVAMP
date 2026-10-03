"""Panda draws through the cuboid maze: the maze problems of the constrained benchmarker.

assets/problems/benchmark/maze_pen_problems.json holds 100 start/goal configurations in the maze scene
(assets/environments/maze/cuboids.json). A 9-sphere pen is attached to the hand and the hand is held at a fixed
height and orientation, free in the plane (the crrtc_maze problem). Settings of the original: range 0.4,
20 projection iterations, 200000 iterations, no dynamic domain, OuterLM, descend rate 1, insert-all waypoints,
perturbation 0.1, one sampler shared by all problems. Counterpart of cpp/benchmark_maze.cc.

    python benchmark_maze.py [--range_=R] [--planner=rrtc] [--first=I --limit=N] [--results=FILE]
                             [--tolerance=T] [--hold_satisfied_rows=False] [--raw_seeds] [--visualize]

--visualize plays the first solved problem of the run in viser (maze, constraint plane, pen and path); use
--first=I --limit=1 to look at problem I. Needs viser and yourdfpy.
"""

import json

import numpy as np
import vamp
from fire import Fire

import benchmark_common as bc

REFERENCE = [1, 0, 0, 0, 0, 0, 0]
POSE = [0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783]
LOWER = [-10.01, -10.01, -0.01, -0.01, -0.01, -10.01]
UPPER = [10.01, 10.01, 0.01, 0.01, 0.01, 10.01]


def maze_cuboids():
    with open(bc.ASSETS / "environments" / "maze" / "cuboids.json") as f:
        data = json.load(f)
    return [([o["x"], o["y"], o["z"]], [o["dx"] / 2, o["dy"] / 2, o["dz"] / 2]) for o in data]


# Nine 1 cm spheres spaced 2 cm along the end-effector z axis.
PEN = [([0.0, 0.0, 0.02 * i], 0.01) for i in range(9)]


def pen():
    attachment = vamp.Attachment(np.identity(4), 0)
    attachment.add_spheres([vamp.Sphere(position, radius) for position, radius in PEN])
    return attachment


def main(range_: float = 0.4, planner: str = "rrtc", first: int = 0, limit: int = None, results: str = "",
         visualize: bool = False, **kwargs):
    with open(bc.ASSETS / "problems" / "benchmark" / "maze_pen_problems.json") as f:
        data = json.load(f)

    cuboids = maze_cuboids()
    cases = [bc.Case(
        start=item["problem_start"],
        goal=item["problem_end"],
        make_environment=lambda: bc.cuboid_environment(cuboids, pen()),
        make_constraints=lambda module: [module.TaskSpaceConstraint(REFERENCE, POSE, LOWER, UPPER)],
        num_obstacles=len(cuboids),
        cuboids=cuboids,
        tsr=(POSE, LOWER[:3], UPPER[:3]),
        attachment=PEN,
    ) for item in data]

    knobs = bc.Knobs(range_=range_, projection_iterations=20)
    bc.run_suite("maze pen problems", cases, knobs, planner=planner, shared_rng=True, first=first,
                 limit=limit, results=results, visualize=visualize, **kwargs)


if __name__ == "__main__":
    Fire(main)
