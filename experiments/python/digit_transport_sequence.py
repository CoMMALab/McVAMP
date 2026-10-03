"""Digit whole-body constrained plan sequence: reach the box, then transport it.

Port of the old scripts/digit_test.py. Legs of the sequence:

  1. standing -> box_top_shelf_pickup  (feet pinned + CoM in support polygon + closed
     leg loops; hands free)
  2. box_top_shelf_pickup -> rack_2    (adds the bimanual box-hold constraint, and the
     box's collision spheres are attached to the left hand)
  3. rack_2 -> box_top_shelf_pickup    (transport back)

Each leg is planned with a regular planner given `constraints=[...]` (rrtc / aorrtc /
grrtstar), after projecting the leg's endpoints onto its constraint manifold. The
simplified path of each leg is written to `<output_prefix>_<i>.csv` if requested (31 columns:
xyz, quaternion xyzw, 24 joints) and can be replayed with vamp/scripts/digit_example.py's
viewer code or meshcat_viz.py.

Run:
    python experiments/python/digit_transport_sequence.py [--planner rrtc] [--output_prefix leg] [--visualize]

--visualize plays the three legs one after another in viser (shelf, support polygon, and the box in the left hand
during the transport legs). Needs viser and yourdfpy.
"""

import os
import time
from pathlib import Path

import numpy as np
import vamp
from fire import Fire

VAMP_ROOT = Path(os.environ.get("VAMP_ROOT", Path(__file__).resolve().parents[2] / "vamp"))
DIGIT_DIR = VAMP_ROOT / "resources" / "digit"
DIGIT_URDF = "digit_model_spherized.urdf"  # Fixed-base variant: viser drives the base frame.

# Free-flyer q layout: base translation (x y z), base quaternion (qx qy qz qw), then the
# 24 actuated joints (left leg, left arm, right leg, right arm).
CONFIGS = {
    "standing": [
        0.028748, -0.0189052, -0.0145299, 0.00483, 0.00413, -0.00106, 0.99998,
        0.397209, -0.0082068, 0.284002, 0.290042, -0.0213535, -0.235481, -0.031921, 0.00127043,
        -0.0782953, 1.04159, 0.0284093, -0.0123198,
        -0.358284, -0.013361, -0.278441, -0.268447, 0.0177607, 0.212721, -0.0692397, -0.000343739,
        0.068962, -1.23829, 0.0369306, -0.00862385],
    "box_top_shelf_pickup": [
        0.00779, -0.02074, 0.00461, -0.00213, -0.00449, -0.00263, 0.99998,
        0.38120, 0.00154, 0.28644, 0.32323, -0.00734, -0.29696, 0.07485, 0.01125,
        0.08865, -0.26348, -0.00645, 0.11539,
        -0.37495, -0.00330, -0.29074, -0.31914, 0.01103, 0.28118, -0.13812, 0.00576,
        0.16191, 0.27674, 0.06755, -0.09470],
    "rack_2": [
        0.00927, -0.01219, -0.47283, -0.00698, -0.01481, -0.00458, 0.99986,
        0.40005, 0.01408, -0.23717, -0.84704, -0.02429, 0.90190, -0.48186, 0.04543,
        -0.17059, -0.36363, 0.06204, 1.09005,
        -0.36164, -0.00267, 0.22962, 0.84538, 0.02448, -0.90900, 0.40052, 0.03680,
        -0.29646, 0.37895, -0.15478, -1.16821],
}

IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
LEFT_FOOT = [0.59, 0.38, 0.4, 0.59, -0.02070, 0.06015, -0.95335]
RIGHT_FOOT = [0.61, -0.36, 0.35, -0.61, -0.02228, -0.11609, -0.94832]
FREE = [10.0] * 6
PINNED = [0.001, 0.001, 0.001, 0.1, 0.1, 0.1]

BOX_GRASP = [0.1, 0.99, 0.0, 0.04, 0.01462, -0.03530, -0.36356]
BOX_GRASP_LOWER = [-0.001, -0.001, -0.001, -0.1, -10.1, -10.1]
BOX_GRASP_UPPER = [0.001, 0.001, 0.001, 0.1, 10.1, 10.1]

# The CoM the Digit header computes is in the world frame (cricket recipe digit.json), so the support polygon is the
# region {0.01 .. -0.05} x {-0.01 .. 0.03} about the feet midpoint, shifted by that midpoint (-0.02149, -0.02797).
FEET_MIDPOINT = [0.5 * (LEFT_FOOT[4] + RIGHT_FOOT[4]), 0.5 * (LEFT_FOOT[5] + RIGHT_FOOT[5])]
SUPPORT_POLYGON = [(x + FEET_MIDPOINT[0], y + FEET_MIDPOINT[1])
                   for x, y in ((0.01, -0.01), (0.01, 0.03), (-0.05, 0.03), (-0.05, -0.01))]

# Shelf scene: (center xyz, full extents), shifted by digit's ground offset.
SHELF_CUBOIDS = [
    ([0.45, 0.0, 0.4], [0.385, 1.0, 0.015]),
    ([0.45, 0.0, 0.79], [0.385, 1.0, 0.015]),
    ([0.45, 0.0, 1.2], [0.385, 1.0, 0.015]),
    ([0.6, 0.0, 1.0], [0.03, 1.0, 2.0]),
    ([0.0, 0.4, 0.225], [0.4, 0.3, 0.45]),
]
SHELF_SHIFT = [0.15, 0.0, -0.95]

# Box carried in the left hand: a 4x4 grid of small spheres in the hand frame.
BOX_SPHERES = [[x, y, z] for x in (-0.05, -0.02) for (y, z) in
               ((0.0, -0.04), (-0.02, -0.07), (-0.04, -0.11), (-0.06, -0.13),
                (0.05, -0.04), (0.03, -0.07), (0.01, -0.11), (-0.01, -0.13))]

LEGS = [
    ("standing", "box_top_shelf_pickup", False),
    ("box_top_shelf_pickup", "rack_2", True),
    ("rack_2", "box_top_shelf_pickup", True),
]


def make_constraints(module, transport):
    # Module end-effector order: left arm, right arm, left toe roll, right toe roll.
    feet = module.TaskSpaceConstraint(
        [IDENTITY] * 4,
        [IDENTITY, IDENTITY, LEFT_FOOT, RIGHT_FOOT],
        [[-b for b in FREE], [-b for b in FREE], [-b for b in PINNED], [-b for b in PINNED]],
        [FREE, FREE, PINNED, PINNED])
    constraints = [feet, module.CoMConstraint(SUPPORT_POLYGON), module.ClosedLoopConstraint()]
    if transport:
        constraints.append(
            module.BimanualTaskSpaceConstraint(BOX_GRASP, BOX_GRASP_LOWER, BOX_GRASP_UPPER))
    return constraints


def make_environment(transport, attachment_radius):
    e = vamp.Environment()
    for center, extents in SHELF_CUBOIDS:
        shifted = [c + s for c, s in zip(center, SHELF_SHIFT)]
        e.add_cuboid(vamp.Cuboid(shifted, [0.0, 0.0, 0.0], [d / 2.0 for d in extents]))
    if transport:
        attachment = vamp.Attachment(np.identity(4), 0)  # Left hand.
        attachment.add_spheres([vamp.Sphere(p, attachment_radius) for p in BOX_SPHERES])
        e.attach(attachment)
    return e


def visualize_legs(module, legs, attachment_radius):
    """Play the legs [(waypoints, transport)] in viser; base pose goes to the /robot frame."""
    import yourdfpy
    from viser_utils import setup_viser_with_robot, add_cuboids, run_viewer

    server, robot = setup_viser_with_robot(DIGIT_DIR, DIGIT_URDF)
    base_frame = server.scene.add_frame("/robot", show_axes=False)
    add_cuboids(
        server,
        [([c + s for c, s in zip(center, SHELF_SHIFT)], [d / 2.0 for d in extents])
         for center, extents in SHELF_CUBOIDS],
        color=(160, 110, 60))

    # The support polygon is in world axes; draw it at the height of the feet.
    foot_z = 0.5 * (LEFT_FOOT[6] + RIGHT_FOOT[6])
    corners = [np.array([vx, vy, foot_z]) for vx, vy in SUPPORT_POLYGON]
    server.scene.add_line_segments(
        "/constraint/support_polygon",
        points=np.array([[corners[i], corners[(i + 1) % len(corners)]] for i in range(len(corners))]),
        colors=(255, 140, 0),
        line_width=4.0)

    # yourdfpy's actuated joint order can differ from the module's; its first seven names are the floating base.
    urdf = yourdfpy.URDF.load(str(DIGIT_DIR / DIGIT_URDF), load_meshes=False, build_collision_scene_graph=False)
    names = list(module.joint_names())[7:]
    remap = [names.index(n) for n in urdf.actuated_joint_names]

    box = [server.scene.add_icosphere(f"/box/sphere_{k}", radius=attachment_radius, position=tuple(p),
                                      color=(220, 60, 60)) for k, p in enumerate(BOX_SPHERES)]
    waypoints = np.concatenate([w for w, _ in legs])
    carrying = np.concatenate([np.full(len(w), t) for w, t in legs])

    def show(q, index):
        q = np.asarray(q)
        base_frame.position = q[0:3]
        base_frame.wxyz = np.array([q[6], q[3], q[4], q[5]])
        robot.update_cfg(q[7:][remap])
        transform = np.array(module.eefk(q.astype(np.float32)))  # Left hand.
        for handle, position in zip(box, BOX_SPHERES):
            handle.visible = bool(carrying[index])
            handle.position = tuple(transform[:3, :3] @ np.asarray(position) + transform[:3, 3])

    run_viewer(server, waypoints, show)


def main(
    planner: str = "rrtc",  # One of rrtc, aorrtc, grrtstar.
    range_: float = 0.75,
    projection_iterations: int = 25,
    attachment_radius: float = 0.02,
    output_prefix: str = "",  # If set, write each leg's path to <prefix>_<i>.csv.
    visualize: bool = False,  # Play the legs in viser after planning.
    **kwargs,
):
    if "digit" not in vamp.robots:
        raise SystemExit(
            "This vamp build does not include the 'digit' robot (it has: " + ", ".join(vamp.robots) + ").\n"
            "Rebuild vamp with it, for example:\n"
            "    pip install . -C cmake.define.VAMP_ROBOTS=\"ur5;panda;fetch;digit\"\n"
            "See the README, section 6.")
    kwargs.setdefault("max_iterations", 100000)
    module, planner_func, plan_settings, simp_settings = (
        vamp.configure_robot_and_planner_with_kwargs("digit", planner, **kwargs))
    rrtc_settings = plan_settings.rrtc if planner == "aorrtc" else plan_settings
    rrtc_settings.range = range_
    if planner != "grrtstar":
        rrtc_settings.radius = 10.0

    constraint_settings = vamp.ConstraintSettings()
    constraint_settings.max_iterations = projection_iterations
    seed_settings = vamp.ConstraintSettings()
    seed_settings.max_iterations = 100
    seed_settings.descend_rate = 0.75

    sampler = module.halton()
    legs = []
    for i, (start_name, goal_name, transport) in enumerate(LEGS):
        constraints = make_constraints(module, transport)
        e = make_environment(transport, attachment_radius)
        projected = []
        for name in (start_name, goal_name):
            try:
                projected.append(module.project(np.array(CONFIGS[name], dtype=np.float32), constraints, seed_settings))
            except ValueError as error:
                raise RuntimeError(f"leg {i} ({start_name} -> {goal_name}): could not project '{name}' onto the "
                                   f"constraints ({error}); check that the support polygon matches the CoM frame "
                                   f"of the generated Digit header") from error
        start, goal = projected
        for name, q in (("start", start), ("goal", goal)):
            if not module.validate(q, e):
                raise RuntimeError(f"leg {i}: projected {name} configuration is in collision")

        t0 = time.perf_counter()
        result = planner_func(
            start, goal, e, plan_settings, sampler,
            constraints=constraints, constraint_settings=constraint_settings)
        elapsed = (time.perf_counter() - t0) * 1e3
        if not result.solved:
            raise RuntimeError(f"leg {i} ({start_name} -> {goal_name}): planning failed")

        path = module.simplify(
            result.path, e, simp_settings, sampler,
            constraints=constraints, constraint_settings=constraint_settings).path
        on_manifold = all(module.satisfied(path[k], constraints, constraint_settings) for k in range(len(path)))
        print(f"leg {i} {start_name} -> {goal_name} ({'transport' if transport else 'reach'}): "
              f"{elapsed:.1f} ms, {len(result.path)} -> {len(path)} states, on manifold: {on_manifold}")
        if output_prefix:
            np.savetxt(f"{output_prefix}_{i}.csv", path.numpy(), delimiter=",")
        legs.append((path.numpy().copy(), transport))

    if visualize:
        visualize_legs(module, legs, attachment_radius)


if __name__ == "__main__":
    Fire(main)
