// Whole-body constrained planning on the Digit humanoid: carry a box from a shelf to a rack.
//
// Port of planning_with_constraints/digit_example.cc. Constraint stack (each is a
// vamp::planning::constraint::Constraint, combined by the ConstraintSet inside the
// ConstrainedLocalPlanner):
//   - TaskSpaceConstraint        both feet pinned to their stance poses (arms left free)
//   - CoMConstraint              center of mass stays over the support polygon between the feet
//   - ClosedLoopConstraint       the two four-bar linkage rods in the legs stay closed
//   - BimanualTaskSpaceConstraint (transport only) the right hand holds a fixed transform relative
//                                to the left hand, as if both grasp one rigid box
//
// Differences from the old branch: Digit is now a floating-base robot with a quaternion base
// (31-dim: x y z qx qy qz qw + 24 joints, was 30-dim with roll/pitch/yaw), the seeds below are
// the ones from vamp/scripts/digit_example.py, and the shelf scene from that script is used
// (the old C++ ran in an empty scene: pass --empty to reproduce that).
//
//
// By default this runs the original study: range {0.75, 0.5} x dynamic domain {off, on} x
// projection method {InnerLM} x descend rate {0.75, 1} x projection iterations {10, 25} x
// insert-all-waypoints {off, on} (32 attempts, one trial each). Only the fastest attempt so far is
// simplified; its path is interpolated to the collision resolution (as in the original; note that
// interpolation leaves the manifold) and written to trajectory.txt.
//
//     ./digit_transport [--quick] [--trials=N] [--no_transport] [--empty]
//                       [--planner=rrtc|aorrtc|grrtstar] [--range=R] [--output=FILE]

#include "common.hh"

#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/manifold/closed_loop_constraint.hh>
#include <vamp/planning/constraints/manifold/com_constraint.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/robots/digit.hh>

using Robot = vamp::robots::Digit;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
namespace vc = vamp::planning::constraint;
using Problem = experiments::Problem<Robot, rake>;

// Free-flyer layout: base translation (x y z), base quaternion (qx qy qz qw), then the 24
// actuated joints (left leg, left arm, right leg, right arm).
static const std::vector<float> box_top_shelf_pickup = {
    0.00779, -0.02074, 0.00461, -0.00213, -0.00449, -0.00263, 0.99998,
    0.38120, 0.00154, 0.28644, 0.32323, -0.00734, -0.29696, 0.07485, 0.01125,
    0.08865, -0.26348, -0.00645, 0.11539,
    -0.37495, -0.00330, -0.29074, -0.31914, 0.01103, 0.28118, -0.13812, 0.00576,
    0.16191, 0.27674, 0.06755, -0.09470};
static const std::vector<float> rack_2 = {
    0.00927, -0.01219, -0.47283, -0.00698, -0.01481, -0.00458, 0.99986,
    0.40005, 0.01408, -0.23717, -0.84704, -0.02429, 0.90190, -0.48186, 0.04543,
    -0.17059, -0.36363, 0.06204, 1.09005,
    -0.36164, -0.00267, 0.22962, 0.84538, 0.02448, -0.90900, 0.40052, 0.03680,
    -0.29646, 0.37895, -0.15478, -1.16821};

// Stance poses (qw qx qy qz x y z) of the toe-roll frames.
static constexpr std::array<float, 7> identity = {1, 0, 0, 0, 0, 0, 0};
static constexpr std::array<float, 7> left_foot = {0.59, 0.38, 0.4, 0.59, -0.02070, 0.06015, -0.95335};
static constexpr std::array<float, 7> right_foot = {0.61, -0.36, 0.35, -0.61, -0.02228, -0.11609, -0.94832};

static constexpr std::array<float, 6> free_bound = {10, 10, 10, 10, 10, 10};
static constexpr std::array<float, 6> pinned_bound = {0.001, 0.001, 0.001, 0.1, 0.1, 0.1};

// Pose of the right hand in the left-hand frame while both grasp the box: position and roll
// are held, the other rotations are left free.
static constexpr std::array<float, 7> box_grasp = {0.1, 0.99, 0.0, 0.04, 0.01462, -0.03530, -0.36356};
static constexpr std::array<float, 6> box_grasp_lower = {-0.001, -0.001, -0.001, -0.1, -10.1, -10.1};
static constexpr std::array<float, 6> box_grasp_upper = {0.001, 0.001, 0.001, 0.1, 10.1, 10.1};

// Support polygon (counterclockwise, xy in the world frame, which is the frame of the CoM the generated Digit
// computes, recipe digit.json). It is the region {0.01..-0.05} x {-0.01..0.03} of the foot-relative CoM shifted by the
// stance's toe midpoint (-0.02149, -0.02797).
static const std::vector<std::array<float, 2>> support_polygon = {
    {0.01 - 0.02149F, -0.01 - 0.02797F},
    {0.01 - 0.02149F, 0.03 - 0.02797F},
    {-0.05 - 0.02149F, 0.03 - 0.02797F},
    {-0.05 - 0.02149F, -0.01 - 0.02797F}};

// Shelf scene (center xyz, full extents) from the G1 humanoid shelf, dropped by digit's ground
// offset and nudged back in x so the seeds clear the boards and back wall.
static const std::vector<std::array<float, 6>> shelf = {
    {0.45, 0.0, 0.4, 0.385, 1.0, 0.015},
    {0.45, 0.0, 0.79, 0.385, 1.0, 0.015},
    {0.45, 0.0, 1.2, 0.385, 1.0, 0.015},
    {0.6, 0.0, 1.0, 0.03, 1.0, 2.0},
    {0.0, 0.4, 0.225, 0.4, 0.3, 0.45},
};
static constexpr std::array<float, 3> shelf_shift = {0.15, 0.0, -0.95};

auto main(int argc, char **argv) -> int
{
    const experiments::Args args(argc, argv);
    const bool transport = not args.flag("no_transport");

    Problem problem;
    problem.name = std::string("digit box ") + ((transport) ? "transport" : "walk-free reach") + ", feet+CoM+loops";
    problem.start = box_top_shelf_pickup;
    problem.goal = rack_2;

    if (not args.flag("empty"))
    {
        for (const auto &box : shelf)
        {
            problem.environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(
                {box[0] + shelf_shift[0], box[1] + shelf_shift[1], box[2] + shelf_shift[2]},
                {0.F, 0.F, 0.F},
                {box[3] / 2, box[4] / 2, box[5] / 2}));
        }
    }

    problem.make_constraints = [transport]() -> std::vector<Problem::ConstraintPtr>
    {
        // End-effector order of the generated robot: left hand, right hand, left toe, right toe.
        // rTe is the identity for all four; wTr pins the feet only (hands stay free).
        const auto feet = std::make_shared<vc::TaskSpaceConstraint<Robot, rake>>(
            std::array<std::array<float, 7>, 4>{identity, identity, identity, identity},
            std::array<std::array<float, 7>, 4>{identity, identity, left_foot, right_foot},
            std::array<std::array<float, 6>, 4>{
                experiments::negated(free_bound),
                experiments::negated(free_bound),
                experiments::negated(pinned_bound),
                experiments::negated(pinned_bound)},
            std::array<std::array<float, 6>, 4>{free_bound, free_bound, pinned_bound, pinned_bound});

        std::vector<Problem::ConstraintPtr> constraints = {
            feet,
            std::make_shared<vc::CoMConstraint<Robot, rake>>(support_polygon),
            std::make_shared<vc::ClosedLoopConstraint<Robot, rake>>()};

        if (transport)
        {
            constraints.emplace_back(std::make_shared<vc::BimanualTaskSpaceConstraint<Robot, rake>>(
                box_grasp, box_grasp_lower, box_grasp_upper));
        }

        return constraints;
    };

    problem.interpolate_dump = true;
    problem.dd_radius = 10.0;
    problem.max_iterations = 100000;
    problem.defaults.range = 0.75;
    problem.defaults.projection_iterations = 25;

    problem.sweep.ranges = {0.75, 0.5};
    problem.sweep.dynamic_domain = {false, true};
    problem.sweep.methods = {vc::ProjMethod::InnerLM};
    problem.sweep.descend_rates = {0.75, 1.0};
    problem.sweep.projection_iterations = {10, 25};
    problem.sweep.emit_all_waypoints = {false, true};

    return experiments::run(std::move(problem), args);
}
