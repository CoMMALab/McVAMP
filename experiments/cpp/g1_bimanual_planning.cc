// REQUIRES A GENERATED ROBOT: g1_unitree (Unitree G1 humanoid), not shipped with mcvamp.
//
// The G1 holds an object with both hands: a BimanualTaskSpaceConstraint fixes the right hand
// 0.3 m to the side of the left one (identity orientation, +-1 mm), while the left arm swings
// between two configurations. Port of planning_with_constraints/vamp_bimanual_example.cc (the
// file name on the old branch says "bimanual" only; the robot is the G1, environment
// assets/environments/shelf/panda.txt).
//
// Generate vamp/robots/g1_unitree.hh with cricket and point CMake at its include root with
// -DEXPERIMENTS_GENERATED_ROBOT_DIR=<dir containing vamp/robots/g1_unitree.hh> (see README).
// The seeds below are in the old 35-dim layout (6-dof base) and are adapted to the quaternion base
// (36-dim) of a regenerated robot at run time.
//
//
// By default this runs the original study: range {0.5, 0.75, 1, 1.5, 2} x dynamic domain {off, on}
// x projection method {InnerLM, OuterLM} x descend rate {0.75, 1} x projection iterations
// {5, 10, 25, 50, 100} x insert-all-waypoints {off, on} x perturbation scale {0.01, 0.1, 0.2}
// (1200 attempts, one trial each; only the fastest attempt so far is simplified and dumped).
//
//     ./g1_bimanual_planning [--quick] [--trials=N] [--planner=rrtc|aorrtc|grrtstar] [--range=R]
//                            [--output=FILE]

#include "common.hh"

#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/robots/g1_unitree.hh>

using Robot = vamp::robots::G1Unitree;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using Problem = experiments::Problem<Robot, rake>;

// Old layout: x y z roll pitch yaw, 12 leg joints, 3 waist joints, 7 left arm, 7 right arm.
static const std::vector<float> start_old = {
    0.0,    0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.767, -0.16, 0.52, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static const std::vector<float> goal_old = {
    0.0,   0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.702, -0.16, 0.52, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);

    Problem problem;
    problem.name = "g1 bimanual hold";
    problem.start = experiments::adapt_base_layout<Robot>(start_old);
    problem.goal = experiments::adapt_base_layout<Robot>(goal_old);

    if (not experiments::load_cuboids_txt(problem.environment, experiments::asset_path("environments/shelf/panda.txt")))
    {
        return 1;
    }

    problem.make_constraints = []() -> std::vector<Problem::ConstraintPtr>
    {
        return {std::make_shared<vc::BimanualTaskSpaceConstraint<Robot, rake>>(
            std::array<float, 7>{1.0, 0.0, 0.0, 0.0, 0.0, -0.3, 0.0},
            std::array<float, 6>{-0.001, -0.001, -0.001, -0.001, -0.001, -0.001},
            std::array<float, 6>{0.001, 0.001, 0.001, 0.001, 0.001, 0.001})};
    };

    problem.report_perturbation = true;
    problem.defaults.range = 1.0;
    problem.defaults.projection_iterations = 25;

    problem.sweep.ranges = {0.5, 0.75, 1.0, 1.5, 2.0};
    problem.sweep.dynamic_domain = {false, true};
    problem.sweep.methods = {vc::ProjMethod::InnerLM, vc::ProjMethod::OuterLM};
    problem.sweep.descend_rates = {0.75, 1.0};
    problem.sweep.projection_iterations = {5, 10, 25, 50, 100};
    problem.sweep.emit_all_waypoints = {false, true};
    problem.sweep.perturbation_scales = {0.01, 0.1, 0.2};

    return experiments::run(std::move(problem), args);
}
