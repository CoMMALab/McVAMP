// REQUIRES A GENERATED ROBOT: bimanual_iiwa (two KUKA iiwa arms), not shipped with mcvamp.
//
// Two iiwas hold one rigid object: BimanualTaskSpaceConstraint keeps the right end-effector at a
// fixed pose in the left end-effector frame (+-1 mm) while moving through the shelf in
// assets/environments/shelf/iiwa.txt. Start/goal are the first two lines of assets/problems/bimanual_iiwa_start_goal.txt.
// Port of planning_with_constraints/vamp_bimanual_example_iiwa.cc.
//
// Generate vamp/robots/bimanual_iiwa.hh with cricket and point CMake at its include root with
// -DEXPERIMENTS_GENERATED_ROBOT_DIR=<dir containing vamp/robots/bimanual_iiwa.hh> (see README).
//
//
// By default this runs the original study: range {1, 1.25} x dynamic domain {off, on} x projection
// method {InnerLM, OuterLM} x descend rate {0.5, 1} x projection iterations {5, 10, 25} x
// insert-all-waypoints {off, on} x perturbation scale {0.01, 0.1, 0.2} (288 attempts, one trial
// each, one sampler shared by all attempts as in the original, every solved attempt simplified).
//
//     ./iiwa_bimanual_planning [--quick] [--trials=N] [--planner=rrtc|aorrtc|grrtstar] [--range=R]
//                              [--output=FILE]

#include "common.hh"

#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/robots/bimanual_iiwa.hh>

using Robot = vamp::robots::BimanualIiwa;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using Problem = experiments::Problem<Robot, rake>;

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);

    Problem problem;
    problem.name = "bimanual iiwa hold, shelf";

    const auto seeds = experiments::load_configurations(experiments::asset_path("problems/bimanual_iiwa_start_goal.txt"), Robot::dimension);
    if (seeds.size() < 2)
    {
        std::cerr << "assets/problems/bimanual_iiwa_start_goal.txt must contain a start and a goal line of " << Robot::dimension
                  << " values" << std::endl;
        return 1;
    }

    problem.start = seeds[0];
    problem.goal = seeds[1];

    if (not experiments::load_cuboids_txt(problem.environment, experiments::asset_path("environments/shelf/iiwa.txt")))
    {
        return 1;
    }

    problem.make_constraints = []() -> std::vector<Problem::ConstraintPtr>
    {
        return {std::make_shared<vc::BimanualTaskSpaceConstraint<Robot, rake>>(
            std::array<float, 7>{-0.0005, 0.927184, -0.364607, 0.0009, 5.96046e-08, 0, 0.6},
            std::array<float, 6>{-0.001, -0.001, -0.001, -0.001, -0.001, -0.001},
            std::array<float, 6>{0.001, 0.001, 0.001, 0.001, 0.001, 0.001})};
    };

    problem.simplify_mode = Problem::SimplifyMode::Always;
    problem.shared_rng = true;
    problem.report_perturbation = true;
    problem.print_before_solve = false;
    problem.dd_radius = 4.0;
    problem.defaults.range = 1.0;
    problem.defaults.projection_iterations = 25;

    problem.sweep.ranges = {1.0, 1.25};
    problem.sweep.dynamic_domain = {false, true};
    problem.sweep.methods = {vc::ProjMethod::InnerLM, vc::ProjMethod::OuterLM};
    problem.sweep.descend_rates = {0.5, 1.0};
    problem.sweep.projection_iterations = {5, 10, 25};
    problem.sweep.emit_all_waypoints = {false, true};
    problem.sweep.perturbation_scales = {0.01, 0.1, 0.2};

    return experiments::run(std::move(problem), args);
}
