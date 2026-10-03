// Panda draws through a cuboid maze with a pen attached to its hand.
//
// Port of planning_with_constraints/vamp_crrtc_example.cc. The maze is loaded from
// `assets/environments/maze/cuboids.json`; a 9-sphere "pen" is attached to the end-effector; the TSR keeps the
// hand at a fixed height and orientation (free in the plane), as in the original.
//
// By default this runs the original study: range {0.5, 0.75, 1, 0.1, 0.2} x dynamic domain {off} x
// projection method {InnerLM, OuterLM} x descend rate {0.75, 1} x projection iterations
// {5, 10, 25, 50, 100} x insert-all-waypoints {off, on} (200 attempts, one trial each, planner only
// -- the original did not simplify), then prints the tally of successful attempts
// ("------Final Result --------", slowest first). The fastest attempt's path goes to trajectory.txt.
//
//     ./crrtc_maze [--quick] [--trials=N] [--planner=rrtc|aorrtc|grrtstar] [--range=R] [--output=FILE]

#include "common.hh"
#include "json_environment.hh"

#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using TSR = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using Problem = experiments::Problem<Robot, rake>;

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);

    Problem problem;
    problem.name = "panda maze, plane TSR";
    problem.start = {-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294};
    problem.goal = {1.40490, 0.35201, -0.22762, -1.90963, 0.10796, 2.26183, 0.22238};

    if (not experiments::load_cuboids_json(problem.environment, experiments::asset_path("environments/maze/cuboids.json")))
    {
        return 1;
    }

    // Pen: nine 1 cm spheres spaced 2 cm along the end-effector z axis.
    vamp::collision::Attachment<float> pen(Eigen::Transform<float, 3, Eigen::Isometry>::Identity());
    for (auto i = 0U; i < 9; ++i)
    {
        pen.spheres.emplace_back(0.F, 0.F, static_cast<float>(i) * 0.02F, 0.01F);
    }

    pen.end_effector = 0;
    problem.environment.attachments.emplace_back(pen);

    problem.make_constraints = []() -> std::vector<Problem::ConstraintPtr>
    {
        const auto tsr = std::make_shared<TSR>(
            experiments::repeat<Robot::n_eef>(std::array<float, 7>{1, 0, 0, 0, 0, 0, 0}),
            experiments::repeat<Robot::n_eef>(
                std::array<float, 7>{0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783}),
            experiments::repeat<Robot::n_eef>(std::array<float, 6>{-10.01, -10.01, -0.01, -0.01, -0.01, -10.01}),
            experiments::repeat<Robot::n_eef>(std::array<float, 6>{10.01, 10.01, 0.01, 0.01, 0.01, 10.01}));
        return {tsr};
    };

    problem.dd_radius = 1.0;
    problem.simplify_mode = Problem::SimplifyMode::None;
    problem.dump_simplified = false;
    problem.defaults.range = 0.5;
    problem.defaults.projection_iterations = 25;

    problem.sweep.ranges = {0.5, 0.75, 1.0, 0.1, 0.2};
    problem.sweep.dynamic_domain = {false};
    problem.sweep.methods = {vc::ProjMethod::InnerLM, vc::ProjMethod::OuterLM};
    problem.sweep.descend_rates = {0.75, 1.0};
    problem.sweep.projection_iterations = {5, 10, 25, 50, 100};
    problem.sweep.emit_all_waypoints = {false, true};

    return experiments::run(std::move(problem), args);
}
