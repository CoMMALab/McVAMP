// Panda end-effector constrained to a line through a sphere cage.
//
// Port of planning_with_constraints/vamp_crrtc_spheres.cc. The task-space region (TSR) holds
// the end-effector at x = 0.3486, z = 0.2399 (+-1 mm) and at the reference orientation
// (+-0.01 rad), leaving only translation along the reference y axis free, so the arm has to
// slide along a line while threading a cage of spheres.
//
// By default this runs the original study: range {0.5, 1, 1.5, 2} x dynamic domain {off, on} x
// projection method {InnerLM, OuterLM} x descend rate {0.75, 1} x projection iterations
// {5, 10, 25, 50, 100} x insert-all-waypoints {off, on} x perturbation scale {0.01, 0.1, 0.2}
// (960 attempts, one trial each, planner + simplification timed), then prints the tally of
// successful attempts ("------Final Result --------", slowest first) and the end-effector poses of
// start and goal. The cage is also written to spheres.txt.
//
//     ./crrtc_spheres [--quick] [--trials=N] [--planner=rrtc|aorrtc|grrtstar] [--range=R]

#include "common.hh"

#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using TSR = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using Problem = experiments::Problem<Robot, rake>;

// Sphere centers of the cage (common radius below).
static const std::vector<std::array<float, 3>> cage = {
    {0.56, 0, 0.450},
    {0.1, 0, 0.7},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
};
static constexpr float radius = 0.15;

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);

    Problem problem;
    problem.name = "panda sphere cage, line TSR";
    problem.start = {1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891};
    problem.goal = {-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24};
    for (const auto &sphere : cage)
    {
        problem.environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }

    problem.make_constraints = []() -> std::vector<Problem::ConstraintPtr>
    {
        // rTe: identity offset frame on the end-effector; wTr: reference frame in the world.
        const auto tsr = std::make_shared<TSR>(
            experiments::repeat<Robot::n_eef>(std::array<float, 7>{1, 0, 0, 0, 0, 0, 0}),
            experiments::repeat<Robot::n_eef>(std::array<float, 7>{0, 1, 0, 0, 0.3486, 0.647752, 0.2399}),
            experiments::repeat<Robot::n_eef>(std::array<float, 6>{-0.001, -10.01, -0.001, -0.01, -0.01, -0.01}),
            experiments::repeat<Robot::n_eef>(std::array<float, 6>{0.001, 10.01, 0.001, 0.01, 0.01, 0.01}));
        return {tsr};
    };

    problem.dd_radius = 1.0;
    problem.defaults.range = 1.0;
    problem.defaults.projection_iterations = 25;

    problem.simplify_mode = Problem::SimplifyMode::Always;
    problem.dump_trajectory = false;  // the original only tallied statistics
    problem.report_perturbation = true;
    problem.print_before_solve = false;
    problem.spheres_dump = "spheres.txt";
    problem.epilogue = [start = problem.start, goal = problem.goal]()
    {
        Robot::ConfigurationArray s, g;
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            s[i] = start[i];
            g[i] = goal[i];
        }

        std::cout << "\n" << Robot::eefk(g).matrix() << "\n\n" << Robot::eefk(s).matrix() << std::endl;
    };

    // The grid of the original example.
    problem.sweep.ranges = {0.5, 1.0, 1.5, 2.0};
    problem.sweep.dynamic_domain = {false, true};
    problem.sweep.methods = {vc::ProjMethod::InnerLM, vc::ProjMethod::OuterLM};
    problem.sweep.descend_rates = {0.75, 1.0};
    problem.sweep.projection_iterations = {5, 10, 25, 50, 100};
    problem.sweep.emit_all_waypoints = {false, true};
    problem.sweep.perturbation_scales = {0.01, 0.1, 0.2};

    return experiments::run(std::move(problem), args);
}
