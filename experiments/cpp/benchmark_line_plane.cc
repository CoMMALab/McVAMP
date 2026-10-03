// Panda end effector constrained to a line or a plane, through random cuboid obstacles: the line/plane
// problem sets of the constrained benchmarker.
//
// Each problem comes with its own obstacles (a list of cuboids), a task-space region (reference frame,
// end-effector pose, bounds) and a start and goal. As in the original, the orientation bounds are opened
// up (+-1000), so only the position is constrained, and one attempt is made per problem with a fresh
// Halton sampler. Problem sets (assets/problems/benchmark/):
//
//     plane_problems.json   1000 problems, end effector free in a plane (x, y), 100 per obstacle-count bin
//     line_problems.json     766 problems, end effector free along a line
//
// Settings of the original: range 1, 200000 iterations, no dynamic domain (radius 1), OuterLM, descend rate 1,
// 15 projection iterations, insert-all waypoints, perturbation scale 0.1. Unlike the original, the result
// is simplified with the local planner's own projection settings (the new simplify takes no separate
// method, rate or iteration count).
//
//     ./benchmark_line_plane [--problems=plane|line] [--file=PATH] [--keep_orientation] [--range=R]
//                            [--planner=...] [--first=I --limit=N] [--results=FILE]
//
// Shared flags (planner, first, limit, results, tolerance, ...): see benchmark_suite.hh.

#include "benchmark_suite.hh"

#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/robots/panda.hh>

#include <fstream>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using TSR = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using Suite = experiments::benchmark::Suite<Robot, rake>;
using Case = experiments::benchmark::Case<Robot, rake>;

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);

    const auto set = args.get("problems", "plane");
    if (set != "plane" and set != "line" and args.get("file", "").empty())
    {
        std::cerr << "--problems must be plane or line (or pass --file=PATH)" << std::endl;
        return 1;
    }

    const auto path =
        args.get("file", experiments::asset_path("problems/benchmark/" + set + "_problems.json"));
    std::ifstream ifs(path);
    if (not ifs.is_open())
    {
        std::cerr << "Failed to open " << path << std::endl;
        return 1;
    }

    nlohmann::json j;
    ifs >> j;
    if (not j.is_array())
    {
        std::cerr << "Expected a top-level JSON array in " << path << std::endl;
        return 1;
    }

    // As in the original: ignore the orientation bounds unless asked to keep them.
    const bool keep_orientation = args.flag("keep_orientation");

    Suite suite;
    suite.name = "line/plane problems (" + path + ")";
    suite.knobs.range = args.get_float("range", 1.F);
    suite.knobs.dynamic_domain = false;
    suite.knobs.method = vc::ProjMethod::OuterLM;
    suite.knobs.descend_rate = 1.F;
    suite.knobs.projection_iterations = 15;
    suite.knobs.emit_all_waypoints = true;
    suite.knobs.perturbation_scale = 0.1F;
    suite.max_iterations = 200000;
    suite.dd_radius = 1.F;

    for (const auto &item : j)
    {
        Case c;
        c.start = item.at("problem_start").get<std::vector<float>>();
        c.goal = item.at("problem_end").get<std::vector<float>>();

        // Cuboids: center xyz and half extents.
        for (const auto &cuboid : item.at("cuboid_obstacles").get<std::vector<std::array<float, 6>>>())
        {
            c.environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(
                {cuboid[0], cuboid[1], cuboid[2]}, {0.F, 0.F, 0.F}, {cuboid[3], cuboid[4], cuboid[5]}));
            ++c.num_obstacles;
        }

        auto lower = item.at("tsr_lower_bound").get<std::array<float, 6>>();
        auto upper = item.at("tsr_upper_bound").get<std::array<float, 6>>();
        if (not keep_orientation)
        {
            for (auto i = 3U; i < 6; ++i)
            {
                lower[i] = -1000.F;
                upper[i] = 1000.F;
            }
        }

        // First argument is the reference frame in the world, second the end-effector pose, as in the
        // original constraint constructor.
        const auto reference = item.at("eef_transforms_ref_frame_w_world").get<std::array<float, 7>>();
        const auto pose = item.at("eef_transforms").get<std::array<float, 7>>();

        c.make_constraints = [reference, pose, lower, upper]() -> std::vector<Case::ConstraintPtr>
        {
            return {std::make_shared<TSR>(
                experiments::repeat<Robot::n_eef>(reference),
                experiments::repeat<Robot::n_eef>(pose),
                experiments::repeat<Robot::n_eef>(lower),
                experiments::repeat<Robot::n_eef>(upper))};
        };

        suite.cases.emplace_back(std::move(c));
    }

    return experiments::benchmark::run(std::move(suite), args);
}
