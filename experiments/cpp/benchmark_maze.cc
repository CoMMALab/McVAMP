// Panda draws through the cuboid maze: the maze problems of the constrained benchmarker.
//
// assets/problems/benchmark/maze_pen_problems.json holds 100 start/goal configurations in the maze scene
// (assets/environments/maze/cuboids.json). A 9-sphere pen is attached to the hand and the hand is held at a
// fixed height and orientation, free in the plane (the crrtc_maze problem). Settings of the original: range 0.4,
// 20 projection iterations, 200000 iterations, no dynamic domain (radius 1), OuterLM, descend rate 1, insert-all waypoints,
// perturbation scale 0.1, and one sampler shared by all problems, as in the original. (The original also had a
// ten-problem "marker" set; it needs a Panda variant with a marker end effector, which is not part of this tree.)
//
//     ./benchmark_maze [--range=R] [--planner=...] [--first=I --limit=N] [--results=FILE]
//
// Shared flags (planner, first, limit, results, tolerance, ...): see benchmark_suite.hh.

#include "benchmark_suite.hh"
#include "json_environment.hh"

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

    std::ifstream ifs(experiments::asset_path("problems/benchmark/maze_pen_problems.json"));
    if (not ifs.is_open())
    {
        std::cerr << "Failed to open the maze problem set" << std::endl;
        return 1;
    }

    nlohmann::json j;
    ifs >> j;

    vamp::collision::Environment<float> maze;
    if (not experiments::load_cuboids_json(maze, experiments::asset_path("environments/maze/cuboids.json")))
    {
        return 1;
    }

    // Pen: nine 1 cm spheres spaced 2 cm along the end-effector z axis.
    {
        vamp::collision::Attachment<float> attachment(Eigen::Transform<float, 3, Eigen::Isometry>::Identity());
        for (auto i = 0U; i < 9; ++i)
        {
            attachment.spheres.emplace_back(0.F, 0.F, static_cast<float>(i) * 0.02F, 0.01F);
        }

        attachment.end_effector = 0;
        maze.attachments.emplace_back(attachment);
    }

    const std::array<float, 7> reference = {1, 0, 0, 0, 0, 0, 0};
    const std::array<float, 7> pose = {0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783};
    const std::array<float, 6> lower = {-10.01, -10.01, -0.01, -0.01, -0.01, -10.01};
    const std::array<float, 6> upper = {10.01, 10.01, 0.01, 0.01, 0.01, 10.01};

    Suite suite;
    suite.name = "maze pen problems";
    suite.knobs.range = args.get_float("range", 0.4F);
    suite.knobs.dynamic_domain = false;
    suite.knobs.method = vc::ProjMethod::OuterLM;
    suite.knobs.descend_rate = 1.F;
    suite.knobs.projection_iterations = 20;
    suite.knobs.emit_all_waypoints = true;
    suite.knobs.perturbation_scale = 0.1F;
    suite.max_iterations = 200000;
    suite.dd_radius = 1.F;
    suite.shared_rng = true;

    for (const auto &item : j)
    {
        Case c;
        c.start = item.at("problem_start").get<std::vector<float>>();
        c.goal = item.at("problem_end").get<std::vector<float>>();
        // Copy the shapes (assigning a whole Environment does not compile: CAPT's allocator has no operator==).
        c.environment.cuboids = maze.cuboids;
        c.environment.z_aligned_cuboids = maze.z_aligned_cuboids;
        c.environment.attachments = maze.attachments;
        c.num_obstacles = maze.cuboids.size();
        c.make_constraints = [=]() -> std::vector<Case::ConstraintPtr>
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
