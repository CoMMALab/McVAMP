#pragma once

// Runner for the benchmark problem sets (assets/problems/benchmark/): many independent problems, one
// fixed set of planner and projection settings per problem, as in the original constrained
// benchmarker. Unlike experiments::run (one problem, a grid of settings), this runs one attempt per
// problem and reports the solve rate and the time and iteration statistics of the solved ones.
//
// Command line (shared by every benchmark built on it):
//   --planner=NAME        rrtc (default), aorrtc or grrtstar
//   --first=I --limit=N   run problems I, I+1, ... (default: from 0, all of them)
//   --results=FILE        write one JSON record per problem (success, time, iterations, cost)
//   --max_iterations=N    planner iteration budget per problem
//   --raw_seeds           give start and goal to the planner without projecting them first
//   --tolerance=T --connect_slack=X --reached_radius2=X --endpoint_tolerance2=X
//                         ConstraintSettings, as in the other examples
//   --hold_satisfied_rows[=false]   keep (default) or zero the Jacobian rows of satisfied components

#include "common.hh"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>

namespace experiments::benchmark
{
    template <typename Robot, std::size_t rake>
    struct Case
    {
        using ConstraintPtr = std::shared_ptr<const vc::Constraint<Robot, rake>>;

        std::vector<float> start;
        std::vector<float> goal;
        vamp::collision::Environment<float> environment;
        std::function<std::vector<ConstraintPtr>()> make_constraints;
        std::size_t num_obstacles = 0;
    };

    template <typename Robot, std::size_t rake>
    struct Suite
    {
        std::string name;
        std::vector<Case<Robot, rake>> cases;
        Knobs knobs;
        std::size_t max_iterations = 200000;
        float dd_radius = 1.F;

        // One sampler for the whole suite (the maze benchmarks) or a fresh one per problem.
        bool shared_rng = false;
    };

    template <typename T>
    inline auto median(std::vector<T> values) -> T
    {
        std::sort(values.begin(), values.end());
        return values[values.size() / 2];
    }

    template <typename Robot, std::size_t rake>
    inline auto run(Suite<Robot, rake> suite, const Args &args) -> int
    {
        const auto planner = args.get("planner", "rrtc");
        const auto first = args.get_size("first", 0);
        const auto limit = args.get_size("limit", suite.cases.size());
        const auto results_path = args.get("results", "");

        const auto optional_float = [&args](const std::string &key) -> std::optional<float>
        {
            return (args.values.count(key) > 0) ? std::optional<float>(args.get_float(key, 0.F)) : std::nullopt;
        };

        const auto end = std::min(suite.cases.size(), first + limit);
        std::cout << suite.name << ": " << planner << ", problems " << first << ".." << end << " of "
                  << suite.cases.size() << std::endl;

        auto shared = std::make_shared<vamp::rng::Halton<Robot>>();

        std::size_t attempted = 0;
        std::size_t solved = 0;
        std::size_t off_manifold = 0;
        std::vector<std::size_t> times_ns;
        std::vector<std::size_t> iterations;
        std::vector<float> costs;
        nlohmann::json records = nlohmann::json::array();

        for (auto i = first; i < end; ++i)
        {
            auto &c = suite.cases[i];

            Problem<Robot, rake> problem;
            problem.start = c.start;
            problem.goal = c.goal;
            problem.make_constraints = c.make_constraints;
            problem.max_iterations = args.get_size("max_iterations", suite.max_iterations);
            problem.dd_radius = suite.dd_radius;
            problem.raw_seeds = args.flag("raw_seeds");
            problem.tolerance = optional_float("tolerance");
            problem.connect_slack = optional_float("connect_slack");
            problem.reached_radius2 = optional_float("reached_radius2");
            problem.endpoint_tolerance2 = optional_float("endpoint_tolerance2");
            if (args.values.count("hold_satisfied_rows") > 0)
            {
                problem.hold_satisfied_rows = args.flag("hold_satisfied_rows");
            }

            c.environment.sort();
            const vamp::collision::Environment<vamp::FloatVector<rake>> env_v(c.environment);
            const auto rng = (suite.shared_rng) ? shared : std::make_shared<vamp::rng::Halton<Robot>>();

            vp::Path<Robot> path;
            vp::Path<Robot> simplified;
            const auto result = attempt<Robot, rake>(
                problem, suite.knobs, planner, env_v, rng, [](std::size_t) { return true; }, path, simplified);

            ++attempted;
            std::cout << "problem " << i << " (" << c.num_obstacles << " obstacles): ";
            if (not result.attempted)
            {
                std::cout << "not planned (seed projection failed or seed in collision)" << std::endl;
            }
            else if (not result.solved)
            {
                std::cout << "no path after " << result.iterations << " iterations" << std::endl;
            }
            else
            {
                ++solved;
                times_ns.push_back(result.planning_ns);
                iterations.push_back(result.iterations);
                costs.push_back(result.simplified_cost);
                off_manifold += (result.on_manifold) ? 0 : 1;
                std::cout << "solved in " << result.planning_ns / 1e6 << " ms, " << result.iterations
                          << " iterations, simplified cost " << result.simplified_cost << std::endl;
            }

            records.push_back(
                {{"problem_number", i},
                 {"success", result.solved},
                 {"solve_time_ns", (result.solved) ? result.planning_ns : 0},
                 {"iterations", result.iterations},
                 {"num_cuboid_obstacles", c.num_obstacles},
                 {"path_cost", (result.solved) ? result.simplified_cost : 0.F},
                 {"on_manifold", result.on_manifold}});
        }

        std::cout << "------ Summary ------" << std::endl
                  << "Total problems: " << attempted << std::endl
                  << "Successful problems: " << solved << std::endl
                  << "Success rate: " << ((attempted > 0) ? 100.0 * solved / attempted : 0.0) << "%" << std::endl;

        if (solved > 0)
        {
            const auto mean_ns = std::accumulate(times_ns.begin(), times_ns.end(), std::size_t{0}) / solved;
            const auto mean_it = std::accumulate(iterations.begin(), iterations.end(), std::size_t{0}) / solved;
            std::cout << "Average time (ms) for successful problems: " << mean_ns / 1e6 << std::endl
                      << "Average iterations for successful problems: " << mean_it << std::endl
                      << "Median time (ms) for successful problems: " << median(times_ns) / 1e6 << std::endl
                      << "Median iterations for successful problems: " << median(iterations) << std::endl
                      << "Mean simplified path cost: "
                      << std::accumulate(costs.begin(), costs.end(), 0.F) / static_cast<float>(solved) << std::endl
                      << "Paths with a waypoint off the manifold: " << off_manifold << std::endl;
        }

        if (not results_path.empty())
        {
            std::ofstream(results_path) << records.dump(1) << std::endl;
            std::cout << "Wrote " << records.size() << " records to " << results_path << std::endl;
        }

        return 0;
    }
}  // namespace experiments::benchmark
