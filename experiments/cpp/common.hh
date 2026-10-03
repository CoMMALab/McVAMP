#pragma once

// Shared plumbing for the manifold-constrained planning examples.
//
// The old (constrained_planner branch) examples each carried their own copy of a large
// parameter sweep around `CRRTC<...>::solve`. In the new API constrained planning is a
// *local planner* handed to the regular planners:
//
//     ConstraintSet<Robot, rake>          - the constraints + projection settings
//     ConstrainedLocalPlanner<Robot, rake, resolution>(set)
//     RRTC<Robot, rake, resolution>::solve(start, goal, env, settings, rng, lp)
//     simplify<Robot, rake, resolution>(path, env, simplify_settings, rng, lp)
//
// This header wraps that boilerplate (problem description, sweep, reporting, CSV output) so
// that each example is only the problem definition.
//
// Old -> new setting names (vamp::planning::constraint::ConstraintSettings):
//     projection_method          -> method
//     descend_rate               -> descend_rate
//     num_projection_iterations  -> max_iterations
//     insert_all_to_tree         -> emit_all_waypoints
//     std_dev_scaling_factor     -> perturbation_scale

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/planners/aorrtc.hh>
#include <vamp/planning/planners/grrtstar.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/utils.hh>

#ifndef EXPERIMENTS_ASSETS_DIR
#define EXPERIMENTS_ASSETS_DIR "assets"
#endif

namespace experiments
{
    namespace vp = vamp::planning;
    namespace vc = vamp::planning::constraint;

    // ---------------------------------------------------------------------------------
    // Command line: `--flag`, `--key=value`.
    // ---------------------------------------------------------------------------------
    struct Args
    {
        Args(int argc, char **argv)
        {
            for (int i = 1; i < argc; ++i)
            {
                std::string a = argv[i];
                if (a.rfind("--", 0) != 0)
                {
                    throw std::invalid_argument("unexpected argument '" + a + "' (use --key=value or --flag)");
                }

                a = a.substr(2);
                const auto eq = a.find('=');
                if (eq == std::string::npos)
                {
                    values[a] = "true";
                }
                else
                {
                    values[a.substr(0, eq)] = a.substr(eq + 1);
                }
            }
        }

        auto flag(const std::string &key) const -> bool
        {
            return values.count(key) > 0 and values.at(key) != "false";
        }

        auto get(const std::string &key, const std::string &fallback) const -> std::string
        {
            const auto it = values.find(key);
            return (it == values.end()) ? fallback : it->second;
        }

        auto get_float(const std::string &key, float fallback) const -> float
        {
            const auto it = values.find(key);
            return (it == values.end()) ? fallback : std::stof(it->second);
        }

        auto get_size(const std::string &key, std::size_t fallback) const -> std::size_t
        {
            const auto it = values.find(key);
            return (it == values.end()) ? fallback : static_cast<std::size_t>(std::stoul(it->second));
        }

        std::map<std::string, std::string> values;
    };

    // ---------------------------------------------------------------------------------
    // Assets and environments.
    // ---------------------------------------------------------------------------------

    // Assets ship in experiments/assets; EXPERIMENTS_ASSETS_DIR is baked in by CMake and can
    // be overridden at run time with the environment variable of the same name.
    inline auto asset_path(const std::string &name) -> std::string
    {
        const char *override_dir = std::getenv("EXPERIMENTS_ASSETS_DIR");
        return std::string((override_dir != nullptr) ? override_dir : EXPERIMENTS_ASSETS_DIR) + "/" + name;
    }

    // Load axis-aligned cuboids from lines of "x, y, z, dx, dy, dz" (center and *full* extents).
    inline auto load_cuboids_txt(
        vamp::collision::Environment<float> &environment,
        const std::string &path,
        const std::array<float, 3> &shift = {0.F, 0.F, 0.F}) -> bool
    {
        std::ifstream infile(path);
        if (not infile.is_open())
        {
            std::cerr << "Failed to open " << path << std::endl;
            return false;
        }

        std::string line;
        while (std::getline(infile, line))
        {
            std::istringstream iss(line);
            char delim;
            float x, y, z, dx, dy, dz;
            if (not(iss >> x >> delim >> y >> delim >> z >> delim >> dx >> delim >> dy >> delim >> dz))
            {
                std::cerr << "Skipping malformed line: " << line << std::endl;
                continue;
            }

            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(
                {x + shift[0], y + shift[1], z + shift[2]}, {0.F, 0.F, 0.F}, {dx / 2, dy / 2, dz / 2}));
        }

        return true;
    }

    // Load whitespace/comma separated configurations, one per line (used for iiwa start/goal).
    inline auto load_configurations(const std::string &path, std::size_t dimension)
        -> std::vector<std::vector<float>>
    {
        std::ifstream infile(path);
        if (not infile.is_open())
        {
            throw std::runtime_error("Failed to open " + path);
        }

        std::vector<std::vector<float>> result;
        std::string line;
        while (std::getline(infile, line))
        {
            for (auto &c : line)
            {
                if (c == ',')
                {
                    c = ' ';
                }
            }

            std::istringstream iss(line);
            std::vector<float> q;
            float v;
            while (iss >> v)
            {
                q.emplace_back(v);
            }

            if (q.size() == dimension)
            {
                result.emplace_back(std::move(q));
            }
        }

        return result;
    }

    // ---------------------------------------------------------------------------------
    // Helpers for building configurations and constraints.
    // ---------------------------------------------------------------------------------
    template <typename Robot>
    inline auto make_configuration(const std::vector<float> &values) -> typename Robot::Configuration
    {
        if (values.size() != Robot::dimension)
        {
            throw std::invalid_argument(
                "configuration has " + std::to_string(values.size()) + " entries, robot has " +
                std::to_string(Robot::dimension));
        }

        typename Robot::ConfigurationArray array;
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            array[i] = values[i];
        }

        return typename Robot::Configuration(array);
    }

    // Broadcast one bound/transform to every end-effector.
    template <std::size_t n, std::size_t m>
    inline auto repeat(const std::array<float, m> &value) -> std::array<std::array<float, m>, n>
    {
        std::array<std::array<float, m>, n> result;
        result.fill(value);
        return result;
    }

    // Negate a bound vector: symmetric bounds [-b, b].
    template <std::size_t m>
    inline auto negated(std::array<float, m> value) -> std::array<float, m>
    {
        for (auto &v : value)
        {
            v = -v;
        }

        return value;
    }

    // Floating-base robots store the base orientation as a quaternion (x y z qx qy qz qw). The
    // old generated robots used x y z roll pitch yaw; convert URDF-style rpy (R = Rz Ry Rx).
    inline void rpy_to_quaternion_xyzw(float roll, float pitch, float yaw, float *out)
    {
        const float cr = std::cos(roll / 2), sr = std::sin(roll / 2);
        const float cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
        const float cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
        out[0] = sr * cp * cy - cr * sp * sy;
        out[1] = cr * sp * cy + sr * cp * sy;
        out[2] = cr * cp * sy - sr * sp * cy;
        out[3] = cr * cp * cy + sr * sp * sy;
    }

    // Seeds written for the old generated humanoids (6-dof base: x y z roll pitch yaw, then the
    // joints) work with robots regenerated by current cricket (quaternion base: x y z qx qy qz qw,
    // then the joints): pass them through this to adapt them to whatever dimension was compiled in.
    template <typename Robot>
    inline auto adapt_base_layout(const std::vector<float> &q) -> std::vector<float>
    {
        if (q.size() == Robot::dimension)
        {
            return q;
        }

        if (q.size() < 6 or q.size() + 1 != Robot::dimension)
        {
            throw std::invalid_argument(
                "cannot adapt a " + std::to_string(q.size()) + "-dim configuration to a " +
                std::to_string(Robot::dimension) + "-dim robot");
        }

        std::vector<float> result(Robot::dimension);
        std::copy(q.begin(), q.begin() + 3, result.begin());
        rpy_to_quaternion_xyzw(q[3], q[4], q[5], result.data() + 3);
        std::copy(q.begin() + 6, q.end(), result.begin() + 7);
        return result;
    }

    inline auto method_name(vc::ProjMethod method) -> const char *
    {
        switch (method)
        {
            case vc::ProjMethod::InnerLM:
                return "InnerLM";
            case vc::ProjMethod::OuterLM:
                return "OuterLM";
            case vc::ProjMethod::GradDesc:
                return "GradDesc";
        }

        return "?";
    }

    // ---------------------------------------------------------------------------------
    // Planning problem + sweep.
    // ---------------------------------------------------------------------------------

    // One point in the planner/projection parameter space.
    struct Knobs
    {
        float range = 1.F;
        bool dynamic_domain = false;
        vc::ProjMethod method = vc::ProjMethod::InnerLM;
        float descend_rate = 1.F;
        std::size_t projection_iterations = 15;
        bool emit_all_waypoints = true;
        float perturbation_scale = 0.1F;
    };

    // Cartesian product of parameter values (what the old examples looped over).
    struct Grid
    {
        std::vector<float> ranges;
        std::vector<bool> dynamic_domain;
        std::vector<vc::ProjMethod> methods;
        std::vector<float> descend_rates;
        std::vector<std::size_t> projection_iterations;
        std::vector<bool> emit_all_waypoints;
        std::vector<float> perturbation_scales;

        auto expand(const Knobs &defaults) const -> std::vector<Knobs>
        {
            const auto or_default = [](const auto &values, const auto &fallback)
            { return values.empty() ? std::decay_t<decltype(values)>{fallback} : values; };

            std::vector<Knobs> result;
            for (const auto range : or_default(ranges, defaults.range))
            for (const bool dd : or_default(dynamic_domain, defaults.dynamic_domain))
            for (const auto method : or_default(methods, defaults.method))
            for (const auto rate : or_default(descend_rates, defaults.descend_rate))
            for (const auto iterations : or_default(projection_iterations, defaults.projection_iterations))
            for (const bool emit : or_default(emit_all_waypoints, defaults.emit_all_waypoints))
            for (const auto scale : or_default(perturbation_scales, defaults.perturbation_scale))
            {
                result.push_back({range, dd, method, rate, iterations, emit, scale});
            }

            return result;
        }
    };

    template <typename Robot, std::size_t rake>
    struct Problem
    {
        using ConstraintPtr = std::shared_ptr<const vc::Constraint<Robot, rake>>;

        std::string name;
        std::vector<float> start;
        std::vector<float> goal;
        vamp::collision::Environment<float> environment;

        // Constraints cache per-evaluation state, so every attempt builds a fresh set.
        std::function<std::vector<ConstraintPtr>()> make_constraints;

        // Radius of the RRT-Connect dynamic domain (only used with dynamic_domain).
        float dd_radius = 4.F;
        std::size_t max_iterations = 100000;

        // Default knobs of a single run (--quick, and the whole study of the single-run examples),
        // and the grid of the original ablation study (the default behavior). An empty grid means
        // the original example was a single run.
        Knobs defaults;
        Grid sweep;

        // What the original example did with each solved attempt.
        enum class SimplifyMode
        {
            None,           // planner result only
            OnImprovement,  // simplify only when it is the fastest so far (and dump that)
            Always,         // simplify every solved attempt and tally simplification statistics
        };

        SimplifyMode simplify_mode = SimplifyMode::OnImprovement;
        bool dump_simplified = true;        // trajectory file holds the simplified (else raw) path
        bool dump_trajectory = true;        // write the fastest attempt's path
        bool interpolate_dump = false;      // interpolate the dumped path to the collision resolution
        bool report_perturbation = false;   // the study loops over / reports the perturbation scale
        bool print_before_solve = true;     // "range, dd method rate" line before every solve
        bool shared_rng = false;            // one sampler for the whole study (else one per attempt)
        std::string spheres_dump;           // write the environment spheres here (x,y,z,r per row)
        std::function<void()> epilogue;     // extra output after the final summary

        // Iteration budget for the one-off projection of start/goal onto the manifold.
        std::size_t seed_iterations = 200;
        bool raw_seeds = false;  // --raw_seeds: feed start/goal unprojected, as the old examples did

        // ConstraintSettings fields outside the ablation grid, set from the command line in run()
        // (--tolerance, --connect_slack, --reached_radius2, --endpoint_tolerance2,
        // --hold_satisfied_rows); unset keeps the library default.
        std::optional<float> tolerance;
        std::optional<float> connect_slack;
        std::optional<float> reached_radius2;
        std::optional<float> endpoint_tolerance2;
        bool hold_satisfied_rows = vc::ConstraintSettings{}.hold_satisfied_rows;  // library default (true)
    };

    struct Attempt
    {
        Knobs knobs;
        bool solved = false;
        std::size_t planning_ns = 0;
        std::size_t iterations = 0;
        std::size_t path_states = 0;
        bool attempted = false;
        std::size_t simplify_ns = 0;
        std::size_t simplified_states = 0;
        float simplified_cost = 0.F;
        bool on_manifold = false;
    };

    template <typename Robot, std::size_t rake>
    using ConstrainedLP = vc::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;

    // Dispatch to the named planner. All three take the local planner as their last argument.
    template <typename Robot, std::size_t rake, typename LP>
    inline auto solve_with_planner(
        const std::string &planner,
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const vamp::collision::Environment<vamp::FloatVector<rake>> &environment,
        const Knobs &knobs,
        std::size_t max_iterations,
        float dd_radius,
        typename vamp::rng::RNG<Robot>::Ptr rng,
        const LP &lp) -> vp::PlanningResult<Robot>
    {
        constexpr auto resolution = Robot::resolution;
        if (planner == "rrtc")
        {
            vp::RRTCSettings settings;
            settings.range = knobs.range;
            settings.dynamic_domain = knobs.dynamic_domain;
            settings.radius = dd_radius;
            settings.max_iterations = max_iterations;
            settings.max_samples = 1000000;  // the old RRTCSettings default; max_iterations alone capped emit-all runs at ~max_iterations/rake extensions
            return vp::RRTC<Robot, rake, resolution>::solve(start, goal, environment, settings, rng, lp);
        }

        if (planner == "aorrtc")
        {
            vp::AORRTCSettings settings;
            settings.rrtc.range = knobs.range;
            settings.rrtc.dynamic_domain = knobs.dynamic_domain;
            settings.rrtc.radius = dd_radius;
            settings.max_iterations = max_iterations;
            settings.max_samples = 1000000;  // the old RRTCSettings default; max_iterations alone capped emit-all runs at ~max_iterations/rake extensions
            return vp::AORRTC<Robot, rake, resolution>::solve(start, goal, environment, settings, rng, lp);
        }

        if (planner == "grrtstar")
        {
            vp::GRRTStarSettings settings;
            settings.range = knobs.range;
            settings.dynamic_domain = knobs.dynamic_domain;
            settings.dd_radius = dd_radius;
            settings.max_iterations = max_iterations;
            settings.max_samples = 1000000;  // the old RRTCSettings default; max_iterations alone capped emit-all runs at ~max_iterations/rake extensions
            return vp::GRRTStar<Robot, rake, resolution>::solve(start, goal, environment, settings, rng, lp);
        }

        throw std::invalid_argument("unknown planner '" + planner + "' (rrtc, aorrtc, grrtstar)");
    }

    template <typename Robot, std::size_t rake>
    inline void write_csv(const std::string &filename, const vp::Path<Robot> &path)
    {
        std::ofstream outfile(filename);
        outfile << std::setprecision(8);
        for (const auto &config : path)
        {
            const auto array = config.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                outfile << ((i == 0) ? "" : ",") << array[i];
            }

            outfile << "\n";
        }
    }

    // Run one attempt. `path_out` receives the raw path and `simplified_out` the simplified one
    // (empty unless the problem simplifies this attempt: `want_simplify`).
    template <typename Robot, std::size_t rake>
    inline auto attempt(
        const Problem<Robot, rake> &problem,
        const Knobs &knobs,
        const std::string &planner,
        const vamp::collision::Environment<vamp::FloatVector<rake>> &env_v,
        typename vamp::rng::RNG<Robot>::Ptr rng,
        const std::function<bool(std::size_t)> &want_simplify,
        vp::Path<Robot> &path_out,
        vp::Path<Robot> &simplified_out) -> Attempt
    {
        Attempt result;
        result.knobs = knobs;

        // Old -> new: projection_method -> method, descend_rate -> descend_rate,
        // num_projection_iterations -> max_iterations, insert_all_to_tree -> emit_all_waypoints,
        // std_dev_scaling_factor -> perturbation_scale.
        vc::ConstraintSettings settings;
        settings.method = knobs.method;
        settings.descend_rate = knobs.descend_rate;
        settings.max_iterations = knobs.projection_iterations;
        settings.emit_all_waypoints = knobs.emit_all_waypoints;
        settings.perturbation_scale = knobs.perturbation_scale;
        if (problem.tolerance) settings.tolerance = *problem.tolerance;
        if (problem.connect_slack) settings.connect_slack = *problem.connect_slack;
        if (problem.reached_radius2) settings.reached_radius2 = *problem.reached_radius2;
        if (problem.endpoint_tolerance2) settings.endpoint_tolerance2 = *problem.endpoint_tolerance2;
        settings.hold_satisfied_rows = problem.hold_satisfied_rows;

        // The local planner owns its constraint set (single-threaded: one set per attempt).
        const ConstrainedLP<Robot, rake> lp(
            vc::ConstraintSet<Robot, rake>(problem.make_constraints(), settings));

        // Strict start/goal policy of the new API: seeds must lie on the manifold, so project
        // them first with a generous iteration budget (not the per-sample planning budget).
        // (The old examples fed the seeds to the planner unprojected.)
        vc::ConstraintSettings seed_settings = settings;
        seed_settings.max_iterations = problem.seed_iterations;
        const vc::ConstraintSet<Robot, rake> seed_set(problem.make_constraints(), seed_settings);

        auto start = make_configuration<Robot>(problem.start);
        auto goal = make_configuration<Robot>(problem.goal);
        if (not problem.raw_seeds and (not seed_set.project(start) or not seed_set.project(goal)))
        {
            std::cout << "  start/goal projection onto the manifold did not converge" << std::endl;
            return result;
        }

        for (const auto &[name, q] : {std::pair{"start", start}, std::pair{"goal", goal}})
        {
            if (not vp::validate_motion<Robot, rake, 1>(q, q, env_v))
            {
                std::cout << "  projected " << name << " configuration is in collision" << std::endl;
                return result;
            }
        }

        auto planned = solve_with_planner<Robot, rake>(
            planner, start, goal, env_v, knobs, problem.max_iterations, problem.dd_radius, rng, lp);

        result.attempted = true;
        result.planning_ns = planned.nanoseconds;
        result.iterations = planned.iterations;
        result.path_states = planned.path.size();
        if (planned.path.empty())
        {
            return result;
        }

        result.solved = true;
        result.on_manifold = std::all_of(
            planned.path.begin(), planned.path.end(), [&lp](const auto &q) { return lp.satisfied(q); });
        path_out = planned.path;

        if (want_simplify(planned.nanoseconds))
        {
            vp::SimplifySettings simplify_settings;
            auto simplified = vp::simplify<Robot, rake, Robot::resolution>(
                planned.path, env_v, simplify_settings, rng, lp);

            result.simplify_ns = simplified.nanoseconds;
            result.simplified_states = simplified.path.size();
            result.simplified_cost = simplified.path.cost();
            result.on_manifold = std::all_of(
                simplified.path.begin(),
                simplified.path.end(),
                [&lp](const auto &q) { return lp.satisfied(q); });
            simplified_out = std::move(simplified.path);
        }

        return result;
    }

    // Entry point shared by the planning examples. By default it runs the *whole* study of the
    // original example: every combination of the original value lists, nested in the original
    // order (range, dynamic domain, projection method, descend rate, projection iterations,
    // insert-all, perturbation), printing per-attempt lines and the original final summary.
    //   --quick            one configuration (the problem's defaults) instead of the study
    //   --trials=N         repeat every configuration N times (the originals ran 1)
    //   --planner=NAME     rrtc (default), aorrtc or grrtstar
    //   --range=R          override the planner range (with --quick, or for every grid point)
    //   --output=FILE      CSV for the fastest attempt (default trajectory.txt)
    //   --max_iterations=N planner iteration budget
    //   --raw_seeds        feed start/goal unprojected, as the old examples did
    //   --tolerance=T      squared-violation convergence threshold of projection (default 1e-6;
    //                      the old branches used 1e-4 on myfork)
    //   --hold_satisfied_rows[=false]   keep (default) or zero the Jacobian rows of already-satisfied components
    //   --connect_slack=X --reached_radius2=X --endpoint_tolerance2=X   other ConstraintSettings
    template <typename Robot, std::size_t rake>
    inline auto run(Problem<Robot, rake> problem, const Args &args) -> int
    {
        using Mode = typename Problem<Robot, rake>::SimplifyMode;

        const auto planner = args.get("planner", "rrtc");
        const auto output = args.get("output", "trajectory.txt");
        const auto trials = args.get_size("trials", 1);
        problem.max_iterations = args.get_size("max_iterations", problem.max_iterations);
        problem.raw_seeds = args.flag("raw_seeds");
        const auto optional_float = [&args](const std::string &key) -> std::optional<float>
        {
            return (args.values.count(key) > 0) ? std::optional<float>(args.get_float(key, 0.F)) : std::nullopt;
        };
        problem.tolerance = optional_float("tolerance");
        problem.connect_slack = optional_float("connect_slack");
        problem.reached_radius2 = optional_float("reached_radius2");
        problem.endpoint_tolerance2 = optional_float("endpoint_tolerance2");
        if (args.values.count("hold_satisfied_rows") > 0)
        {
            problem.hold_satisfied_rows = args.flag("hold_satisfied_rows");
        }
        if (args.values.count("range") > 0)
        {
            problem.defaults.range = args.get_float("range", problem.defaults.range);
            problem.sweep.ranges.clear();
        }

        problem.environment.sort();
        const vamp::collision::Environment<vamp::FloatVector<rake>> env_v(problem.environment);

        if (not problem.spheres_dump.empty())
        {
            std::ofstream sphere_file(problem.spheres_dump);
            for (const auto &s : problem.environment.spheres)
            {
                sphere_file << s.x << "," << s.y << "," << s.z << "," << s.r << "\n";
            }
        }

        const auto grid = (args.flag("quick")) ? Grid{}.expand(problem.defaults) :
                                                 problem.sweep.expand(problem.defaults);

        std::cout << problem.name << ": " << planner << ", " << grid.size() << " configuration(s) x "
                  << trials << " trial(s)" << std::endl;
        {
            const vc::ConstraintSettings defaults;
            std::cout << "constraint settings (grid knobs vary per attempt): tolerance="
                      << problem.tolerance.value_or(defaults.tolerance)
                      << " connect_slack=" << problem.connect_slack.value_or(defaults.connect_slack)
                      << " reached_radius2=" << problem.reached_radius2.value_or(defaults.reached_radius2)
                      << " endpoint_tolerance2=" << problem.endpoint_tolerance2.value_or(defaults.endpoint_tolerance2)
                      << " hold_satisfied_rows=" << problem.hold_satisfied_rows << " raw_seeds=" << problem.raw_seeds
                      << std::endl;
        }

        std::vector<Attempt> solved;
        vp::Path<Robot> best_path;
        bool have_best = false;
        std::size_t best_ns = 0;
        auto shared_rng = std::make_shared<vamp::rng::Halton<Robot>>();

        for (const auto &knobs : grid)
        {
            for (std::size_t trial = 0; trial < trials; ++trial)
            {
                if (problem.print_before_solve)
                {
                    std::cout << knobs.range << ", " << knobs.dynamic_domain << " " << static_cast<int>(knobs.method)
                              << " " << knobs.descend_rate << " " << std::flush;
                }

                typename vamp::rng::RNG<Robot>::Ptr rng =
                    (problem.shared_rng) ? typename vamp::rng::RNG<Robot>::Ptr(shared_rng) :
                                           typename vamp::rng::RNG<Robot>::Ptr(
                                               std::make_shared<vamp::rng::Halton<Robot>>());

                const auto want_simplify = [&](std::size_t planning_ns)
                {
                    return problem.simplify_mode == Mode::Always or
                           (problem.simplify_mode == Mode::OnImprovement and
                            (not have_best or planning_ns < best_ns));
                };

                vp::Path<Robot> path, simplified;
                const auto a = attempt<Robot, rake>(problem, knobs, planner, env_v, rng, want_simplify, path, simplified);
                if (not a.solved)
                {
                    std::cout << "No path found with settings: " << knobs.range << ", " << knobs.dynamic_domain
                              << " " << static_cast<int>(knobs.method) << " " << knobs.descend_rate << " "
                              << knobs.projection_iterations << " " << knobs.perturbation_scale << " "
                              << knobs.emit_all_waypoints << std::endl;
                    continue;
                }

                if (not have_best or a.planning_ns < best_ns)
                {
                    have_best = true;
                    best_ns = a.planning_ns;
                    best_path = (problem.dump_simplified and not simplified.empty()) ? simplified : path;
                    std::cout << "\nPrinting Result!! " << path.size() << std::endl;
                }
                else
                {
                    std::cout << "solved in " << a.planning_ns / 1e6 << " ms" << std::endl;
                }

                solved.push_back(a);
            }
        }

        // The original tally: successful attempts, slowest first.
        std::sort(
            solved.begin(),
            solved.end(),
            [](const auto &l, const auto &r) { return l.planning_ns > r.planning_ns; });

        const bool with_simplify = problem.simplify_mode == Mode::Always;
        std::cout << "------Final Result --------" << std::endl;
        std::cout << "# " << ((with_simplify) ? "" : "") << "range, dynamic_domain, method, descend_rate, proj_iters, "
                  << ((problem.report_perturbation) ? "perturbation, " : "")
                  << "insert_all(emit_all), planning_ms, planner_iterations, path_states"
                  << ((with_simplify) ? ", simplify_ms, simplified_states, total_ms, simplified_cost" : "")
                  << ", on_manifold" << std::endl;
        for (const auto &a : solved)
        {
            const auto &k = a.knobs;
            std::cout << k.range << ", " << k.dynamic_domain << ", " << static_cast<int>(k.method) << ", "
                      << k.descend_rate << ", " << k.projection_iterations << ", ";
            if (problem.report_perturbation)
            {
                std::cout << k.perturbation_scale << ", ";
            }

            std::cout << k.emit_all_waypoints << ", " << a.planning_ns / 1e6 << ", " << a.iterations << ", "
                      << a.path_states;
            if (with_simplify)
            {
                std::cout << ", " << a.simplify_ns / 1e6 << ", " << a.simplified_states << ", "
                          << (a.planning_ns + a.simplify_ns) / 1e6 << ", " << a.simplified_cost;
            }

            std::cout << ", " << a.on_manifold << std::endl;
        }

        std::cout << "---------------------------" << std::endl;
        std::cout << "Solved " << solved.size() << "/" << grid.size() * trials << " attempts" << std::endl;
        if (problem.epilogue)
        {
            problem.epilogue();
        }

        if (solved.empty())
        {
            std::cout << "No configuration solved the problem." << std::endl;
            return 1;
        }

        if (problem.dump_trajectory)
        {
            if (problem.interpolate_dump)
            {
                best_path.interpolate_to_resolution(Robot::resolution);
            }

            write_csv<Robot, rake>(output, best_path);
            std::cout << "Wrote " << best_path.size() << " states of the fastest attempt to " << output
                      << std::endl;
        }

        return 0;
    }
}  // namespace experiments
