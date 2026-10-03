// Constrained planning with OMPL's ProjectedStateSpace where OMPL drives everything: the
// constraint is exposed to OMPL through its Eigen interface (function / project / distance) and
// OMPL's default constrained motion validation (discrete geodesics) checks edges; VAMP supplies
// the manifold projection and SIMD collision checking as the state validity checker.
//
// Port of planning_with_constraints/constrained_ompl_statespace_integration.cc (--scene=maze,
// default) and of scripts/cpp/ompl_regular_integration.cc (--scene=cage). The latter differed in
// the scene, the TSR bounds, InnerLM instead of OuterLM and a placeholder Jacobian override; that
// override is dropped here (OMPL then differentiates the constraint function numerically).
//
// Needs OMPL (see README).
//
//     ./ompl_constrained_statespace [--scene=maze|cage] [--method=InnerLM|OuterLM|GradDesc]
//                                   [--time=SECONDS] [--range=R] [--output=FILE]

#include "json_environment.hh"
#include "ompl_common.hh"

using namespace ompl_demo;

auto main(int argc, char **argv) -> int
{
    const experiments::Args args(argc, argv);
    const auto scene = args.get("scene", "maze");
    if (scene != "maze" and scene != "cage")
    {
        std::cerr << "--scene must be maze or cage" << std::endl;
        return 1;
    }

    const bool maze = scene == "maze";

    EnvironmentInput environment;
    std::array<float, dimension> start, goal;
    std::array<float, 7> reference;
    std::array<float, 6> lower, upper;
    if (maze)
    {
        // Cuboid maze + a 9-sphere pen on the end-effector, plane TSR (free x, y and yaw).
        if (not experiments::load_cuboids_json(environment, experiments::asset_path("environments/maze/cuboids.json")))
        {
            return 1;
        }

        vamp::collision::Attachment<float> pen(Eigen::Transform<float, 3, Eigen::Isometry>::Identity());
        for (auto i = 0U; i < 9; ++i)
        {
            pen.spheres.emplace_back(0.F, 0.F, static_cast<float>(i) * 0.02F, 0.01F);
        }

        environment.attachments.emplace_back(pen);
        start = {-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294};
        goal = {1.40490, 0.35201, -0.22762, -1.90963, 0.10796, 2.26183, 0.22238};
        reference = {0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783};
        lower = {-10.01, -10.01, -0.01, -0.01, -0.01, -10.01};
        upper = {10.01, 10.01, 0.01, 0.01, 0.01, 10.01};
    }
    else
    {
        // Sphere cage, line TSR (free along the reference y axis only).
        sphere_cage(
            environment,
            {{0.56, 0, 0.450}, {0.1, 0, 0.7}, {0, 0.55, 0.25}, {-0.55, 0, 0.25}, {-0.35, -0.35, 0.25},
             {0, -0.55, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8}, {-0.35, 0.35, 0.8}, {-0.55, 0, 0.8},
             {-0.35, -0.35, 0.8}, {0, -0.55, 0.8}, {0.35, -0.35, 0.8}},
            0.15);
        start = {1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100};
        goal = {-1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000};
        reference = {0, 1, 0, 0, 0.3486, 0.647752, 0.2399};
        lower = {-0.01, -10.01, -0.01, -0.01, -0.01, -0.01};
        upper = {0.01, 10.01, 0.01, 0.01, 0.01, 0.01};
    }

    environment.sort();
    const EnvironmentVector env_v(environment);

    // Projection settings of the old code: 25 iterations, step 1.0, OuterLM (maze) / InnerLM (cage).
    vc::ConstraintSettings settings;
    const auto method = args.get("method", (maze) ? "OuterLM" : "InnerLM");
    settings.method = (method == "OuterLM") ? vc::ProjMethod::OuterLM :
                      (method == "GradDesc") ? vc::ProjMethod::GradDesc :
                                               vc::ProjMethod::InnerLM;
    settings.max_iterations = 25;
    settings.emit_all_waypoints = false;

    auto constraint = std::make_shared<VampConstraint>(
        ConstraintSet(make_tsr_constraints(reference, lower, upper), settings), 1e-4);
    const ConstraintSet validity_set(make_tsr_constraints(reference, lower, upper), settings);

    auto space = std::make_shared<ob::ProjectedStateSpace>(make_ambient_space(), constraint);
    constraint->set_space(space);
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(space);
    csi->setStateValidityChecker(std::make_shared<VampStateValidator>(csi, env_v, validity_set));
    csi->setup();

    Eigen::VectorXd sv(dimension), gv(dimension);
    for (auto i = 0U; i < dimension; ++i)
    {
        sv[i] = start[i];
        gv[i] = goal[i];
    }

    ob::ScopedState<> start_state(space), goal_state(space);
    start_state->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
    goal_state->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);

    auto pdef = std::make_shared<ob::ProblemDefinition>(csi);
    pdef->setStartAndGoalStates(start_state, goal_state);
    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
    objective->setCostThreshold(objective->infiniteCost());
    pdef->setOptimizationObjective(objective);

    auto planner = std::make_shared<og::RRTConnect>(csi);
    planner->setProblemDefinition(pdef);
    planner->setRange(args.get_float("range", 1.0));
    planner->setup();

    const auto start_time = std::chrono::steady_clock::now();
    const ob::PlannerStatus solved = planner->ob::Planner::solve(args.get_float("time", 100.F));
    const auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    if (not solved)
    {
        std::cout << "No solution found (" << constraint->num_failed_projections << " failed projections)"
                  << std::endl;
        return 1;
    }

    auto *path = pdef->getSolutionPath()->as<og::PathGeometric>();
    std::cout << "Found solution in " << nanoseconds / 1e6 << " ms: " << path->getStateCount()
              << " states, length " << path->length() << std::endl;
    write_path(args.get("output", "trajectory.txt"), *path, space.get());
    return 0;
}
