// Constrained planning with OMPL's ProjectedStateSpace + RRT-Connect, VAMP as manifold and
// collision checker.
//
// Port of planning_with_constraints/constrained_ompl_integration.cc. The Panda's end-effector is
// held at a fixed height/roll/pitch (a TaskSpaceConstraint) while it moves through a sphere
// cage. OMPL's constrained state space calls back into VAMP for projection
// (ConstraintSet::project); by default edges are validated by VAMP's ConstrainedLocalPlanner
// (edge traced on the manifold and SIMD collision-checked), or with --ompl_motion by OMPL's own
// discrete geodesic (the old "state_only" mode).
//
// Needs OMPL (see README); the OMPL fork's virtual Constraint::project(State *) is overridden.
//
//     ./ompl_constrained_projected [--ompl_motion] [--range=R] [--max_iterations=N] [--output=FILE]

#include "ompl_common.hh"

#include <ompl/base/PlannerTerminationCondition.h>

using namespace ompl_demo;

// Edges checked by OMPL: the goal must be on the manifold and a discrete geodesic must exist.
// Collision checking still happens through the state validity checker.
struct OmplMotionValidator : public ob::MotionValidator
{
    using ob::MotionValidator::MotionValidator;

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        auto *space = dynamic_cast<ob::ProjectedStateSpace *>(si_->getStateSpace().get());
        if (space == nullptr)
        {
            throw ompl::Exception("Expected ProjectedStateSpace");
        }

        const auto *constraint = dynamic_cast<const VampConstraint *>(space->getConstraint().get());
        if (constraint == nullptr)
        {
            throw ompl::Exception("Expected VampConstraint");
        }

        return constraint->constraints.satisfied(state_to_vamp(s2, space)) and space->discreteGeodesic(s1, s2, false);
    }

    auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }
};

auto main(int argc, char **argv) -> int
{
    const experiments::Args args(argc, argv);
    const std::size_t max_iterations = args.get_size("max_iterations", 100000);

    const std::array<float, dimension> start = {1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100};
    const std::array<float, dimension> goal = {-1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000};

    // Sphere cage.
    EnvironmentInput environment;
    sphere_cage(
        environment,
        {{0.56, 0, 0.450}, {0.1, 0, 0.7}, {0, 0.55, 0.25}, {-0.55, 0, 0.25}, {-0.35, -0.35, 0.25},
         {0, -0.55, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8}, {-0.35, 0.35, 0.8}, {-0.55, 0, 0.8},
         {-0.35, -0.35, 0.8}, {0, -0.55, 0.8}, {0.35, -0.35, 0.8}},
        0.15);
    environment.sort();
    const EnvironmentVector env_v(environment);

    // Constraint settings as in the old code: InnerLM projection, 15 iterations, step 1.0.
    vc::ConstraintSettings settings;
    settings.method = vc::ProjMethod::InnerLM;
    settings.max_iterations = 15;
    settings.emit_all_waypoints = false;

    const auto make_set = [&settings]()
    {
        return ConstraintSet(
            make_tsr_constraints(
                {0, 1, 0, 0, 0.3486, 0.647752, 0.2399},
                {-0.01, -10.01, -0.01, -10.01, -10.01, -10.01},
                {0.01, 10.01, 0.01, 10.01, 10.01, 10.01}),
            settings);
    };

    auto constraint = std::make_shared<VampConstraint>(make_set());
    const LocalPlanner lp(make_set());  // separate set: constraints cache per-evaluation state

    auto space = std::make_shared<ob::ProjectedStateSpace>(make_ambient_space(), constraint);
    constraint->set_space(space);
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(space);

    std::shared_ptr<ob::MotionValidator> motion_validator;
    if (args.flag("ompl_motion"))
    {
        motion_validator = std::make_shared<OmplMotionValidator>(csi);
    }
    else
    {
        motion_validator = std::make_shared<VampMotionValidator>(csi, env_v, lp);
    }

    const ConstraintSet validity_set = make_set();
    csi->setStateValidityChecker(std::make_shared<VampStateValidator>(csi, env_v, validity_set));
    csi->setMotionValidator(motion_validator);
    csi->setup();

    ob::ScopedState<> start_state(space), goal_state(space);
    for (auto i = 0U; i < dimension; ++i)
    {
        start_state[i] = start[i];
        goal_state[i] = goal[i];
    }

    std::cout << "Start valid? " << csi->isValid(start_state.get()) << "\n";
    std::cout << "Goal valid? " << csi->isValid(goal_state.get()) << "\n";

    auto pdef = std::make_shared<ob::ProblemDefinition>(csi);
    pdef->setStartAndGoalStates(start_state, goal_state);

    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
    objective->setCostThreshold(objective->infiniteCost());
    pdef->setOptimizationObjective(objective);

    auto planner = std::make_shared<og::RRTConnect>(csi);
    planner->setProblemDefinition(pdef);
    planner->setRange(args.get_float("range", 1.0));
    planner->setup();

    // Terminate after a fixed number of termination-condition polls (roughly, iterations).
    auto counter = std::make_shared<std::size_t>(0);
    ob::PlannerTerminationCondition ptc([counter, max_iterations]() mutable { return ++(*counter) >= max_iterations; });

    const auto start_time = std::chrono::steady_clock::now();
    const ob::PlannerStatus solved = planner->solve(ptc);
    const auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    std::cout << "Iterations: " << *counter << ", failed projections: " << constraint->num_failed_projections
              << std::endl;

    if (solved != ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "No solution found" << std::endl;
        return 1;
    }

    auto *path = pdef->getSolutionPath()->as<og::PathGeometric>();
    std::cout << "Found solution in " << nanoseconds / 1e6 << " ms: " << path->getStateCount()
              << " states, length " << path->length() << std::endl;
    write_path(args.get("output", "trajectory.txt"), *path, space.get());
    return 0;
}
