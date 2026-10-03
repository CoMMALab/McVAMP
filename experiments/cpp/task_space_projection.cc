// Projection onto a task-space manifold, without any planner.
//
// Port of planning_with_constraints/task_space_constraints_examples.cc, a debugging harness for
// the projection machinery on the Panda + maze scene. It
//   1. prints the start/goal end-effector poses and their TSR error,
//   2. projects start and goal onto the manifold with ConstraintSet::project,
//   3. discretizes the straight line start -> goal into a SIMD rake of `rake` configurations,
//      projects the whole rake with ConstraintSet::project_all (what ConstrainedLocalPlanner
//      does for every edge) and prints the gaps between adjacent projected lanes. A gap much larger
//      than distance / rake means the projection tore the path apart and the edge is rejected.
// The projected start/goal are written to trajectory.txt (one CSV row each).
//
// Note: the goal below sits at a rotation error of about pi from the reference orientation, where
// the se(3) log map is singular, so projecting it is numerically fragile: it fails with InnerLM, and
// with OuterLM it converged in some builds of this example and not in others (the Python
// API shows the same). The example reports this instead of hiding it.
//
//     ./task_space_projection [--method=InnerLM|OuterLM|GradDesc] [--iterations=N] [--output=FILE]

#include "common.hh"
#include "json_environment.hh"

#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using TSR = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using ConstraintSet = vamp::planning::constraint::ConstraintSet<Robot, rake>;

static constexpr Robot::ConfigurationArray start = {-1.45508, -0.718918, 2.46299, -0.29207, -0.396585, 2.28344, -0.613603};
static constexpr Robot::ConfigurationArray goal = {0.2013, 0.0540317, -2.7026, -0.138382, -0.902814, 3.35919, -1.78867};

static void print_configuration(const char *label, const Robot::Configuration &q)
{
    const auto array = q.to_array();
    std::cout << label << ":";
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        std::cout << " " << array[i];
    }

    std::cout << std::endl;
}

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);
    std::cout << std::fixed << std::setprecision(5);

    vamp::collision::Environment<float> environment;
    if (not experiments::load_cuboids_json(environment, experiments::asset_path("environments/maze/cuboids.json")))
    {
        return 1;
    }

    environment.sort();
    const vamp::collision::Environment<vamp::FloatVector<rake>> env_v(environment);

    vc::ConstraintSettings settings;
    const auto method = args.get("method", "InnerLM");
    settings.method = (method == "OuterLM") ? vc::ProjMethod::OuterLM :
                      (method == "GradDesc") ? vc::ProjMethod::GradDesc :
                                               vc::ProjMethod::InnerLM;
    settings.max_iterations = args.get_size("iterations", 100);

    const auto tsr = std::make_shared<TSR>(
        experiments::repeat<Robot::n_eef>(std::array<float, 7>{1, 0, 0, 0, 0, 0, 0}),
        experiments::repeat<Robot::n_eef>(std::array<float, 7>{0, 1, 0, 0, 0.3486, 0.647752, 0.2399}),
        experiments::repeat<Robot::n_eef>(std::array<float, 6>{-0.01, -10.01, -0.01, -0.01, -0.01, -0.01}),
        experiments::repeat<Robot::n_eef>(std::array<float, 6>{0.01, 10.01, 0.01, 0.01, 0.01, 0.01}));
    const ConstraintSet constraints({tsr}, settings);

    // Broadcast a configuration into a block whose `rake` lanes are all identical, and read the
    // per-lane squared constraint error of it.
    const auto error_of = [&constraints](const Robot::Configuration &q)
    {
        Robot::ConfigurationBlock<rake> block;
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = q.broadcast(i);
        }

        return constraints.squared_error(block)[{0, 0}];
    };

    auto start_q = Robot::Configuration(start);
    auto goal_q = Robot::Configuration(goal);

    std::cout << "Start end-effector pose:\n" << Robot::eefk(start).matrix() << std::endl;
    std::cout << "Goal end-effector pose:\n" << Robot::eefk(goal).matrix() << std::endl;
    std::cout << "Squared TSR error before projection: start " << error_of(start_q) << ", goal "
              << error_of(goal_q) << std::endl;

    const bool start_ok = constraints.project(start_q);
    const bool goal_ok = constraints.project(goal_q);
    print_configuration("Projected start", start_q);
    print_configuration("Projected goal ", goal_q);
    std::cout << "Projection converged: start " << start_ok << ", goal " << goal_ok
              << "; satisfied afterwards: " << constraints.satisfied(start_q) << ", "
              << constraints.satisfied(goal_q) << std::endl;
    std::cout << "Collision free: start "
              << vamp::planning::validate_motion<Robot, rake, 1>(start_q, start_q, env_v) << ", goal "
              << vamp::planning::validate_motion<Robot, rake, 1>(goal_q, goal_q, env_v) << std::endl;

    // Project the rake along the straight line between the projected seeds.
    const auto vector = goal_q - start_q;
    const float distance = vector.l2_norm();
    const auto percents = vamp::FloatVector<rake>(vamp::planning::Percents<rake>::percents);

    Robot::ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = start_q.broadcast(i) + vector.broadcast(i) * percents;
    }

    const bool rake_ok = constraints.project_all(block, distance);
    std::cout << "\nProjected the straight-line rake (" << rake << " lanes, length " << distance
              << "): all lanes converged = " << rake_ok << std::endl;

    float max_gap = 0.F;
    for (auto lane = 0U; lane < rake; ++lane)
    {
        float gap2 = 0.F;
        for (auto j = 0U; j < Robot::dimension; ++j)
        {
            const float previous = (lane == 0) ? start_q.to_array()[j] : block[{j, lane - 1}];
            const float diff = block[{j, lane}] - previous;
            gap2 += diff * diff;
        }

        max_gap = std::max(max_gap, std::sqrt(gap2));
        std::cout << "  lane " << lane << ": gap to previous " << std::sqrt(gap2) << " (unconstrained spacing "
                  << distance / static_cast<float>(rake) << ")" << std::endl;
    }

    std::cout << "Largest gap " << max_gap << "; an edge would be "
              << ((max_gap * max_gap > 4.F * (distance / rake) * (distance / rake)) ? "rejected" : "accepted")
              << " by the continuity check (gap > 2x lane spacing)" << std::endl;

    std::ofstream outfile(args.get("output", "trajectory.txt"));
    for (const auto &q : {start_q, goal_q})
    {
        const auto array = q.to_array();
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            outfile << ((i == 0) ? "" : ",") << array[i];
        }

        outfile << "\n";
    }

    return (start_ok and goal_ok) ? 0 : 1;
}
