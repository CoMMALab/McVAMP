// REQUIRES A GENERATED ROBOT: g1_unitree (Unitree G1 humanoid), not shipped with mcvamp.
//
// Projection test for the G1 feet + CoM + bimanual constraint stack, without a planner. Port of
// planning_with_constraints/com_constraint.cc: report a start and goal
// configuration (first end-effector pose and largest constraint-row violation), project both onto
// the manifold (InnerLM, generous iteration budget), and write
// the unprojected and projected configurations to trajectory.txt (four CSV rows: start,
// projected start, goal, projected goal) for inspection in a viewer.
//
// Generate vamp/robots/g1_unitree.hh with cricket and point CMake at its include root with
// -DEXPERIMENTS_GENERATED_ROBOT_DIR=<dir containing vamp/robots/g1_unitree.hh> (see README).
//
//     ./g1_com_projection [--output=FILE]

#include "g1_common.hh"

#include <vamp/robots/g1_unitree.hh>

using Robot = vamp::robots::G1Unitree;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using ConstraintSet = vamp::planning::constraint::ConstraintSet<Robot, rake>;

static const std::vector<float> start_old = {
    -0.01691, -0.00008, 0.52435, -0.00002, 0.00087,  0.00058,  -0.89966, -0.00864, 0.01527,
    1.75038,  -0.87267, 0.01757, -0.89936, 0.00875,  -0.01522, 1.75040,  -0.87267, -0.01753,
    1.59564,  0.14700,  0.37000, 0.00002,  0.00014,  0.00009,  -0.00003, -0.00006, 0.00000,
    -0.00001, -0.00001, 0.00017, 0.00007,  -0.00000, 0.00003,  0.00000,  0.00002};
static const std::vector<float> goal_old = {
    -0.0804749, -0.222722, 0.329562,    0.0752235, -0.234729, -0.187489, -0.540513,  0.159474,  0.036218,
    1.20727,    -0.508012, -0.00816858, -0.55574,  -0.25357,  0.255191,  1.33009,    -0.561027, -0.0319013,
    0.578079,   0.0344014, 0.0243313,   -0.355914, -0.118091, -0.25972,  -0.0948007, -0.253484, 0.0772362,
    0.174869,   0.171759,  0.0892086,   0.124599,  0.0630061, 0.118393,  0.0219054,  0.0661802};

// Robot::eefk gives the pose of the first end-effector only (Robot::end_effectors lists all four),
// so report that plus the stacked constraint-row errors of the whole stack.
static void print_state(const char *label, const ConstraintSet &constraints, const Robot::Configuration &q)
{
    const auto full = q.to_array();
    std::array<float, Robot::dimension> values;
    std::copy(full.begin(), full.begin() + Robot::dimension, values.begin());
    std::cout << label << ": left hand at " << Robot::eefk(values).translation().transpose() << "\n";

    Robot::ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = q.broadcast(i);
    }

    std::vector<float> error(constraints.total_rows());
    std::vector<float> jacobian(constraints.total_rows() * Robot::dimension);
    constraints.error_jacobian(block, 0, error.data(), jacobian.data());
    float largest = 0.F;
    for (const auto e : error)
    {
        largest = std::max(largest, std::abs(e));
    }

    std::cout << "  " << error.size() << " constraint rows, largest violation " << largest << std::endl;
}

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);
    std::cout << std::fixed << std::setprecision(5);

    experiments::G1Stance stance;
    stance.left_foot = {1, 0, 0, 0, 0.12, 0.11, 0.0};
    stance.right_foot = {1, 0, 0, 0, 0.12, -0.11, 0.0};
    stance.foot_bound = {0.01, 0.01, 0.01, 0.05, 0.05, 0.05};
    stance.polygon = {{0.08, -0.045}, {0.08, 0.045}, {0.06, 0.045}, {0.06, -0.045}};
    stance.hands_transform = {1., 0.0005, 0.0005, 0.0005, 0.0, -0.303, 0.0};
    stance.hands_lower = {-0.001, -0.001, -0.001, -0.5, -0.001, -0.001};
    stance.hands_upper = {0.001, 0.001, 0.001, 0.5, 0.001, 0.001};

    vc::ConstraintSettings settings;
    settings.method = vc::ProjMethod::InnerLM;
    settings.max_iterations = 100;
    const ConstraintSet constraints(experiments::g1_constraints<Robot, rake>(stance), settings);

    std::ofstream outfile(args.get("output", "trajectory.txt"));
    const auto write = [&outfile](const Robot::Configuration &q)
    {
        const auto array = q.to_array();
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            outfile << ((i == 0) ? "" : ",") << array[i];
        }

        outfile << "\n";
    };

    bool all_ok = true;
    for (const auto &[label, seed] : {std::pair{"start", start_old}, std::pair{"goal", goal_old}})
    {
        const auto q0 = experiments::make_configuration<Robot>(experiments::adapt_base_layout<Robot>(seed));
        auto q = q0;
        print_state((std::string(label) + " (before projection)").c_str(), constraints, q0);
        const bool ok = constraints.project(q);
        all_ok = all_ok and ok;
        std::cout << label << " projection converged = " << ok << ", satisfied = " << constraints.satisfied(q)
                  << ", moved " << (q - q0).l2_norm() << std::endl;
        print_state((std::string(label) + " (after projection)").c_str(), constraints, q);
        write(q0);
        write(q);
    }

    return (all_ok) ? 0 : 1;
}
