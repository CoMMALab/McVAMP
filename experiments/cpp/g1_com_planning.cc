// REQUIRES A GENERATED ROBOT: g1_unitree (Unitree G1 humanoid), not shipped with mcvamp.
//
// Whole-body G1 planning under three simultaneous constraints: feet pinned to the ground
// (TaskSpaceConstraint), center of mass over a small support polygon (CoMConstraint), and both
// hands holding one object (BimanualTaskSpaceConstraint, 0.303 m apart, +-0.5 rad slack about
// the hand-to-hand axis), in the scene assets/environments/shelf/humanoid.txt. Port of
// planning_with_constraints/vamp_com_constraint_example.cc.
//
// Generate vamp/robots/g1_unitree.hh with cricket and point CMake at its include root with
// -DEXPERIMENTS_GENERATED_ROBOT_DIR=<dir containing vamp/robots/g1_unitree.hh> (see README).
// The seeds are in the old 35-dim layout and adapted to a quaternion-base robot at run time.
//
//
// By default this runs the original study: range {0.5, 0.75, 1} x dynamic domain {off, on} x
// projection method {InnerLM, OuterLM} x descend rate {0.75, 1} x projection iterations
// {5, 10, 25, 50, 100} x insert-all-waypoints {off, on} (240 attempts, one trial each; planner
// only, the fastest raw path is dumped, as in the original).
//
//     ./g1_com_planning [--quick] [--trials=N] [--planner=rrtc|aorrtc|grrtstar] [--range=R]
//                       [--output=FILE]

#include "g1_common.hh"

#include <vamp/robots/g1_unitree.hh>

using Robot = vamp::robots::G1Unitree;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using Problem = experiments::Problem<Robot, rake>;

static const std::vector<float> start_old = {
    -0.01691, -0.00008, 0.52435, -0.00002, 0.00087,  0.00058,  -0.89966, -0.00864, 0.01527,
    1.75038,  -0.87267, 0.01757, -0.89936, 0.00875,  -0.01522, 1.75040,  -0.87267, -0.01753,
    1.59564,  0.14700,  0.37000, 0.00002,  0.00014,  0.00009,  -0.00003, -0.00006, 0.00000,
    -0.00001, -0.00001, 0.00017, 0.00007,  -0.00000, 0.00003,  0.00000,  0.00002};
static const std::vector<float> goal_old = {
    0.03977,  -0.00000, 0.72049,  0.00000,  0.03753,  -0.00002, -0.31120, 0.00002,  0.00004,
    0.64394,  -0.41457, -0.00000, -0.31121, -0.00004, 0.00000,  0.64394,  -0.41456, -0.00000,
    -0.00002, -0.00000, -0.24500, -0.00031, -0.00532, -0.00196, -0.00210, 0.00362,  -0.00156,
    0.00867,  -0.00646, 0.00363,  -0.00701, 0.00148,  -0.00521, 0.00134,  0.01348};

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);

    Problem problem;
    problem.name = "g1 feet + CoM + bimanual hold, humanoid shelf";
    problem.start = experiments::adapt_base_layout<Robot>(start_old);
    problem.goal = experiments::adapt_base_layout<Robot>(goal_old);

    if (not experiments::load_cuboids_txt(problem.environment, experiments::asset_path("environments/shelf/humanoid.txt")))
    {
        return 1;
    }

    problem.make_constraints = []()
    {
        experiments::G1Stance stance;
        stance.left_foot = {1, 0, 0, 0, 0.12, 0.11, 0.0};
        stance.right_foot = {1, 0, 0, 0, 0.12, -0.11, 0.0};
        stance.foot_bound = {0.01, 0.01, 0.01, 0.05, 0.05, 0.05};
        stance.polygon = {{0.08, -0.045}, {0.08, 0.045}, {0.06, 0.045}, {0.06, -0.045}};
        stance.hands_transform = {1., 0.0005, 0.0005, 0.0005, 0.0, -0.303, 0.0};
        stance.hands_lower = {-0.001, -0.001, -0.001, -0.5, -0.001, -0.001};
        stance.hands_upper = {0.001, 0.001, 0.001, 0.5, 0.001, 0.001};
        return experiments::g1_constraints<Robot, rake>(stance);
    };

    problem.simplify_mode = Problem::SimplifyMode::None;
    problem.dump_simplified = false;
    problem.dd_radius = 10.0;
    problem.defaults.range = 1.0;
    problem.defaults.dynamic_domain = true;
    problem.defaults.projection_iterations = 25;

    problem.sweep.ranges = {0.5, 0.75, 1.0};
    problem.sweep.dynamic_domain = {false, true};
    problem.sweep.methods = {vc::ProjMethod::InnerLM, vc::ProjMethod::OuterLM};
    problem.sweep.descend_rates = {0.75, 1.0};
    problem.sweep.projection_iterations = {5, 10, 25, 50, 100};
    problem.sweep.emit_all_waypoints = {false, true};

    return experiments::run(std::move(problem), args);
}
