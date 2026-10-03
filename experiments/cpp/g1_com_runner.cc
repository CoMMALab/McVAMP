// REQUIRES A GENERATED ROBOT: g1_unitree (Unitree G1 humanoid), not shipped with mcvamp.
//
// Single-configuration variant of g1_com_planning in a different scene (assets/environments/shelf/drake.txt),
// with a larger CoM support polygon (0.2 x 0.5 m) and stance-relative foot poses: the runner
// used for one-off tests of the same feet + CoM + bimanual stack. Port of
// planning_with_constraints/vamp_com_constraint_runner.cc (which passed 4x4 matrices; they are
// near-identity rotations, given here as the equivalent quaternions).
//
// Generate vamp/robots/g1_unitree.hh with cricket and point CMake at its include root with
// -DEXPERIMENTS_GENERATED_ROBOT_DIR=<dir containing vamp/robots/g1_unitree.hh> (see README).
//
// A single run in the original (dynamic domain on, range 1, InnerLM, descend rate 1), so there is
// no study here.
//
//     ./g1_com_runner [--planner=rrtc|aorrtc|grrtstar] [--range=R] [--output=FILE]

#include "g1_common.hh"

#include <vamp/robots/g1_unitree.hh>

using Robot = vamp::robots::G1Unitree;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using Problem = experiments::Problem<Robot, rake>;

static const std::vector<float> start_old = {
    0.0,  0.0, -0.2, 0.0,  0.0,      0.0, -0.9,  0.0,    0.0,  1.75, -0.87267, 0.0,
    -0.9, 0.0, 0.0,  1.75, -0.87267, 0.0, 1.595, -0.277, 0.52, 0.0,  0.0,      0.0,
    0.0,  0.0, 0.0,  0.0,  0.0,      0.0, 0.0,   0.0,    0.0,  0.0,  0.0};
static const std::vector<float> goal_old = {
    0.035, 0.0, 0.031, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,   0.0, 0.0,   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

auto main(int argc, char **argv) -> int
{
    const experiments::Args args(argc, argv);

    Problem problem;
    problem.name = "g1 feet + CoM + bimanual hold, drake shelf";
    problem.start = experiments::adapt_base_layout<Robot>(start_old);
    problem.goal = experiments::adapt_base_layout<Robot>(goal_old);

    if (not experiments::load_cuboids_txt(problem.environment, experiments::asset_path("environments/shelf/drake.txt")))
    {
        return 1;
    }

    problem.make_constraints = []()
    {
        experiments::G1Stance stance;
        stance.left_foot = {1, 0.0005, 0.0005, 0.0005, 0.13, 0.11, -0.725};
        stance.right_foot = {1, 0.0005, 0.0005, 0.0005, 0.13, -0.11, -0.725};
        stance.foot_bound = {0.02, 0.01, 0.01, 0.05, 0.05, 0.05};
        stance.polygon = {{0.2, -0.25}, {0.2, 0.25}, {0.0, 0.25}, {0.0, -0.25}};
        stance.hands_transform = {1., 0.0005, 0.0005, 0.0005, 0.0, -0.303, 0.0};
        stance.hands_lower = {-0.001, -0.001, -0.001, -0.001, -0.001, -0.001};
        stance.hands_upper = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
        return experiments::g1_constraints<Robot, rake>(stance);
    };

    problem.simplify_mode = Problem::SimplifyMode::None;  // the original dumped the raw path
    problem.dump_simplified = false;
    problem.print_before_solve = false;
    problem.defaults.range = 1.0;
    problem.defaults.dynamic_domain = true;
    problem.defaults.projection_iterations = 15;

    return experiments::run(std::move(problem), args);
}
