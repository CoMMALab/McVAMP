// Bimanual Panda holding an object, planned through a shelf: the exact problem of
// `vamp/scripts/constrained_example.py --mode bimanual`, run as the old G1 bimanual ablation study
// (vamp_bimanual_example.cc -> g1_bimanual_planning.cc), so the result can be compared with
// myfork's vamp_bimanual_panda_example.cc on identical inputs.
//
// A BimanualTaskSpaceConstraint fixes the right hand relative to the left (wxyz = [0, 0, 1, 0],
// xyz = [0, 0, 0.221814], +-1 mm), as if both arms grasp one rigid object. The environment is a
// two-board shelf with a center divider and the ground; both seeds sit on the manifold with
// >5 cm clearance.
//
// By default this runs the same grid as g1_bimanual_planning: range {0.5, 0.75, 1, 1.5, 2} x dynamic
// domain {off, on} x projection method {InnerLM, OuterLM} x descend rate {0.75, 1} x projection
// iterations {5, 10, 25, 50, 100} x insert-all-waypoints {off, on} x perturbation scale
// {0.01, 0.1, 0.2} (1200 attempts, one trial each; only the fastest attempt so far is simplified
// and dumped). Dynamic-domain radius is 1.0 here, and the same value is used in myfork's variant.
//
//     ./bimanual_panda_planning [--quick] [--trials=N] [--planner=rrtc|aorrtc|grrtstar] [--range=R]
//                               [--output=FILE] [--raw_seeds]

#include "common.hh"

#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/robots/bimanual_panda.hh>

using Robot = vamp::robots::BimanualPanda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using Problem = experiments::Problem<Robot, rake>;

// (center, full extents): two boards, a center divider, the ground.
struct Box
{
    std::array<float, 3> center;
    std::array<float, 3> extents;
};

static const std::array<Box, 4> shelf = {{
    {{0.5F, 0.0F, 0.2F}, {0.3F, 3.0F, 0.014F}},
    {{0.5F, 0.0F, 0.4F}, {0.3F, 3.0F, 0.014F}},
    {{0.5F, 0.0F, 0.3F}, {0.3F, 0.01F, 0.15F}},
    {{0.0F, 0.0F, -0.2F}, {5.0F, 5.0F, 0.2F}},
}};

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);

    Problem problem;
    problem.name = "bimanual panda hold (constrained_example bimanual problem)";
    problem.start = {-1.388458F, 1.789655F, 0.526891F, -2.779171F, -0.986079F, 3.079894F, -0.75567F,
                     1.630401F,  0.982874F, -0.542026F, -2.682339F, -0.376891F, 2.048735F, 0.422199F};
    problem.goal = {-2.118829F, 0.419675F, 2.249477F, -2.045575F, 1.324726F, 1.762167F, -0.726496F,
                    1.423746F,  1.290487F, -2.148522F, -2.168713F, -0.143205F, 2.446808F, -1.55776F};

    for (const auto &box : shelf)
    {
        problem.environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(
            box.center,
            {0.F, 0.F, 0.F},
            {box.extents[0] / 2, box.extents[1] / 2, box.extents[2] / 2}));
    }

    problem.make_constraints = []() -> std::vector<Problem::ConstraintPtr>
    {
        return {std::make_shared<vc::BimanualTaskSpaceConstraint<Robot, rake>>(
            std::array<float, 7>{0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.221814},
            std::array<float, 6>{-0.001, -0.001, -0.001, -0.001, -0.001, -0.001},
            std::array<float, 6>{0.001, 0.001, 0.001, 0.001, 0.001, 0.001})};
    };

    problem.dd_radius = 1.0;
    problem.report_perturbation = true;
    problem.defaults.range = 1.0;
    problem.defaults.projection_iterations = 25;

    problem.sweep.ranges = {0.5, 0.75, 1.0, 1.5, 2.0};
    problem.sweep.dynamic_domain = {false, true};
    problem.sweep.methods = {vc::ProjMethod::InnerLM, vc::ProjMethod::OuterLM};
    problem.sweep.descend_rates = {0.75, 1.0};
    problem.sweep.projection_iterations = {5, 10, 25, 50, 100};
    problem.sweep.emit_all_waypoints = {false, true};
    problem.sweep.perturbation_scales = {0.01, 0.1, 0.2};

    return experiments::run(std::move(problem), args);
}
