// Panda carries a "mug" through a shelf: end-effector slides in a horizontal plane.
//
// Port of planning_with_constraints/vamp_crrtc_transport_mug.cc. The TSR keeps the
// end-effector at a fixed height (+-1 cm) and orientation (+-0.01 rad) relative to the
// reference frame, free to move in the reference x-y plane, while the arm reaches between the
// shelf boards from `assets/environments/shelf/panda.txt`. The original was a single run (range 1.0,
// InnerLM, descend rate 1.0, no dynamic domain) followed by simplification and a dump of the
// simplified path; so is this: there is no study to run.
//
// Note: with the shelf as shipped, the compartment ceiling (z = 0.4) leaves the Panda's hand too
// little clearance at the TSR height (z = 0.243), and no path exists; the planner reports "no path"
// (the same holds through the Python API). --open_shelf removes that top board, which makes the
// problem solvable.
//
//     ./crrtc_transport_mug [--open_shelf] [--planner=rrtc|aorrtc|grrtstar] [--range=R]
//                           [--output=FILE]

#include "common.hh"

#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using TSR = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using Problem = experiments::Problem<Robot, rake>;

auto main(int argc, char **argv) -> int
{
    namespace vc = vamp::planning::constraint;
    const experiments::Args args(argc, argv);

    Problem problem;
    problem.name = "panda shelf transport, plane TSR";
    problem.start = {-1.053, -1.39, 1.878, -1.434, -0.531, 2.386, 2.761};
    problem.goal = {-2.132, 1.558, 1.406, -1.452, 0.228, 2.444, -1.034};

    if (not experiments::load_cuboids_txt(problem.environment, experiments::asset_path("environments/shelf/panda.txt")))
    {
        return 1;
    }

    if (args.flag("open_shelf"))
    {
        // Second line of environments/shelf/panda.txt: the board at z = 0.4 above the compartment.
        problem.environment.cuboids.erase(problem.environment.cuboids.begin() + 1);
    }

    // The original example built a two-sphere "mug" attachment but left it commented out; it can be
    // enabled here by attaching spheres to end-effector 0.
    //
    //     vamp::collision::Attachment<float> mug(Eigen::Transform<float, 3, Eigen::Isometry>(
    //         Eigen::Translation<float, 3>(0.F, 0.F, 0.05F)));
    //     mug.spheres.emplace_back(vamp::collision::factory::sphere::array({0.F, 0.F, 0.F}, 0.03F));
    //     mug.spheres.emplace_back(vamp::collision::factory::sphere::array({0.F, 0.F, 0.03F}, 0.03F));
    //     mug.end_effector = 0;
    //     problem.environment.attachments.emplace_back(mug);

    problem.make_constraints = []() -> std::vector<Problem::ConstraintPtr>
    {
        const auto tsr = std::make_shared<TSR>(
            experiments::repeat<Robot::n_eef>(std::array<float, 7>{1, 0, 0, 0, 0, 0, 0}),
            experiments::repeat<Robot::n_eef>(std::array<float, 7>{0, 0.707107, 0, 0.707107, 0.354, 0.7, 0.243}),
            experiments::repeat<Robot::n_eef>(std::array<float, 6>{-10.01, -10.01, -0.01, -0.01, -0.01, -0.01}),
            experiments::repeat<Robot::n_eef>(std::array<float, 6>{10.01, 10.01, 0.01, 0.01, 0.01, 0.01}));
        return {tsr};
    };

    problem.defaults.range = 1.0;
    problem.defaults.method = vc::ProjMethod::InnerLM;
    problem.defaults.descend_rate = 1.0;
    problem.print_before_solve = false;
    problem.simplify_mode = Problem::SimplifyMode::Always;

    return experiments::run(std::move(problem), args);
}
