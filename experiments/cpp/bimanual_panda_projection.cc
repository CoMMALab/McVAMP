// Projection onto a bimanual relative-pose manifold (two Pandas holding one rigid object).
//
// Port of planning_with_constraints/bimanual_constraint.cc (which did not compile on the old
// branch: it referenced an undeclared `task_constraint`). BimanualTaskSpaceConstraint holds the
// pose of end-effector 1 (right) in the frame of end-effector 0 (left) to a fixed transform lTr,
// within +-1e-5. The example
//   1. projects a perturbed configuration with the GradDesc and InnerLM projection methods, and
//   2. walks from start to goal in 10 straight-line steps, projecting every step onto the
//      manifold and reporting whether projection converged and the residual constraint error.
//
//     ./bimanual_panda_projection

#include "common.hh"

#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/robots/bimanual_panda.hh>

using Robot = vamp::robots::BimanualPanda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using BimanualConstraint = vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>;
using ConstraintSet = vamp::planning::constraint::ConstraintSet<Robot, rake>;

static const std::vector<float> start = {
    -1.3238, 1.358, 1.0783, -2.4974, 0.5572, 2.5477, -1.4485, 1.2848, 1.2911, -1.0714, -2.4884, -0.6705, 2.5082, 0.7243};
static const std::vector<float> goal = {
    -1.997, 0.385, 2.1832, -2.0013, 1.3083, 1.8498, -0.7243, 1.2835, 1.3097, -2.0683, -2.1051, -0.1333, 2.4786, -0.7243};
static const std::vector<float> perturbed = {
    -1.45844, 1.1634, 1.29928, -2.39818, 0.70742, 2.40812, -1.30366, 1.28454, 1.29482, -1.27078, -2.41174, -0.56306, 2.50228, 0.43458};

static auto make_set(vamp::planning::constraint::ProjMethod method) -> ConstraintSet
{
    vamp::planning::constraint::ConstraintSettings settings;
    settings.method = method;
    settings.max_iterations = 100;

    // lTr (qw qx qy qz x y z) with +-1e-5 slack on every se(3) component.
    const auto constraint = std::make_shared<BimanualConstraint>(
        std::array<float, 7>{0.0000348, 0.3745484, 0.9272074, 0.0000018, 0.0, 0.0, 0.171814},
        std::array<float, 6>{-1e-5, -1e-5, -1e-5, -1e-5, -1e-5, -1e-5},
        std::array<float, 6>{1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5});
    return ConstraintSet({constraint}, settings);
}

static auto squared_error(const ConstraintSet &set, const Robot::Configuration &q) -> float
{
    Robot::ConfigurationBlock<rake> block;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = q.broadcast(i);
    }

    return set.squared_error(block)[{0, 0}];
}

auto main(int, char **) -> int
{
    namespace vc = vamp::planning::constraint;
    std::cout << std::fixed << std::setprecision(4);

    const auto q0 = experiments::make_configuration<Robot>(perturbed);
    for (const auto method : {vc::ProjMethod::GradDesc, vc::ProjMethod::InnerLM})
    {
        const auto set = make_set(method);
        auto q = q0;
        const float before = squared_error(set, q);
        const bool converged = set.project(q);
        std::cout << experiments::method_name(method) << ": squared error " << before << " -> "
                  << squared_error(set, q) << ", converged = " << converged << std::endl;
    }

    std::cout << "\nStraight-line steps start -> goal, each projected with InnerLM:" << std::endl;
    const auto set = make_set(vc::ProjMethod::InnerLM);
    const auto start_q = experiments::make_configuration<Robot>(start);
    const auto goal_q = experiments::make_configuration<Robot>(goal);
    const auto extension = goal_q - start_q;

    for (auto step = 0U; step <= 10; ++step)
    {
        auto q = start_q + extension * (static_cast<float>(step) / 10.F);
        const float before = squared_error(set, q);
        const bool converged = set.project(q);
        std::cout << "step " << step << ": error " << before << " -> " << squared_error(set, q)
                  << ", converged = " << converged << ", moved " << (q - start_q - extension * (step / 10.F)).l2_norm()
                  << std::endl;
    }

    return 0;
}
