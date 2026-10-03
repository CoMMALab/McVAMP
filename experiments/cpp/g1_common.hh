#pragma once

// Constraint stack shared by the G1 humanoid CoM examples (robot: g1_unitree, generated with
// cricket; end-effector order: left hand, right hand, left foot, right foot).
//
//   - TaskSpaceConstraint: hands free (+-10), feet pinned to `left_foot` / `right_foot` poses
//                          within `foot_bound`
//   - CoMConstraint: center of mass over `polygon` (counterclockwise xy vertices)
//   - BimanualTaskSpaceConstraint: right hand at `hands_transform` in the left-hand frame

#include <array>
#include <memory>
#include <vector>

#include "common.hh"

#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/manifold/com_constraint.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>

namespace experiments
{
    struct G1Stance
    {
        std::array<float, 7> left_foot;
        std::array<float, 7> right_foot;
        std::array<float, 6> foot_bound;  // symmetric: [-b, b]
        std::vector<std::array<float, 2>> polygon;
        std::array<float, 7> hands_transform;
        std::array<float, 6> hands_lower;
        std::array<float, 6> hands_upper;
    };

    template <typename Robot, std::size_t rake>
    inline auto g1_constraints(const G1Stance &stance)
        -> std::vector<std::shared_ptr<const vamp::planning::constraint::Constraint<Robot, rake>>>
    {
        namespace vc = vamp::planning::constraint;
        constexpr std::array<float, 7> identity = {1, 0, 0, 0, 0, 0, 0};
        constexpr std::array<float, 6> free_bound = {10, 10, 10, 10, 10, 10};

        const auto feet = std::make_shared<vc::TaskSpaceConstraint<Robot, rake>>(
            repeat<Robot::n_eef>(identity),
            std::array<std::array<float, 7>, 4>{identity, identity, stance.left_foot, stance.right_foot},
            std::array<std::array<float, 6>, 4>{
                negated(free_bound), negated(free_bound), negated(stance.foot_bound), negated(stance.foot_bound)},
            std::array<std::array<float, 6>, 4>{free_bound, free_bound, stance.foot_bound, stance.foot_bound});

        return {
            feet,
            std::make_shared<vc::CoMConstraint<Robot, rake>>(stance.polygon),
            std::make_shared<vc::BimanualTaskSpaceConstraint<Robot, rake>>(
                stance.hands_transform, stance.hands_lower, stance.hands_upper)};
    }
}  // namespace experiments
