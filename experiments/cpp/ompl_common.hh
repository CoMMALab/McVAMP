#pragma once

// Glue between OMPL's constrained-planning framework and VAMP's manifold constraints (Panda).
//
// OMPL owns the sampling-based planner (RRT-Connect here) and the ProjectedStateSpace; VAMP
// supplies (a) the manifold: a vamp::planning::constraint::ConstraintSet wrapped in an
// ompl::base::Constraint, and (b) SIMD collision checking as validity checkers. Compared with the
// old ComposableConstraints-based code, all VAMP-side calls now go through:
//   ConstraintSet::project / satisfied / squared_error   (single configurations)
//   ConstrainedLocalPlanner::validate                     (projected + collision-checked edge)
//
// OMPL::Constraint::project(ob::State *) is a virtual of the OMPL fork this was developed against
// (the vamp "myfork" OMPL 2.0 beta); stock OMPL only has the Eigen overload.

#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "common.hh"

#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/robots/panda.hh>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Exception.h>

namespace ompl_demo
{
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    namespace vc = vamp::planning::constraint;

    using Robot = vamp::robots::Panda;
    static constexpr std::size_t dimension = Robot::dimension;
    static constexpr std::size_t rake = vamp::FloatVectorWidth;
    using Configuration = Robot::Configuration;
    using ConstraintSet = vc::ConstraintSet<Robot, rake>;
    using LocalPlanner = vc::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
    using EnvironmentInput = vamp::collision::Environment<float>;
    using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

    // A single-end-effector Panda TSR: rTe is the identity, wTr the reference frame in the world.
    inline auto make_tsr_constraints(
        const std::array<float, 7> &world_to_reference,
        const std::array<float, 6> &lower,
        const std::array<float, 6> &upper) -> std::vector<std::shared_ptr<const vc::Constraint<Robot, rake>>>
    {
        return {std::make_shared<vc::TaskSpaceConstraint<Robot, rake>>(
            experiments::repeat<Robot::n_eef>(std::array<float, 7>{1, 0, 0, 0, 0, 0, 0}),
            experiments::repeat<Robot::n_eef>(world_to_reference),
            experiments::repeat<Robot::n_eef>(lower),
            experiments::repeat<Robot::n_eef>(upper))};
    }

    template <typename Vector>
    inline auto to_vamp(const Vector &x) -> Configuration
    {
        std::array<float, dimension> array;
        for (auto i = 0U; i < dimension; ++i)
        {
            array[i] = static_cast<float>(x[i]);
        }

        return Configuration(array);
    }

    inline auto to_reals(const Configuration &c) -> std::vector<double>
    {
        const auto array = c.to_array();
        return std::vector<double>(array.begin(), array.begin() + dimension);
    }

    // Real-vector ambient space with the Panda's joint limits.
    inline auto make_ambient_space() -> std::shared_ptr<ob::RealVectorStateSpace>
    {
        auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);

        // Bounds from the VAMP robot: scale the unit configuration space to min/max.
        auto low = Configuration(std::array<float, dimension>{});
        auto high = Configuration(std::array<float, dimension>{1, 1, 1, 1, 1, 1, 1});
        Robot::scale_configuration(low);
        Robot::scale_configuration(high);

        ob::RealVectorBounds bounds(dimension);
        for (auto i = 0U; i < dimension; ++i)
        {
            bounds.setLow(i, low[{0, i}]);
            bounds.setHigh(i, high[{0, i}]);
        }

        space->setBounds(bounds);
        return space;
    }

    // ompl::base::Constraint over a VAMP ConstraintSet. The set's squared-error (a scalar, zero on
    // the manifold) is the constraint function, so the co-dimension is 1; projection is VAMP's
    // iterative projection. Not thread-safe (the set caches Jacobians).
    class VampConstraint : public ob::Constraint
    {
    public:
        explicit VampConstraint(ConstraintSet set, double tolerance = 1e-4)
          : ob::Constraint(dimension, 1, tolerance), constraints(std::move(set))
        {
        }

        auto error(const Configuration &q) const -> double
        {
            Robot::ConfigurationBlock<rake> block;
            for (auto i = 0U; i < dimension; ++i)
            {
                block[i] = q.broadcast(i);
            }

            return static_cast<double>(constraints.squared_error(block)[{0, 0}]);
        }

        auto project_configuration(Configuration &q) const -> bool
        {
            const bool ok = constraints.project(q);
            if (not ok)
            {
                ++num_failed_projections;
            }

            return ok;
        }

        // Eigen interface (used by OMPL's projected state space).
        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
        {
            out[0] = error(to_vamp(x));
        }

        auto distance(const Eigen::Ref<const Eigen::VectorXd> &x) const -> double override
        {
            return error(to_vamp(x));
        }

        auto isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const -> bool override
        {
            return constraints.satisfied(to_vamp(x));
        }

        auto project(Eigen::Ref<Eigen::VectorXd> x) const -> bool override
        {
            auto q = to_vamp(x);
            if (not project_configuration(q))
            {
                return false;
            }

            const auto array = q.to_array();
            for (auto i = 0U; i < dimension; ++i)
            {
                x[i] = static_cast<double>(array[i]);
            }

            return true;
        }

        // ob::State interface (the OMPL fork's virtual): read the real values of the projected state.
        auto project(ob::State *state) const -> bool override
        {
            const auto space = space_.lock();
            if (not space)
            {
                // No space registered: OMPL's default (dispatches to the Eigen overload).
                return ob::Constraint::project(state);
            }

            std::vector<double> reals(dimension);
            space->copyToReals(reals, state);
            auto q = to_vamp(reals);
            const bool ok = project_configuration(q);
            space->copyFromReals(state, to_reals(q));
            return ok;
        }

        void set_space(const std::shared_ptr<ob::ProjectedStateSpace> &space)
        {
            space_ = space;
        }

        ConstraintSet constraints;
        mutable std::size_t num_failed_projections = 0;

    private:
        std::weak_ptr<ob::ProjectedStateSpace> space_;
    };

    // Reads the (ambient) reals of a projected-space state.
    inline auto state_to_vamp(const ob::State *state, const ob::StateSpace *space) -> Configuration
    {
        std::vector<double> reals(space->getDimension());
        space->copyToReals(reals, state);
        return to_vamp(reals);
    }

    // State validity: on the manifold and collision-free.
    struct VampStateValidator : public ob::StateValidityChecker
    {
        VampStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v, const ConstraintSet &set)
          : ob::StateValidityChecker(si), env_v(env_v), set(set)
        {
        }

        auto isValid(const ob::State *state) const -> bool override
        {
            const auto q = state_to_vamp(state, si_->getStateSpace().get());
            return set.satisfied(q) and vamp::planning::validate_motion<Robot, rake, 1>(q, q, env_v);
        }

        const EnvironmentVector &env_v;
        const ConstraintSet &set;
    };

    // Motion validity: VAMP's constrained local planner traces the edge a -> b on the manifold and
    // collision-checks it (what the old project_constraint_motion did).
    struct VampMotionValidator : public ob::MotionValidator
    {
        VampMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v, const LocalPlanner &lp)
          : ob::MotionValidator(si), env_v(env_v), lp(lp)
        {
        }

        auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
        {
            const auto *space = si_->getStateSpace().get();
            const bool ok = lp.validate(state_to_vamp(s1, space), state_to_vamp(s2, space), env_v);
            if (not ok)
            {
                ++num_rejected;
            }

            return ok;
        }

        auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const -> bool override
        {
            throw ompl::Exception("Not implemented!");
        }

        const EnvironmentVector &env_v;
        const LocalPlanner &lp;
        mutable std::size_t num_rejected = 0;
    };

    inline void write_path(const std::string &filename, const og::PathGeometric &path, const ob::StateSpace *space)
    {
        std::ofstream outfile(filename);
        outfile << std::fixed << std::setprecision(10);
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            std::vector<double> reals(space->getDimension());
            space->copyToReals(reals, path.getState(i));
            for (std::size_t j = 0; j < dimension; ++j)
            {
                outfile << ((j == 0) ? "" : ",") << reals[j];
            }

            outfile << "\n";
        }
    }

    inline auto sphere_cage(EnvironmentInput &environment, const std::vector<std::array<float, 3>> &centers, float radius)
    {
        for (const auto &sphere : centers)
        {
            environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
        }
    }
}  // namespace ompl_demo
