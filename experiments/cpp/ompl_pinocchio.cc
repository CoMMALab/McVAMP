// OMPL constrained planning with the task-space constraint evaluated by pinocchio instead of
// VAMP's generated kernels; VAMP is used only for SIMD collision checking. A reference point for
// how much the generated constraint kernels and VAMP's projection buy over the "usual" stack.
//
// Port of planning_with_constraints/ompl_integration_pinocchio.cc. Nothing here touches the
// manifold-constraint API of VAMP (the old file only included the old constraint headers without
// using them); it needs OMPL and pinocchio, and the Panda URDF from the vamp repo
// (resources/panda/panda_spherized.urdf, found via the VAMP_ROOT environment variable or the
// path CMake baked in).
//
// The TSR is the panda "line" constraint of crrtc_spheres: bounds are applied to the se(3) log of
// the end-effector pose in the reference frame, projected with damped least squares.
//
//     ./ompl_pinocchio [--time=SECONDS] [--range=R] [--output=FILE]

#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "common.hh"

#include <vamp/robots/panda.hh>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Exception.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/log.hpp>
#include <pinocchio/spatial/se3.hpp>

#ifndef EXPERIMENTS_VAMP_ROOT
#define EXPERIMENTS_VAMP_ROOT "../vamp"
#endif

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Task space region: bounds on the se(3) log of the end-effector in the reference frame.
struct TSR
{
    Eigen::Isometry3d T_ref;
    Eigen::Matrix<double, 6, 1> lower;
    Eigen::Matrix<double, 6, 1> upper;
};

class SE3Constraint : public ob::Constraint
{
public:
    SE3Constraint(const pinocchio::Model &model, pinocchio::FrameIndex ee_frame, const TSR &tsr)
      : ob::Constraint(model.nq, 6, dist_tol)
      , model_(model)
      , data_(model_)
      , ee_frame_(ee_frame)
      , tsr_(tsr)
      , T_ref_pin_(tsr.T_ref.rotation(), tsr.T_ref.translation())
    {
    }

    // Hinge of the pose error against the TSR bounds (zero inside).
    void function(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        const pinocchio::SE3 T_err = T_ref_pin_.actInv(data_.oMf[ee_frame_]);
        const Eigen::VectorXd error = pinocchio::log6(T_err).toVector();

        for (int i = 0; i < 6; ++i)
        {
            if (error[i] > tsr_.upper[i])
            {
                out[i] = error[i] - tsr_.upper[i];
            }
            else if (error[i] < tsr_.lower[i])
            {
                out[i] = error[i] - tsr_.lower[i];
            }
            else
            {
                out[i] = 0.0;
            }
        }
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &q, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        pinocchio::Data::Matrix6x J(6, model_.nv);
        pinocchio::getFrameJacobian(model_, data_, ee_frame_, pinocchio::LOCAL, J);

        const pinocchio::SE3 T_err = T_ref_pin_.actInv(data_.oMf[ee_frame_]);
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(T_err, Jlog);
        out = -Jlog * J;
    }

    // Damped-least-squares projection.
    bool project(Eigen::Ref<Eigen::VectorXd> x) const override
    {
        Eigen::VectorXd f(6);
        Eigen::MatrixXd J(6, model_.nv);

        for (std::size_t i = 0; i < max_iters_; ++i)
        {
            function(x, f);
            if (f.norm() < dist_tol)
            {
                return true;
            }

            jacobian(x, J);

            pinocchio::Data::Matrix6 JJt;
            JJt.noalias() = J * J.transpose();
            JJt.diagonal().array() += lambda_;
            const Eigen::VectorXd delta = -J.transpose() * JJt.ldlt().solve(f);

            x = pinocchio::integrate(model_, x, -delta * 0.25);
            if (delta.norm() < dist_tol)
            {
                return true;
            }
        }

        return false;
    }

    double distance(const Eigen::Ref<const Eigen::VectorXd> &x) const override
    {
        Eigen::VectorXd out(6);
        function(x, out);
        return out.squaredNorm();
    }

    bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
    {
        return distance(x) < 0.0001;
    }

private:
    pinocchio::Model model_;
    mutable pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_;
    TSR tsr_;
    pinocchio::SE3 T_ref_pin_;

    std::size_t max_iters_ = 50;
    double dist_tol = 1e-3;
    double lambda_ = 1e-3;
};

// State validity by VAMP's SIMD collision checker.
struct VAMPStateValidator : public ob::StateValidityChecker
{
    VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        const Eigen::Map<Eigen::VectorXd> &x = *state->as<ob::ConstrainedStateSpace::StateType>();
        std::array<float, dimension> values;
        for (auto i = 0U; i < dimension; ++i)
        {
            values[i] = static_cast<float>(x[i]);
        }

        const Configuration q(values);
        return vamp::planning::validate_motion<Robot, rake, 1>(q, q, env_v);
    }

    const EnvironmentVector &env_v;
};

// Edges: OMPL's discrete geodesic on the constraint manifold.
struct GeodesicMotionValidator : public ob::MotionValidator
{
    using ob::MotionValidator::MotionValidator;

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        auto *space = dynamic_cast<ob::ProjectedStateSpace *>(si_->getStateSpace().get());
        if (space == nullptr)
        {
            throw ompl::Exception("Expected ProjectedStateSpace");
        }

        return space->discreteGeodesic(s1, s2, false);
    }

    auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }
};

auto main(int argc, char **argv) -> int
{
    const experiments::Args args(argc, argv);

    // Reference frame of the constraint: wxyz + xyz, +-1 cm on all but the y translation.
    const std::array<double, 7> reference = {0, 1, 0, 0, 0.3486, 0.647752, 0.2399};
    TSR tsr;
    tsr.T_ref = Eigen::Isometry3d::Identity();
    tsr.T_ref.rotate(Eigen::Quaterniond(reference[0], reference[1], reference[2], reference[3]));
    tsr.T_ref.pretranslate(Eigen::Vector3d(reference[4], reference[5], reference[6]));
    tsr.lower << -0.01, -10.01, -0.01, -0.01, -0.01, -0.01;
    tsr.upper << 0.01, 10.01, 0.01, 0.01, 0.01, 0.01;

    const char *root_override = std::getenv("VAMP_ROOT");
    const std::string urdf_file =
        std::string((root_override != nullptr) ? root_override : EXPERIMENTS_VAMP_ROOT) +
        "/resources/panda/panda_spherized.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_file, model);
    const pinocchio::FrameIndex ee_frame = model.getFrameId("panda_grasptarget");
    if (ee_frame >= static_cast<pinocchio::FrameIndex>(model.nframes))
    {
        std::cerr << "frame panda_grasptarget not found in " << urdf_file << std::endl;
        return 1;
    }

    // Ambient space with the Panda's joint limits from VAMP.
    auto ambient = std::make_shared<ob::RealVectorStateSpace>(dimension);
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

    ambient->setBounds(bounds);

    auto constraint = std::make_shared<SE3Constraint>(model, ee_frame, tsr);
    auto space = std::make_shared<ob::ProjectedStateSpace>(ambient, constraint);
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(space);

    EnvironmentInput environment;
    const std::vector<std::array<float, 3>> cage = {
        {0.56, 0, 0.450}, {0.1, 0, 0.7}, {0, 0.55, 0.25}, {-0.55, 0, 0.25}, {-0.35, -0.35, 0.25},
        {0, -0.55, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8}, {-0.35, 0.35, 0.8}, {-0.55, 0, 0.8},
        {-0.35, -0.35, 0.8}, {0, -0.55, 0.8}, {0.35, -0.35, 0.8}};
    for (const auto &sphere : cage)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, 0.15F));
    }

    environment.sort();
    const EnvironmentVector env_v(environment);

    csi->setStateValidityChecker(std::make_shared<VAMPStateValidator>(csi, env_v));
    csi->setMotionValidator(std::make_shared<GeodesicMotionValidator>(csi));
    csi->setup();

    Eigen::VectorXd sv(dimension), gv(dimension);
    sv << 1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100;
    gv << -1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000;

    ob::ScopedState<> start(space), goal(space);
    start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
    goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);

    auto pdef = std::make_shared<ob::ProblemDefinition>(csi);
    pdef->setStartAndGoalStates(start, goal);
    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
    objective->setCostThreshold(objective->infiniteCost());
    pdef->setOptimizationObjective(objective);

    auto planner = std::make_shared<og::RRTConnect>(csi);
    planner->setProblemDefinition(pdef);
    planner->setRange(args.get_float("range", 1.0));
    planner->setup();

    const auto start_time = std::chrono::steady_clock::now();
    const ob::PlannerStatus solved = planner->ob::Planner::solve(args.get_float("time", 50.F));
    const auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    if (not solved)
    {
        std::cout << "No solution found" << std::endl;
        return 1;
    }

    auto *path = pdef->getSolutionPath()->as<og::PathGeometric>();
    std::cout << "Found solution in " << nanoseconds / 1e6 << " ms: " << path->getStateCount()
              << " states, length " << path->length() << std::endl;

    std::ofstream outfile(args.get("output", "trajectory.txt"));
    outfile << std::fixed << std::setprecision(10);
    for (std::size_t i = 0; i < path->getStateCount(); ++i)
    {
        std::vector<double> reals(space->getDimension());
        space->copyToReals(reals, path->getState(i));
        for (std::size_t j = 0; j < dimension; ++j)
        {
            outfile << ((j == 0) ? "" : ",") << reals[j];
        }

        outfile << "\n";
    }

    return 0;
}
