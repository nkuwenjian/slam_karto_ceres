// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)
//
// An example of solving a graph-based formulation of Simultaneous Localization
// and Mapping (SLAM). It reads a 2D pose graph problem definition file in the
// g2o format, formulates and solves the Ceres optimization problem, and outputs
// the original and optimized poses to file for plotting.

#include "slam_karto_ceres/ceres_solver.h"

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "slam_karto_ceres/cost_functor/angle_manifold.h"
#include "slam_karto_ceres/cost_functor/pose_graph_2d_error_term.h"
#include "slam_karto_ceres/types.h"

namespace slam_karto_ceres {

namespace {
// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const std::vector<Constraint2d>& constraints,
                              std::map<int, Pose2d>* poses,
                              ceres::Problem* problem) {
  CHECK(poses != nullptr);
  CHECK(problem != nullptr);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction* loss_function = nullptr;
  ceres::Manifold* angle_manifold = cost_functor::AngleManifold::Create();

  for (const auto& constraint : constraints) {
    auto pose_begin_iter = poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    auto pose_end_iter = poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    Eigen::Matrix3d sqrt_information = constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction* cost_function =
        cost_functor::PoseGraph2dErrorTerm::Create(constraint.x, constraint.y,
                                                   constraint.yaw_radians,
                                                   std::move(sqrt_information));
    problem->AddResidualBlock(
        cost_function, loss_function, &pose_begin_iter->second.x,
        &pose_begin_iter->second.y, &pose_begin_iter->second.yaw_radians,
        &pose_end_iter->second.x, &pose_end_iter->second.y,
        &pose_end_iter->second.yaw_radians);

    problem->SetManifold(&pose_begin_iter->second.yaw_radians, angle_manifold);
    problem->SetManifold(&pose_end_iter->second.yaw_radians, angle_manifold);
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  auto pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(&pose_start_iter->second.x);
  problem->SetParameterBlockConstant(&pose_start_iter->second.y);
  problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem* problem) {
  CHECK(problem != nullptr);

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  LOG(INFO) << summary.BriefReport();

  return summary.IsSolutionUsable();
}

}  // namespace

void CeresSolver::Clear() { corrections_.clear(); }

const karto::ScanSolver::IdPoseVector& CeresSolver::GetCorrections() const {
  return corrections_;
}

void CeresSolver::Compute() {
  corrections_.clear();

  LOG(INFO) << "Calling ceres for loop closure.";

  ceres::Problem problem;
  BuildOptimizationProblem(constraints_, &poses_, &problem);
  SolveOptimizationProblem(&problem);

  LOG(INFO) << "Finished ceres for loop closure.";

  for (const auto& iter : poses_) {
    corrections_.emplace_back(
        iter.first,
        karto::Pose2{iter.second.x, iter.second.y, iter.second.yaw_radians});
  }
}

void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* vertex) {
  const karto::Pose2& pose = vertex->GetObject()->GetCorrectedPose();
  int pose_id = vertex->GetObject()->GetUniqueId();
  Pose2d pose2d;
  pose2d.x = pose.GetX();
  pose2d.y = pose.GetY();
  pose2d.yaw_radians = pose.GetHeading();
  poses_[pose_id] = pose2d;

  VLOG(4) << "AddNode id: " << vertex->GetObject()->GetUniqueId();
}

void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* edge) {
  karto::LocalizedRangeScan* source = edge->GetSource()->GetObject();
  karto::LocalizedRangeScan* target = edge->GetTarget()->GetObject();
  auto* link_info = dynamic_cast<karto::LinkInfo*>(edge->GetLabel());

  const karto::Pose2& diff = link_info->GetPoseDifference();
  karto::Matrix3 precision_matrix = link_info->GetCovariance().Inverse();
  Eigen::Matrix<double, 3, 3> info;
  info(0, 0) = precision_matrix(0, 0);
  info(0, 1) = info(1, 0) = precision_matrix(0, 1);
  info(0, 2) = info(2, 0) = precision_matrix(0, 2);
  info(1, 1) = precision_matrix(1, 1);
  info(1, 2) = info(2, 1) = precision_matrix(1, 2);
  info(2, 2) = precision_matrix(2, 2);
  Eigen::Vector3d measurement(diff.GetX(), diff.GetY(), diff.GetHeading());

  Constraint2d constraint2d;
  constraint2d.id_begin = source->GetUniqueId();
  constraint2d.id_end = target->GetUniqueId();
  constraint2d.x = measurement(0);
  constraint2d.y = measurement(1);
  constraint2d.yaw_radians = measurement(2);
  constraint2d.information = info;
  constraints_.push_back(std::move(constraint2d));

  VLOG(4) << "AddConstraint source id: " << source->GetUniqueId()
          << ", target id: " << target->GetUniqueId();
}

}  // namespace slam_karto_ceres
