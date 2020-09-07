#ifndef SLAM_KARTO_CERES_CERES_SOLVER_H
#define SLAM_KARTO_CERES_CERES_SOLVER_H

#include <ros/ros.h>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_karto/Mapper.h"
#include "slam_karto_ceres/types.h"

class CeresSolver : public karto::ScanSolver
{
public:
  CeresSolver();
  virtual ~CeresSolver();

  virtual void Clear();
  virtual void Compute();
  virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

private:
  std::map<int, Pose2d> poses_;
  std::vector<Constraint2d> constraints_;

  karto::ScanSolver::IdPoseVector corrections_;
};

#endif  // SLAM_KARTO_CERES_CERES_SOLVER_H