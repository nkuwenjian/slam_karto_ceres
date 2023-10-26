/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <memory>

#include "glog/logging.h"
#include "open_karto/Mapper.h"

#include "slam_karto_ceres/ceres_solver.h"

namespace slam_karto_ceres {

/**
 * Sample code to demonstrate karto map creation
 * Create a laser range finder device and three "dummy" range scans.
 * Add the device and range scans to a karto Mapper.
 */
std::unique_ptr<karto::Dataset> CreateMap(karto::Mapper* mapper) {
  // Sanity checks.
  CHECK_NOTNULL(mapper);

  std::unique_ptr<karto::Dataset> dataset = std::make_unique<karto::Dataset>();

  // Create a laser range finder device - use SmartPointer to let karto
  // subsystem manage memory.
  karto::Name name("laser0");
  karto::LaserRangeFinder* laser =
      karto::LaserRangeFinder::CreateLaserRangeFinder(
          karto::LaserRangeFinder_Custom, name);
  laser->SetOffsetPose({1.0, 0.0, 0.0});
  laser->SetAngularResolution(karto::math::DegreesToRadians(0.5));
  laser->SetRangeThreshold(12.0);

  dataset->Add(laser);

  // Create three localized range scans, all using the same range readings, but
  // with different poses.

  // Create a vector of range readings. Simple example where all the
  // measurements are the same value.
  std::vector<kt_double> readings(361, 3.0);

  {
    // Create localized range scan.
    karto::LocalizedRangeScan* range_scan =
        new karto::LocalizedRangeScan(name, readings);
    range_scan->SetOdometricPose({0.0, 0.0, 0.0});
    range_scan->SetCorrectedPose({0.0, 0.0, 0.0});

    // Add the localized range scan to the mapper.
    mapper->Process(range_scan);
    LOG(INFO) << std::fixed
              << "Odometric pose: " << range_scan->GetOdometricPose()
              << ", corrected pose: " << range_scan->GetCorrectedPose();

    // Add the localized range scan to the dataset.
    dataset->Add(range_scan);
  }

  {
    // Create localized range scan.
    karto::LocalizedRangeScan* range_scan =
        new karto::LocalizedRangeScan(name, readings);
    range_scan->SetOdometricPose({1.0, 0.0, 1.57});
    range_scan->SetCorrectedPose({1.0, 0.0, 1.57});

    // Add the localized range scan to the mapper.
    mapper->Process(range_scan);
    LOG(INFO) << std::fixed
              << "Odometric pose: " << range_scan->GetOdometricPose()
              << ", corrected pose: " << range_scan->GetCorrectedPose();

    // Add the localized range scan to the dataset.
    dataset->Add(range_scan);
  }

  {
    // Create localized range scan.
    karto::LocalizedRangeScan* range_scan =
        new karto::LocalizedRangeScan(name, readings);
    range_scan->SetOdometricPose({1.0, -1.0, 2.35619449});
    range_scan->SetCorrectedPose({1.0, -1.0, 2.35619449});

    // Add the localized range scan to the mapper.
    mapper->Process(range_scan);
    LOG(INFO) << std::fixed
              << "Odometric pose: " << range_scan->GetOdometricPose()
              << ", corrected pose: " << range_scan->GetCorrectedPose();

    // Add the localized range scan to the dataset.
    dataset->Add(range_scan);
  }

  return dataset;
}

/**
 * Sample code to demonstrate basic occupancy grid creation and print occupancy
 * grid.
 */
std::unique_ptr<karto::OccupancyGrid> CreateOccupancyGrid(
    karto::Mapper* mapper, kt_double resolution) {
  // Sanity checks.
  CHECK_NOTNULL(mapper);
  LOG(INFO) << "Generating map...";

  // Create a map (occupancy grid) - time it.
  std::unique_ptr<karto::OccupancyGrid> occ_grid(
      karto::OccupancyGrid::CreateFromScans(mapper->GetAllProcessedScans(),
                                            resolution));
  return occ_grid;
}

/**
 * Sample code to print a basic occupancy grid.
 */
void PrintOccupancyGrid(const karto::OccupancyGrid* occ_grid) {
  if (occ_grid == nullptr) {
    return;
  }

  // Output ASCII representation of map.
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset =
      occ_grid->GetCoordinateConverter()->GetOffset();

  LOG(INFO) << std::fixed << "width: " << width << ", height: " << height
            << ", scale: " << occ_grid->GetCoordinateConverter()->GetScale()
            << ", offset: (" << offset.GetX() << "," << offset.GetY() << ").";

  for (kt_int32s y = height - 1; y >= 0; y--) {
    for (kt_int32s x = 0; x < width; x++) {
      // Getting the value at position (x,y).
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value) {
        case karto::GridStates_Unknown:
          std::cout << "*";
          break;
        case karto::GridStates_Occupied:
          std::cout << "X";
          break;
        case karto::GridStates_Free:
          std::cout << " ";
          break;
        default:
          std::cout << "?";
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

}  // namespace slam_karto_ceres

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  // Create karto default mapper.
  std::unique_ptr<karto::Mapper> mapper = std::make_unique<karto::Mapper>();

  // Create and set solver.
  std::unique_ptr<slam_karto_ceres::CeresSolver> solver =
      std::make_unique<slam_karto_ceres::CeresSolver>();
  mapper->SetScanSolver(solver.get());

  // Sample code that creates a map from sample device and sample localized
  // range scans.

  // Clear mapper.
  mapper->Reset();

  // Create map from created dataset.
  std::unique_ptr<karto::Dataset> dataset =
      slam_karto_ceres::CreateMap(mapper.get());

  // Create occupancy grid at 0.1 meters resolution and print grid.
  std::unique_ptr<karto::OccupancyGrid> occ_grid =
      slam_karto_ceres::CreateOccupancyGrid(mapper.get(), 0.1);

  slam_karto_ceres::PrintOccupancyGrid(occ_grid.get());

  return 0;
}
