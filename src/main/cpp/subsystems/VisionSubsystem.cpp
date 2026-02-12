// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <vector>

using namespace VisionConstants;

VisionSubsystem::VisionSubsystem(std::function<frc::Pose3d()> accessor, std::function<void(frc::Pose3d, units::millisecond_t)> updater) :
    SubsystemBase("Vision Subsystem"),
    m_camera{kName},
    m_poseEstimator{kAprilTags, kRobotToCam},
    m_accessor{accessor},
    m_updater{updater} {
}

std::optional<frc::Pose3d> VisionSubsystem::SeedPose() {
  std::optional<frc::Pose3d> seed = {};

  ProcessCameraResults([&seed](frc::Pose3d pose, units::millisecond_t timestamp) {
    seed = pose;
  });

  return seed;
}

void VisionSubsystem::Periodic() {
  ProcessCameraResults(m_updater);
}

void VisionSubsystem::ProcessCameraResults(std::function<void(frc::Pose3d, units::millisecond_t)> updater) {
  std::vector<photon::PhotonPipelineResult> camResults = m_camera.GetAllUnreadResults();
  for (auto camResult : camResults) {
    if (camResult.HasTargets()) {
      auto result = m_poseEstimator.EstimateCoprocMultiTagPose(camResult);
      if (!result.has_value()) {
        result = m_poseEstimator.EstimateClosestToCameraHeightPose(camResult);
      }

      if (result.has_value()) {
        // The camera successfully captured target information
        updater(result.value().estimatedPose, result.value().timestamp);
      }
    }
  }
}
