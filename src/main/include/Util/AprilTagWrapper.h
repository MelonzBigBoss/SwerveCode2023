// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
 
#include <photonlib/PhotonPoseEstimator.h>
#include "frc/apriltag/AprilTagFieldLayout.h" 
#include <frc/apriltag/AprilTagFields.h>
#include "frc/Timer.h"    
class AprilTagWrapper {
  public:
    AprilTagWrapper(); 
    std::optional<photonlib::EstimatedRobotPose> getEstimatedGlobalPose(); 

  private:
  frc::AprilTagFieldLayout atfl = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp);
  
  photonlib::PhotonPoseEstimator estimator1 = photonlib::PhotonPoseEstimator(atfl, photonlib::MULTI_TAG_PNP, photonlib::PhotonCamera("OV9281-01"), frc::Transform3d (frc::Translation3d(units::inch_t(12),units::inch_t(12.5),units::meter_t(0.548)),frc::Rotation3d(units::radian_t(0), units::radian_t(0), units::degree_t(-45))) );
  photonlib::PhotonPoseEstimator estimator2 = photonlib::PhotonPoseEstimator(atfl, photonlib::MULTI_TAG_PNP, photonlib::PhotonCamera("Cam_2"), frc::Transform3d (frc::Translation3d(units::inch_t(-12),units::inch_t(-12.5),units::meter_t(0.548)),frc::Rotation3d(units::radian_t(0), units::radian_t(0), units::degree_t(45))) ); 
};