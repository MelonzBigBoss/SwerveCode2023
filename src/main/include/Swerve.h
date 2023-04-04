// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include "AprilTagWrapper.h"
#include "AHRS.h"
#include <frc/controller/ProfiledPIDController.h>
#include "SwerveModule.h"
#include "Constants.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SendableBuilderImpl.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/music/Orchestra.h>

class Swerve : public frc2::SubsystemBase {
 public:
  //ctre::phoenix::music::Orchestra orchestra {};
  std::string songs[11] = {
      "song1.chrp",
      "song2.chrp",
      "song3.chrp",
      "song4.chrp",
      "song5.chrp",
      "song6.chrp",
      "song7.chrp",
      "song8.chrp",
      "song9.chrp", /* the remaining songs play better with three or more FXs */
      "song10.chrp",
      "song11.chrp",
    };

  Swerve();
  void Drive(frc::Translation2d translation, double rotation, bool fieldRelative, bool isOpenLoop);
  void UpdateOdometry();
  void ZeroGyro();
  void SetModuleStates(wpi::array<frc::SwerveModuleState,4> swervemodulestates);
  frc::Pose2d GetPose();
  void resetPose(frc::Pose2d pose = frc::Pose2d(frc::Translation2d(0_m,0_m), frc::Rotation2d(0_deg)));
  void DriveFieldRelative(frc::ChassisSpeeds speeds);
  float gyroAngle = 0.0f;
  frc::Pose2d EstimatedRobotPose {};
  SwerveModule::SwerveModuleConstants SwerveModuleConstantsArray[4] = {
    SwerveModule::SwerveModuleConstants(5,4,6, constants::modOffsets[0]),
    SwerveModule::SwerveModuleConstants(11,10,12,constants::modOffsets[1]), 
    SwerveModule::SwerveModuleConstants(2,1,3,constants::modOffsets[2]),
    SwerveModule::SwerveModuleConstants(8,7,9,constants::modOffsets[3]) 
  };
    frc::Field2d m_field {};
    AprilTagWrapper pcw {};
    AHRS gyro {frc::SPI::Port::kMXP};
    frc::ProfiledPIDController<units::meters> rotatePID {0.01, 0, 0.001, frc::TrapezoidProfile<units::meters>::Constraints{10_mps, 20_mps_sq}};

  void Periodic() override;
  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
    SwerveModule m_frontLeft {0, SwerveModuleConstantsArray[0]};
    SwerveModule m_frontRight {1, SwerveModuleConstantsArray[1]};
    SwerveModule m_backLeft {2, SwerveModuleConstantsArray[2]};
    SwerveModule m_backRight {3, SwerveModuleConstantsArray[3]};
public:
    frc::SwerveDriveKinematics<constants::motorcount> mSwerveDriveKinematics{constants::m_frontLeftLocation, constants::m_frontRightLocation, constants::m_backLeftLocation, constants::m_backRightLocation};
    SwerveModule mSwerveMods[4] = { m_frontLeft, m_frontRight, m_backLeft, m_backRight };
    wpi::array<frc::SwerveModulePosition,4> mSwerveModulePositions = {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition(),
    };

    frc::SwerveDrivePoseEstimator<constants::motorcount> poseEstimator {mSwerveDriveKinematics, frc::Rotation2d(units::degree_t(gyroAngle)), mSwerveModulePositions, frc::Pose2d(frc::Translation2d(units::meter_t(3),units::meter_t(5)), frc::Rotation2d(units::degree_t(0)))};
};