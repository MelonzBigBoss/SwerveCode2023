// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "ctre/Phoenix.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "Constants.h"
#include <math.h>
#include "ctre/Phoenix.h"
#include "Conversions.h"

#pragma once

class SwerveModule {
 public:
    struct SwerveModuleConstants {
      int driveMotorID;
      int angleMotorID;
      int cancoderID;
      units::degree_t angleOffset;
    };

    SwerveModule();
    SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants);
    int moduleNumber;
    void setDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop);
    frc::Rotation2d getCanCoder();
    frc::SwerveModuleState getState();
    frc::SwerveModulePosition getPosition();
    frc::SimpleMotorFeedforward<units::meter> feedforward {constants::driveV, constants::drivekV, constants::drivekA};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* mAngleMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* mDriveMotor;
    ctre::phoenix::sensors::WPI_CANCoder* angleEncoder;
  private:
    units::degree_t angleOffset;
    
    frc::Rotation2d getAngle();
    void resetToAbsolute();
    void configAngleEncoder();
    void configAngleMotor();
    void configDriveMotor();

    TalonFXConfiguration swerveAngleFXConfig {};
    TalonFXConfiguration swerveDriveFXConfig {};
    CANCoderConfiguration swerveCanCoderConfig {};

    inline static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = std::fmod(scopeReference, 360.0);
      if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
      } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
        newAngle += 360;
      }
      while (newAngle > upperBound) {
        newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
        newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
        newAngle += 360;
      }
      return newAngle;
    }

    inline static frc::SwerveModuleState optimize(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle) {
      double targetAngle = placeInAppropriate0To360Scope(currentAngle.Degrees().value(), desiredState.angle.Degrees().value());
      double targetSpeed = desiredState.speed.value();
      double delta = targetAngle - currentAngle.Degrees().value();
      if (abs(delta) > 90){
          targetSpeed = -targetSpeed;
          targetAngle += delta > 90 ? (-180) : (180);
      }        
      return frc::SwerveModuleState(units::meters_per_second_t(targetSpeed), frc::Rotation2d(units::degree_t(targetAngle)));
    }
};
