// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#define PRACTICE 0


#pragma once
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <iostream>
#include <numbers>
#include <map>
struct ArmState {
  double fWristPos;
  double fArmPos;
  double fShoulderPos;

  double bWristPos;
  double bArmPos;
  double bShoulderPos;
};
#define s(NAME, W, A, S) {NAME, ArmState(W, A, S, W, A, S)},
#define a(NAME, fW, fA, fS, bW, bA, bS) {NAME, ArmState(fW, fA, fS, bW, bA, bS)},
namespace constants {
    inline std::map<std::string, ArmState> Positions{
      {"STOWED", ArmState(0,0,0,0,0,0)},
      s("sKEBOB", 30, 30, 30)
      a("KEBOB", 10, 10, 10, 20, 20, 20)
    }; 

  constexpr double deadBand = 0.06;

  constexpr units::volt_t maxVoltage = 12_V;
  constexpr units::ampere_t stallCur = 257_A;
  constexpr units::newton_meter_t stallTorque = 4.69_Nm;
  constexpr double freeSpinRPM = 6380;
  constexpr units::ampere_t freeSpinCur = 1.5_A;
  constexpr double swerveGearRatio = 6.12; // L3
  constexpr int motorcount = 4;
  constexpr units::pound_t mass = 70_lb;
  constexpr units::degree_t gyroOffset = 180_deg;
  constexpr units::meter_t wheelDiameter = 4_in;
  constexpr units::radian_t wheelCircumference = units::radian_t(wheelDiameter.value() * std::numbers::pi);
  constexpr units::meter_t robotWidth = 24_in;
  constexpr units::meter_t robotLength = 25_in;
  
  constexpr units::meters_per_second_t maxSpeed =  units::meters_per_second_t(freeSpinRPM / 60.0 / swerveGearRatio * wheelDiameter.value() * std::numbers::pi);
  constexpr units::meters_per_second_squared_t maxAcceleration = (2.0 * motorcount * stallTorque * swerveGearRatio) / (wheelDiameter * mass);
  const units::radians_per_second_t maxAngularSpeed = units::radians_per_second_t( maxSpeed.value() / (std::hypot(robotWidth.value() / 2, robotLength.value() / 2)));

  constexpr units::volt_t driveV = 0.2159 * 1_V;//0.056_V; //
  constexpr auto drivekV = 0.0535 * 1_V / 1_mps;//maxVoltage / maxSpeed; // 
  constexpr auto drivekA = 0.0106 * 1_V / 1_mps_sq;//maxVoltage / maxAcceleration; // 

  const frc::Translation2d m_frontLeftLocation{robotWidth /2 - 3_in, robotLength /2 - 3_in};
  const frc::Translation2d m_frontRightLocation{robotWidth /2 - 3_in, -robotLength /2 - 3_in};
  const frc::Translation2d m_backLeftLocation{-robotWidth /2 - 3_in, robotLength /2 - 3_in};
  const frc::Translation2d m_backRightLocation{-robotWidth /2 - 3_in, -robotLength /2 - 3_in};

  #if PRACTICE 
    constexpr units::degree_t modOffsets[] = {274_deg, 234_deg, 319_deg, 305_deg};
  #else
    constexpr units::degree_t modOffsets[] = {13_deg, 218_deg, 310_deg, 288_deg};
  #endif


}  
