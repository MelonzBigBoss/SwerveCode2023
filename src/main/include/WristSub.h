// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SendableBuilderImpl.h>
#include <frc2/command/SubsystemBase.h>
class WristSub : public frc2::SubsystemBase {
  
 public:
  WristSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void moveWrist(double output);
  void setWristPosition(double percent);
  void stop();
  void InitSendable(wpi::SendableBuilder& builder) override;
 private:
      ctre::phoenix::motorcontrol::can::WPI_TalonFX* mWristMotor;
      inline double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
      }
};
