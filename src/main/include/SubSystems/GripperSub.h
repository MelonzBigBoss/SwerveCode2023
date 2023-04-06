// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include "rev/Rev2mDistanceSensor.h"
#include <frc/smartdashboard/SendableBuilderImpl.h>

class GripperSub : public frc2::SubsystemBase {
 public:
  GripperSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
     ctre::phoenix::motorcontrol::can::WPI_TalonFX* mGripperMotor;
     rev::Rev2mDistanceSensor* distSensor;
     
};
