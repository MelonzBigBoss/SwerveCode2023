// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/led/CANdle.h>
#include <frc/DriverStation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <frc/smartdashboard/SendableBuilderImpl.h>
#include <frc2/command/SubsystemBase.h>

class LEDSub : public frc2::SubsystemBase {
 public:
  LEDSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  ctre::phoenix::led::CANdle m_candle {19, "rio"};
  const int ledCount = 200;
  const int ledOffset = 8;
  int colora = 255;
  int colorb = 255;
  ctre::phoenix::led::LarsonAnimation* animation;

};
