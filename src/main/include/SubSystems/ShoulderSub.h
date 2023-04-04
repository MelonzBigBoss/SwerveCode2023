// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SendableBuilderImpl.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>

#pragma once

class ShoulderSub : public frc2::SubsystemBase {
 public:
  ShoulderSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void moveArm(double output);
  void moveArmPosition(double percent);
  void SetBrakeMode(bool brake);
  void stop();
  double getAngle();
  void InitSendable(wpi::SendableBuilder& builder) override;
 private:
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* mLeadGearBoxMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* mChildGearBoxMotor;
    ctre::phoenix::sensors::WPI_CANCoder* mShoulderEncoder;

    frc::PIDController pidcontroller {0.015, 0.0, 0.0};

    const double armEncoderOffset = -12;

    inline double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};
