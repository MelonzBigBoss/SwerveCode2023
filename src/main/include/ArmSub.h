// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SendableBuilderImpl.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "map"
class ArmSub : public frc2::SubsystemBase {
 public:
  ArmSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
    std::map<std::string, double> Positions{
        {"Extended" , 100},
        {"Middle", 50},
        {"Retracted", 0}
    };
    
  void Periodic() override;
  void telescopeArm(double output);
  void telescopeArmPos(double percent);
  void stop();
  void InitSendable(wpi::SendableBuilder& builder) override;
 private:
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* mArmMotor;
    
    inline double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


};
