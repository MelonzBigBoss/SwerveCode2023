// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "SubSystems/ShoulderSub.h"
#include "SubSystems/ArmSub.h"
#include "SubSystems/WristSub.h"
class MoveArmToPosition
    : public frc2::CommandHelper<frc2::CommandBase, MoveArmToPosition> {
 public:
  MoveArmToPosition(ShoulderSub* sShoulder, ArmSub* sArm, WristSub* sWrist, double percent);
  ShoulderSub* sShoulder;
  ArmSub* sArm;
  WristSub* sWrist;
  double percent = 0;
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};