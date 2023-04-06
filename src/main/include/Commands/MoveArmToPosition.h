// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "SubSystems/ShoulderSub.h"
#include "SubSystems/ArmSub.h"
#include "SubSystems/WristSub.h"
#include "SubSystems/Swerve.h"

#include "Util/Constants.h"
class MoveArmToPosition
    : public frc2::CommandHelper<frc2::CommandBase, MoveArmToPosition> {
 public:
  MoveArmToPosition(Swerve* sSwerve, ShoulderSub* sShoulder, ArmSub* sArm, WristSub* sWrist, std::string state = "STOWED");
  ShoulderSub* sShoulder;
  ArmSub* sArm;
  WristSub* sWrist;
  Swerve* sSwerve;
  std::string state = 0;
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  private:
    ArmState storedState;
};
