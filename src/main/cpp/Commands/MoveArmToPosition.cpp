// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/MoveArmToPosition.h"

MoveArmToPosition::MoveArmToPosition(ShoulderSub* sShoulder, ArmSub* sArm, WristSub* sWrist, double percent) {
  this->sShoulder = sShoulder;
  this->sArm = sArm;
  this->sWrist = sWrist;
  this->percent = percent;
  AddRequirements(sShoulder);
  AddRequirements(sArm);
  AddRequirements(sWrist);
}

// Called when the command is initially scheduled.
void MoveArmToPosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MoveArmToPosition::Execute() {
 // sShoulder->moveArmPosition(percent);
  sArm->telescopeArmPos(percent);
  sWrist->setWristPosition(percent);
}

// Called once the command ends or is interrupted.
void MoveArmToPosition::End(bool interrupted) {
  sShoulder->stop();
  sArm->stop();
  sWrist->stop();
}

// Returns true when the command should end.
bool MoveArmToPosition::IsFinished() {
  return false;
}
