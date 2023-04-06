// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/MoveArmToPosition.h"

MoveArmToPosition::MoveArmToPosition(Swerve* sSwerve, ShoulderSub* sShoulder, ArmSub* sArm, WristSub* sWrist, std::string state) {
  this->sShoulder = sShoulder;
  this->sArm = sArm;
  this->sWrist = sWrist;
  this->sSwerve = sSwerve;
  this->state = state;
  AddRequirements(sShoulder);
  AddRequirements(sArm);
  AddRequirements(sWrist);
}

// Called when the command is initially scheduled.
void MoveArmToPosition::Initialize() {
  storedState = constants::Positions.at(state);
}

// Called repeatedly when this Command is scheduled to run
void MoveArmToPosition::Execute() {
  if (abs(sSwerve->gyroAngle < 90)) {
    sShoulder->moveArmPosition(storedState.fShoulderPos);
    sArm->telescopeArmPos(storedState.fArmPos);
    sWrist->setWristPosition(storedState.fWristPos);
  } else {
    sShoulder->moveArmPosition(storedState.bShoulderPos);
    sArm->telescopeArmPos(storedState.bArmPos);
    sWrist->setWristPosition(storedState.bWristPos);
  }
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
