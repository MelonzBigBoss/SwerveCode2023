// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
  m_container.Panel.SetOutputs(0b11111111111111111111111111111111);
  m_container.XBoxController.SetOutputs(0b11111111111111111111111111111111);
  m_container.XBoxController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1); // Earthquake
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  m_container.sShoulder.SetBrakeMode(brakeSwitch.Get());
  m_container.sArm.SetBrakeMode(brakeSwitch.Get());
}

void Robot::DisabledExit() {
  m_container.sShoulder.SetBrakeMode(true);
  m_container.sArm.SetBrakeMode(true);
}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
