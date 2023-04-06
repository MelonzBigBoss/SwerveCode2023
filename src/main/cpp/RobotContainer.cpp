// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  //ConfigureBindings();
  ConfigureControllerBindings();
  ConfigureSmartDashBoard();

  frc::DriverStation::SilenceJoystickConnectionWarning(true);
} 

void RobotContainer::ConfigureSmartDashBoard() {
  m_chooser.SetDefaultOption("Nothing", "nothing");
  m_chooser.AddOption("Auto B", "auto2");
  m_chooser.AddOption("Auto A", "auto1");

  frc::SmartDashboard::PutData("Auto", &m_chooser);
  frc::SmartDashboard::PutData("*WristSub", &sWrist);
  frc::SmartDashboard::PutData("*SwerveSub", &sSwerve);
  frc::SmartDashboard::PutData("*ArmSub", &sArm);
  frc::SmartDashboard::PutData("*ShoulderSub", &sShoulder);
  frc::SmartDashboard::PutData("*LedSub", &sLED);

  frc::SmartDashboard::PutData("PDH", &PDH);
}

void RobotContainer::ConfigureBindings() {
  sSwerve.SetDefaultCommand(TeleopSwerve(&sSwerve, &PanelJoysticks, 0, 1, 2, true, true));
  button2->OnTrue(new frc2::InstantCommand([this] {sSwerve.ZeroGyro();}, {}));
}

void RobotContainer::ConfigureControllerBindings() {
  sSwerve.SetDefaultCommand(TeleopSwerve(&sSwerve, &XBoxController, 1, 0, 4, true, true));
  RightBumper->OnTrue(new frc2::InstantCommand([this] {sSwerve.ZeroGyro();}, {}));
  LeftBumper->WhileTrue(MoveArmToPosition(&sSwerve, &sShoulder, &sArm, &sWrist, "KEBOB").ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  if (m_chooser.GetSelected() == "auto1") {
    return autos::PathPlannerAutoTestB(&sSwerve);
  } 
  if (m_chooser.GetSelected() == "auto2") {
    return autos::PathPlannerAutoTest(&sSwerve);
  }
  return autos::Nothing();
}
