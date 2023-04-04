// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include "SubSystems/Swerve.h"
#include "Commands/TeleopSwerve.h"
#include "frc/Joystick.h"
#include "frc2/command/button/JoystickButton.h"
#include "Autos.h"
#include "frc/smartdashboard/SendableChooser.h"
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/XboxController.h>
#include <pathplanner/lib/PathPlanner.h>
#include <frc/PowerDistribution.h>
#include "SubSystems/ShoulderSub.h"
#include "SubSystems/ArmSub.h"
#include "SubSystems/WristSub.h"
#include "SubSystems/LEDSub.h"
#include "Commands/MoveArmToPosition.h"
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc/DriverStation.h>

using namespace pathplanner;
class RobotContainer {
 public:
  RobotContainer();
  Swerve sSwerve {};
  WristSub sWrist {};
  ArmSub sArm {};
  ShoulderSub sShoulder {};
  LEDSub sLED {};

  frc::PowerDistribution PDH {1, frc::PowerDistribution::ModuleType::kRev};

  frc2::CommandPtr GetAutonomousCommand();
  frc::SendableChooser<std::string> m_chooser;
  frc::Joystick PanelJoysticks{0};
  frc::Joystick XBoxController{1};
  frc::Joystick Panel{2};
 private:
  void ConfigureBindings();
  void ConfigureControllerBindings();
  void ConfigureSmartDashBoard();
  frc2::JoystickButton* button2 = new frc2::JoystickButton(&PanelJoysticks, 2);
  frc2::JoystickButton* RightBumper = new frc2::JoystickButton(&XBoxController, 6);
  frc2::JoystickButton* LeftBumper = new frc2::JoystickButton(&XBoxController, 5);
};
