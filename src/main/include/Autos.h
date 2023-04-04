// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "SubSystems/Swerve.h"
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/WaitCommand.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include <frc2/command/Commands.h>

using namespace pathplanner;

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr Nothing();
frc2::CommandPtr PathPlannerAutoTest(Swerve* swerve);
frc2::CommandPtr PathPlannerAutoTestB(Swerve* swerve);
}  // namespace autos
