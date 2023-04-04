// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/TeleopSwerve.h"

TeleopSwerve::TeleopSwerve(Swerve* s_Swerve, frc::Joystick* controller, int translationAxis, int strafeAxis, int rotationAxis, bool fieldRelative, bool openLoop) {
  this->s_Swerve = s_Swerve;
  AddRequirements(s_Swerve);

  this->controller = controller;
  this->translationAxis = translationAxis;
  this->strafeAxis = strafeAxis;
  this->rotationAxis = rotationAxis;
  this->fieldRelative = fieldRelative;
  this->openLoop = openLoop;
}

// Called when the command is initially scheduled.
void TeleopSwerve::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopSwerve::Execute() {
    double yAxis = controller->GetRawAxis(translationAxis);
    double xAxis = controller->GetRawAxis(strafeAxis);   
    double rAxis = controller->GetRawAxis(rotationAxis);
    yAxis = (abs(yAxis) > constants::deadBand) * mapdouble(abs(yAxis), 0.06, 1, 0, 1) * (std::pow(-1, (yAxis < 0)));
    xAxis = (abs(xAxis) > constants::deadBand) * mapdouble(abs(xAxis), 0.06, 1, 0, 1) * (std::pow(-1, (xAxis < 0)));
    rAxis = (abs(rAxis) > constants::deadBand) * mapdouble(abs(rAxis), 0.06, 1, 0, 1) * (std::pow(-1, (rAxis < 0)));
    translation = frc::Translation2d(units::meter_t(yAxis), units::meter_t(xAxis)) * constants::maxSpeed.value();
    rotation = rAxis * constants::maxAngularSpeed.value();
    s_Swerve->Drive(translation, rotation, fieldRelative, openLoop);
}

// Called once the command ends or is interrupted.
void TeleopSwerve::End(bool interrupted) {}

// Returns true when the command should end.
bool TeleopSwerve::IsFinished() {
  return false;
}
