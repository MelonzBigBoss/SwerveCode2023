// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Swerve.h"

Swerve::Swerve() {  
    rotatePID.SetTolerance(units::meter_t(0.001));

    //for(SwerveModule mod : mSwerveMods) {
    //    orchestra.AddInstrument(*mod.mAngleMotor);
    //    orchestra.AddInstrument(*mod.mDriveMotor);
    //}
    //orchestra.LoadMusic("music/" + songs[10]);
}

void Swerve::Periodic() {
    gyroAngle = -gyro.GetYaw();
    UpdateOdometry();

    //if (!orchestra.IsPlaying()) {
    //    orchestra.Play();
    //}
}
void Swerve::ZeroGyro() {
    gyro.ZeroYaw();
}
void Swerve::UpdateOdometry() {
  
    mSwerveModulePositions = wpi::array<frc::SwerveModulePosition,4> {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition(),
    };

    poseEstimator.Update(frc::Rotation2d(units::degree_t(gyroAngle)), mSwerveModulePositions); 
    if (auto estimatedpos = pcw.getEstimatedGlobalPose(); estimatedpos.has_value())
        poseEstimator.AddVisionMeasurement(estimatedpos.value().estimatedPose.ToPose2d(), frc::Timer::GetFPGATimestamp());

    EstimatedRobotPose = poseEstimator.GetEstimatedPosition();
    
    m_field.GetObject("Robot")->SetPose(EstimatedRobotPose);
    frc::SmartDashboard::PutData("Field", &m_field);
}

void Swerve::Drive(frc::Translation2d translation, double rotation, bool fieldRelative, bool isOpenLoop) {
    wpi::array<frc::SwerveModuleState, 4> swerveModuleStates =
        mSwerveDriveKinematics.ToSwerveModuleStates(
            fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                units::meters_per_second_t(translation.X().value()), 
                                units::meters_per_second_t(translation.Y().value()), 
                                units::radians_per_second_t(rotation), 
                                frc::Rotation2d(units::degree_t(gyroAngle - constants::gyroOffset.value())) 
                            )
                            : frc::ChassisSpeeds(
                                units::meters_per_second_t(translation.X().value()), 
                                units::meters_per_second_t(translation.Y().value()), 
                                units::radians_per_second_t(rotation)
                            ));
    mSwerveDriveKinematics.DesaturateWheelSpeeds(&swerveModuleStates, constants::maxSpeed);

    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
}  

void Swerve::DriveFieldRelative(frc::ChassisSpeeds speeds) {
    wpi::array<frc::SwerveModuleState, 4> swerveModuleStates = mSwerveDriveKinematics.ToSwerveModuleStates(speeds);
    mSwerveDriveKinematics.DesaturateWheelSpeeds(&swerveModuleStates, constants::maxSpeed);

    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
    }
}
void Swerve::SetModuleStates(wpi::array<frc::SwerveModuleState,4> swervemodulestates) {
        mSwerveDriveKinematics.DesaturateWheelSpeeds(&swervemodulestates, constants::maxSpeed);

    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(swervemodulestates[mod.moduleNumber], false);
    }
}
frc::Pose2d Swerve::GetPose(){
    return EstimatedRobotPose;
}
void Swerve::resetPose(frc::Pose2d pose){
    gyroAngle = -gyro.GetYaw();
    poseEstimator.ResetPosition(frc::Rotation2d(units::degree_t(gyroAngle)), mSwerveModulePositions, pose);
}

void Swerve::InitSendable(wpi::SendableBuilder& builder) {
    builder.AddDoubleProperty("Angle", [this] { return gyroAngle ; }, nullptr);
    
    for (unsigned int i = 0; i < std::size(mSwerveMods); ++i) {
        builder.AddDoubleProperty("Mod " + std::to_string(i) + " Cancoder", [=,this] { return mSwerveMods[i].getCanCoder().Degrees().value(); }, nullptr);
    }
}