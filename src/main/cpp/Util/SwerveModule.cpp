// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Util/SwerveModule.h"

SwerveModule::SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){

    SupplyCurrentLimitConfiguration angleSupplyLimit {true, 25, 40, 0.1};

    swerveAngleFXConfig.slot0.kP = 0.6;
    swerveAngleFXConfig.slot0.kI = 0;
    swerveAngleFXConfig.slot0.kD = 12;
    swerveAngleFXConfig.slot0.kF = 0;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
    swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;

    SupplyCurrentLimitConfiguration driveSupplyLimit {true, 35, 60, 0.1};

    swerveDriveFXConfig.slot0.kP = 0.1;
    swerveDriveFXConfig.slot0.kI = 0;
    swerveDriveFXConfig.slot0.kD = 0;
    swerveDriveFXConfig.slot0.kF = 0;        
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    swerveDriveFXConfig.openloopRamp = 0.05;
    swerveDriveFXConfig.closedloopRamp = 0.5;
    swerveDriveFXConfig.nominalOutputForward = 0.03;
    swerveDriveFXConfig.nominalOutputReverse = -0.03;
    
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = false;
    swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;


    this->moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    mDriveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
    configDriveMotor();

    mAngleMotor = new WPI_TalonFX(moduleConstants.angleMotorID);
    configAngleMotor();
}

frc::Rotation2d SwerveModule::getAngle(){      
    return frc::Rotation2d(units::degree_t(Conversions::falconToDegrees(mAngleMotor->GetSelectedSensorPosition(), 12.8)));
}

void SwerveModule::configAngleEncoder(){        
    angleEncoder->ConfigFactoryDefault();
    angleEncoder->ConfigAllSettings(swerveCanCoderConfig,200);
}

void SwerveModule::configAngleMotor(){
    mAngleMotor->ConfigFactoryDefault();
    mAngleMotor->ConfigAllSettings(swerveAngleFXConfig,200);
    mAngleMotor->SetInverted(false);
    mAngleMotor->SetNeutralMode(NeutralMode::Coast);
    resetToAbsolute();
}

void SwerveModule::configDriveMotor(){        
    mDriveMotor->ConfigFactoryDefault();
    mDriveMotor->ConfigAllSettings(swerveDriveFXConfig, 200);
    mDriveMotor->SetInverted(false);
    mDriveMotor->SetNeutralMode(NeutralMode::Brake);
    mDriveMotor->SetSelectedSensorPosition(0);
}

void SwerveModule::resetToAbsolute(){
    double absolutePosition = (getCanCoder().Degrees().value() - angleOffset.value()) / (360.0 / (12.8 * 2048.0));
    mAngleMotor->SetSelectedSensorPosition(absolutePosition);
}

frc::Rotation2d SwerveModule::getCanCoder(){
    return frc::Rotation2d(units::degree_t(angleEncoder->GetAbsolutePosition()));
}

frc::SwerveModuleState SwerveModule::getState(){
    units::meters_per_second_t velocity = units::meters_per_second_t(Conversions::falconToMPS(mDriveMotor->GetSelectedSensorVelocity(), 0.1 * 3.14, 6.12));
    frc::Rotation2d angle = frc::Rotation2d(units::degree_t(Conversions::falconToDegrees(mAngleMotor->GetSelectedSensorPosition(), 12.8)));
    return frc::SwerveModuleState(velocity, angle);
}

frc::SwerveModulePosition SwerveModule::getPosition(){
    units::meter_t distance = units::meter_t(Conversions::falconToMeters(mDriveMotor->GetSelectedSensorPosition(), 0.1 * 3.14, 6.12));
    return frc::SwerveModulePosition( distance, getAngle() );
}

void SwerveModule::setDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop){
    desiredState = optimize(desiredState, getState().angle); 

    if(isOpenLoop){
        double percentOutput = desiredState.speed.value() / constants::maxSpeed.value();
        mDriveMotor->Set(ControlMode::PercentOutput, percentOutput);
    }
    else {
        double velocity = Conversions::MPSToFalcon(desiredState.speed.value(), 0.1 * 3.14, 6.12);
        mDriveMotor->Set(ControlMode::Velocity, velocity, DemandType::DemandType_ArbitraryFeedForward, feedforward.Calculate(desiredState.speed).value()); 
    }

    if (abs(desiredState.speed.value()) > constants::maxSpeed.value() * 0.01) 
        mAngleMotor->Set(ControlMode::Position, Conversions::degreesToFalcon(desiredState.angle.Degrees().value(), 12.8));
}