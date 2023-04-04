// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SubSystems/ShoulderSub.h"

ShoulderSub::ShoulderSub() {
    mLeadGearBoxMotor = new WPI_TalonFX(20, "Karen");
    mChildGearBoxMotor = new WPI_TalonFX(21, "Karen");
    mShoulderEncoder = new WPI_CANCoder(14, "Karen");

    CANCoderConfiguration shoulderEncoderConfig {};

    shoulderEncoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    shoulderEncoderConfig.sensorDirection = false;
    shoulderEncoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    shoulderEncoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;
    shoulderEncoderConfig.magnetOffsetDegrees = armEncoderOffset;

    mShoulderEncoder->ConfigFactoryDefault();
    mShoulderEncoder->ConfigAllSettings(shoulderEncoderConfig);

    TalonFXConfiguration gearBoxMotorFXConfig {};

    SupplyCurrentLimitConfiguration driveSupplyLimit {true, 35, 60, 0.1};

    gearBoxMotorFXConfig.slot0.kP = 0.1;
    gearBoxMotorFXConfig.slot0.kI = 0;
    gearBoxMotorFXConfig.slot0.kD = 0;
    gearBoxMotorFXConfig.slot0.kF = 0;        
    gearBoxMotorFXConfig.supplyCurrLimit = driveSupplyLimit;
    gearBoxMotorFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    gearBoxMotorFXConfig.openloopRamp = 0.2;
    gearBoxMotorFXConfig.closedloopRamp = 0.5;
    gearBoxMotorFXConfig.nominalOutputForward = 0.0;
    gearBoxMotorFXConfig.nominalOutputReverse = 0.0;
    gearBoxMotorFXConfig.remoteFilter1.remoteSensorDeviceID = 14;
    gearBoxMotorFXConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    gearBoxMotorFXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor1;
    gearBoxMotorFXConfig.primaryPID.selectedFeedbackCoefficient = 3600/4096;  

    mLeadGearBoxMotor->ConfigFactoryDefault();
    mLeadGearBoxMotor->ConfigAllSettings(gearBoxMotorFXConfig);
    mLeadGearBoxMotor->SetInverted(true);
    mLeadGearBoxMotor->SetNeutralMode(NeutralMode::Brake);
    mLeadGearBoxMotor->SetSelectedSensorPosition(0);

    mChildGearBoxMotor->ConfigFactoryDefault();
    mChildGearBoxMotor->ConfigAllSettings(gearBoxMotorFXConfig);
    mChildGearBoxMotor->SetInverted(true);
    mChildGearBoxMotor->SetNeutralMode(NeutralMode::Brake);
    mChildGearBoxMotor->SetSelectedSensorPosition(0);
    mChildGearBoxMotor->Follow(*mLeadGearBoxMotor, FollowerType_PercentOutput);

  //  mLeadGearBoxMotor->ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
}

// This method will be called once per scheduler run
void ShoulderSub::Periodic() {
}

void ShoulderSub::stop() {
  mLeadGearBoxMotor->StopMotor();
}

void ShoulderSub::moveArm(double output) {
    mLeadGearBoxMotor->Set(ControlMode::PercentOutput, output);
}

void ShoulderSub::moveArmPosition(double percent) {
  double arb_ff = (mShoulderEncoder->GetAbsolutePosition() + 1 /*value to bring it to 0*/) * 1 /*horizontalholdpower*/;
  mLeadGearBoxMotor->Set(ControlMode::PercentOutput, pidcontroller.Calculate(getAngle(), percent), DemandType_ArbitraryFeedForward, arb_ff);
}

void ShoulderSub::SetBrakeMode(bool brake) {
  mLeadGearBoxMotor->SetNeutralMode((NeutralMode)(1 + brake));
  mChildGearBoxMotor->SetNeutralMode((NeutralMode)(1 + brake));
}

void ShoulderSub::InitSendable(wpi::SendableBuilder& builder) {
  builder.AddDoubleProperty("Angle", [this] {return getAngle(); }, nullptr);
  builder.AddDoubleProperty("Raw Angle", [this] {return mLeadGearBoxMotor->GetSelectedSensorPosition(); }, nullptr);
}

double ShoulderSub::getAngle() { 
  return mapdouble(mLeadGearBoxMotor->GetSelectedSensorPosition(), 0, 2783, 0, 100);  
}