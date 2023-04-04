// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ArmSub.h"

ArmSub::ArmSub() {
    mArmMotor = new WPI_TalonFX(22, "Karen");

    TalonFXConfiguration armMotorFXConfig {};

    SupplyCurrentLimitConfiguration driveSupplyLimit {true, 25, 25, 0};

    armMotorFXConfig.slot0.kP = 0.1;
    armMotorFXConfig.slot0.kI = 0;
    armMotorFXConfig.slot0.kD = 0;
    armMotorFXConfig.slot0.kF = 0;
    armMotorFXConfig.motionAcceleration = 10000; // 2048_u / 100_ms / 1_s   
    armMotorFXConfig.motionCruiseVelocity = 1000; // 2048_u / 100_ms        
    armMotorFXConfig.supplyCurrLimit = driveSupplyLimit;
    armMotorFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    armMotorFXConfig.openloopRamp = 0.2;
    armMotorFXConfig.closedloopRamp = 0.5;
    armMotorFXConfig.nominalOutputForward = 0.03;
    armMotorFXConfig.nominalOutputReverse = -0.03;
    armMotorFXConfig.peakOutputForward = 1;
    armMotorFXConfig.peakOutputReverse = -1;

    mArmMotor->ConfigFactoryDefault();
    mArmMotor->ConfigAllSettings(armMotorFXConfig);
    mArmMotor->SetInverted(false);
    mArmMotor->SetNeutralMode(NeutralMode::Brake);
    mArmMotor->SetSelectedSensorPosition(0);
    mArmMotor->SetSensorPhase(true);
}

// This method will be called once per scheduler run
void ArmSub::Periodic() {}

void ArmSub::stop() {
  mArmMotor->StopMotor();
}

void ArmSub::telescopeArm(double output){
  mArmMotor->Set(ControlMode::PercentOutput, output);
}

void ArmSub::telescopeArmPos(double percent) {
  mArmMotor->Set(ControlMode::MotionMagic,  mapdouble(percent, 0, 100, 0, 52689));
}

void ArmSub::InitSendable(wpi::SendableBuilder& builder) {
  frc2::SubsystemBase::InitSendable(builder);
  builder.AddDoubleProperty("Extension", [this] { return mapdouble(mArmMotor->GetSelectedSensorPosition(), 0, 52689, 0, 100); }, nullptr);
  builder.AddDoubleProperty("Raw Extension", [this] { return mArmMotor->GetSelectedSensorPosition(); }, nullptr);
}