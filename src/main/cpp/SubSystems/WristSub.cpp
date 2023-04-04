// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SubSystems/WristSub.h"

WristSub::WristSub() {
    mWristMotor = new WPI_TalonFX(40, "Karen");

    TalonFXConfiguration wristMotorFXConfig {};

    SupplyCurrentLimitConfiguration SupplyLimit {true, 35, 60, 0.1};

    wristMotorFXConfig.slot0.kP = 0.1;
    wristMotorFXConfig.slot0.kI = 0;
    wristMotorFXConfig.slot0.kD = 0;
    wristMotorFXConfig.slot0.kF = 0;   
    wristMotorFXConfig.motionCruiseVelocity = 10000;
    wristMotorFXConfig.motionAcceleration = 5000;     
    wristMotorFXConfig.supplyCurrLimit = SupplyLimit;
    wristMotorFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    wristMotorFXConfig.openloopRamp = 0.2;
    wristMotorFXConfig.closedloopRamp = 0.5;
    wristMotorFXConfig.nominalOutputForward = 0.03;
    wristMotorFXConfig.nominalOutputReverse = -0.03;
    wristMotorFXConfig.peakOutputForward = 1;
    wristMotorFXConfig.peakOutputReverse = -1;

    mWristMotor->ConfigFactoryDefault();
    mWristMotor->ConfigAllSettings(wristMotorFXConfig);
    mWristMotor->SetInverted(true);
    mWristMotor->SetNeutralMode(NeutralMode::Brake);
    mWristMotor->SetSelectedSensorPosition(0);
    mWristMotor->SetSensorPhase(true);
}

// This method will be called once per scheduler run
void WristSub::Periodic() {

}

void WristSub::stop() {
  mWristMotor->StopMotor();
}

void WristSub::moveWrist(double output) {
  mWristMotor->Set(ControlMode::PercentOutput, output);
}

void WristSub::setWristPosition(double percent) {
  mWristMotor->Set(ControlMode::MotionMagic, mapdouble(percent, 0,100, 0, 16653));
}

void WristSub::InitSendable(wpi::SendableBuilder& builder) {
  builder.AddDoubleProperty("Angle", [this] { return mapdouble(mWristMotor->GetSelectedSensorPosition(), 0,16653, 0, 100); }, nullptr);
}
