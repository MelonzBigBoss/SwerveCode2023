// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SubSystems/GripperSub.h"

GripperSub::GripperSub() {
    mGripperMotor = new WPI_TalonFX(35, "Karen");

    TalonFXConfiguration gripperMotorFXConfig {};

    SupplyCurrentLimitConfiguration SupplyLimit {true, 25, 25, 0};

    gripperMotorFXConfig.slot0.kP = 0.1;
    gripperMotorFXConfig.slot0.kI = 0;
    gripperMotorFXConfig.slot0.kD = 0;
    gripperMotorFXConfig.slot0.kF = 0;    
    gripperMotorFXConfig.supplyCurrLimit = SupplyLimit;
    gripperMotorFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    gripperMotorFXConfig.openloopRamp = 0;
    gripperMotorFXConfig.closedloopRamp = 0.5;
    gripperMotorFXConfig.nominalOutputForward = 0.03;
    gripperMotorFXConfig.nominalOutputReverse = -0.03;
    gripperMotorFXConfig.peakOutputForward = 1;
    gripperMotorFXConfig.peakOutputReverse = -1;

    mGripperMotor->ConfigFactoryDefault();
    mGripperMotor->ConfigAllSettings(gripperMotorFXConfig);
    mGripperMotor->SetInverted(false);
    mGripperMotor->SetNeutralMode(NeutralMode::Brake);
    mGripperMotor->SetSelectedSensorPosition(0);

    distSensor = new rev::Rev2mDistanceSensor{rev::Rev2mDistanceSensor::Port::kMXP, rev::Rev2mDistanceSensor::DistanceUnit::kInches};
}

// This method will be called once per scheduler run
void GripperSub::Periodic() {}

void GripperSub::InitSendable(wpi::SendableBuilder& builder) {
    builder.AddDoubleProperty("Distance", [this] {return distSensor->GetRange();}, nullptr);
}