// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LEDSub.h"

LEDSub::LEDSub() {
    ctre::phoenix::led::CANdleConfiguration candleConfig{};
    candleConfig.statusLedOffWhenActive = true;
    candleConfig.disableWhenLOS = false;
    candleConfig.stripType = ctre::phoenix::led::LEDStripType::RGB;
    candleConfig.brightnessScalar = 1;
    candleConfig.vBatOutputMode = ctre::phoenix::led::VBatOutputMode::On;
    candleConfig.enableOptimizations = true;
    candleConfig.v5Enabled = true;

    m_candle.ConfigAllSettings(candleConfig);
    m_candle.ClearAnimation(0);
    //m_candle.SetLEDs(255 * (frc::DriverStation::IsEnabled()),255 * (frc::DriverStation::IsDisabled()),0,0,ledOffset,ledCount);  
    animation = new ctre::phoenix::led::LarsonAnimation(colora,colorb,colora/2 + colorb / 2,0,0.7,ledCount, ctre::phoenix::led::LarsonAnimation::BounceMode::Front, 7, ledOffset);    
    
};

// This method will be called once per scheduler run
void LEDSub::Periodic() {
    animation = new ctre::phoenix::led::LarsonAnimation(colora,colorb,colora/2 + colorb / 2,0,0.7,ledCount, ctre::phoenix::led::LarsonAnimation::BounceMode::Front, 7, ledOffset);    
    m_candle.Animate(*animation);
}
  
void LEDSub::InitSendable(wpi::SendableBuilder& builder) {
    builder.AddIntegerProperty("Color A", [this] {return colora;}, [this] (int c) {colora = c;} );
    builder.AddIntegerProperty("Color B", [this] {return colorb;}, [this] (int c) {colorb = c;} );

}
