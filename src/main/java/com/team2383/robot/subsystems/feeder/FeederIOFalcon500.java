package com.team2383.robot.subsystems.feeder;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class FeederIOFalcon500 implements FeederIO {
    TalonFX motor = new TalonFX(FeederConstants.kMotorID);

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.current = motor.getSupplyCurrent();
        inputs.power = motor.getMotorOutputPercent();
    }

    @Override
    public void setPower(double power) {
        motor.set(TalonFXControlMode.PercentOutput, power);
    }
}
