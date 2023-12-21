package com.team2383.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class FeederIOFalcon500 implements FeederIO {
    final TalonFX motor = new TalonFX(FeederConstants.kMotorID);

    final VoltageOut voltageOut = new VoltageOut(0);

    double voltage = 0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.current = motor.getSupplyCurrent().getValue();
        inputs.power = voltage;
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        motor.setControl(voltageOut.withOutput(voltage));
    }
}
