package com.team2383.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

public class FeederIOFalcon500 implements FeederIO {
    private final TalonFX motor;

    private final VoltageOut voltageOut;

    double voltage = 0;

    public FeederIOFalcon500(int motorID) {
        motor = new TalonFX(motorID);
        OrchestraContainer.getInstance().addMotor(motor);

        voltageOut = new VoltageOut(0, false, false, false, false);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // inputs.current = motor.getSupplyCurrent().getValue();
        inputs.power = voltage;
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        motor.setControl(voltageOut.withOutput(voltage));
    }
}
