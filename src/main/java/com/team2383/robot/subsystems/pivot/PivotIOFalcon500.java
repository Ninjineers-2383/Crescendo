package com.team2383.robot.subsystems.pivot;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOFalcon500 implements PivotIO {

    private final TalonFX motor;
    private final VoltageOut voltageOut = new VoltageOut(0, false, false, false, false);

    public PivotIOFalcon500() {
        motor = new TalonFX(PivotConstants.kMotorID);

        motor.setInverted(true);

    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.current = motor.getSupplyCurrent().getValue();
        inputs.appliedVoltage = motor.getSupplyVoltage().getValue();
    }

    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    public void forceAngle(Rotation2d angle) {
    }

}
