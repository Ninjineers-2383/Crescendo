package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOFalcon500 implements ShooterIO {
    private final TalonFX leftMotor = new TalonFX(ShooterConstants.kLeftMotorID);
    private final TalonFX rightMotor = new TalonFX(ShooterConstants.kRightMotorID);

    private final VoltageOut voltageOut = new VoltageOut(0, false, false, false, false);

    double voltage = 0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftCurrent = leftMotor.getSupplyCurrent().getValue();
        inputs.rightCurrent = rightMotor.getSupplyCurrent().getValue();
        inputs.power = voltage;
    }

    @Override
    public void setPower(double power) {
        voltage = power * 12.0;
        leftMotor.setControl(voltageOut.withOutput(voltage));
        rightMotor.setControl(voltageOut.withOutput(voltage));
    }
}
