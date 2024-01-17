package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOFalcon500 implements ShooterIO {
    private final TalonFX leftMotor = new TalonFX(ShooterConstants.kLeftMotorID, "Drive");
    private final TalonFX rightMotor = new TalonFX(ShooterConstants.kRightMotorID, "Drive");

    private final VelocityVoltage voltageOut = new VelocityVoltage(0);

    double voltage = 0;

    public ShooterIOFalcon500() {
        Slot0Configs leftConfigs = new Slot0Configs();
        leftConfigs.kP = 1;
        leftConfigs.kI = 0;
        leftConfigs.kD = 0;
        leftConfigs.kA = 0.05;
        leftConfigs.kV = 0.05;
        leftConfigs.kS = 0.05;

        leftMotor.getConfigurator().apply(leftConfigs);

        Slot0Configs rightConfigs = new Slot0Configs();
        rightConfigs.kP = 1;
        rightConfigs.kI = 0;
        rightConfigs.kD = 0;
        rightConfigs.kA = 0.05;
        rightConfigs.kV = 0.05;
        rightConfigs.kS = 0.05;

        rightMotor.getConfigurator().apply(rightConfigs);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftCurrent = leftMotor.getSupplyCurrent().getValue();
        inputs.rightCurrent = rightMotor.getSupplyCurrent().getValue();
        inputs.leftVoltage = leftMotor.getMotorVoltage().getValue();
        inputs.rightVoltage = rightMotor.getMotorVoltage().getValue();
        inputs.leftPosition = leftMotor.getPosition().getValue();
        inputs.rightPosition = rightMotor.getPosition().getValue();
        inputs.leftVelocity = leftMotor.getVelocity().getValue();
        inputs.rightVelocity = rightMotor.getVelocity().getValue();
    }

    @Override
    public void setRPM(double RPM) {
        leftMotor.setControl(voltageOut.withVelocity(RPM / 60.0));
        rightMotor.setControl(voltageOut.withVelocity(RPM / 60.0));
    }
}
