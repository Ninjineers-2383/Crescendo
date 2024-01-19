package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

public class ShooterIOFalcon500Neo implements ShooterIO {
    private final TalonFX topMotor = new TalonFX(ShooterConstants.kTopMotorID, "Drive");
    private final TalonFX bottomMotor = new TalonFX(ShooterConstants.kBottomMotorID, "Drive");

    private final CANSparkMax sideMotor = new CANSparkMax(ShooterConstants.kSideMotorID, MotorType.kBrushless);

    private final VelocityVoltage voltageOut = new VelocityVoltage(0);

    public ShooterIOFalcon500Neo() {
        topMotor.getConfigurator().apply(ShooterConstants.kTopConfigs);

        bottomMotor.getConfigurator().apply(ShooterConstants.kBottomConfigs);

        sideMotor.getPIDController().setP(ShooterConstants.kBottomP);
        sideMotor.getPIDController().setI(ShooterConstants.kBottomI);
        sideMotor.getPIDController().setD(ShooterConstants.kBottomD);
        sideMotor.getPIDController().setFF(ShooterConstants.kBottomF);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topCurrent = topMotor.getSupplyCurrent().getValue();
        inputs.bottomCurrent = bottomMotor.getSupplyCurrent().getValue();
        inputs.sideCurrent = sideMotor.getOutputCurrent();

        inputs.topVoltage = topMotor.getMotorVoltage().getValue();
        inputs.bottomVoltage = bottomMotor.getMotorVoltage().getValue();
        inputs.sideVoltage = sideMotor.getBusVoltage() * sideMotor.getAppliedOutput();

        inputs.topPosition = topMotor.getPosition().getValue();
        inputs.bottomPosition = bottomMotor.getPosition().getValue();
        inputs.sidePosition = sideMotor.getEncoder().getPosition();

        inputs.topVelocity = topMotor.getVelocity().getValue();
        inputs.bottomVelocity = bottomMotor.getVelocity().getValue();
        inputs.sideVelocity = sideMotor.getEncoder().getVelocity();
    }

    @Override
    public void setTopBottomRPM(double RPM) {
        topMotor.setControl(voltageOut.withVelocity(RPM / 60.0));
        bottomMotor.setControl(voltageOut.withVelocity(RPM / 60.0));
    }

    @Override
    public void setSideRPM(double RPM) {
        sideMotor.getPIDController().setReference(RPM, ControlType.kVelocity);
    }

    @Override
    public void setTopBottomVoltage(double voltage) {
        topMotor.setVoltage(voltage);
        bottomMotor.setVoltage(voltage);
    }

    @Override
    public void setSideVoltage(double voltage) {
        sideMotor.setVoltage(voltage);
    }
}
