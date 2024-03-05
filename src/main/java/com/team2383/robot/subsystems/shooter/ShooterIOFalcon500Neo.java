package com.team2383.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import com.team2383.robot.subsystems.orchestra.OrchestraContainer;

import java.util.List;

public class ShooterIOFalcon500Neo implements ShooterIO {
    private final TalonFX topMotor = new TalonFX(ShooterConstants.kTopMotorID, "Drive");
    private final TalonFX bottomMotor = new TalonFX(ShooterConstants.kBottomMotorID, "Drive");

    private final CANSparkMax sideMotor = new CANSparkMax(ShooterConstants.kSideMotorID, MotorType.kBrushless);

    private final VelocityVoltage voltageOut = new VelocityVoltage(0);

    private double topBottomSetpoint = 0.0;
    private double sideSetpoint = 0.0;

    private final List<StatusSignal<Double>> topBottomSupplyCurrent;
    private final List<StatusSignal<Double>> topBottomSupplyVoltage;
    private final List<StatusSignal<Double>> topBottomPosition;
    private final List<StatusSignal<Double>> topBottomVelocityRPM;

    public ShooterIOFalcon500Neo() {
        OrchestraContainer.getInstance().addMotor(topMotor);
        OrchestraContainer.getInstance().addMotor(bottomMotor);

        topMotor.getConfigurator().apply(ShooterConstants.kTopConfigs);

        bottomMotor.getConfigurator().apply(ShooterConstants.kBottomConfigs);

        sideMotor.getPIDController().setP(ShooterConstants.kSideP, 0);
        sideMotor.getPIDController().setI(ShooterConstants.kSideI, 0);
        sideMotor.getPIDController().setD(ShooterConstants.kSideD, 0);
        sideMotor.getPIDController().setFF(ShooterConstants.kSideV, 0);

        sideMotor.setIdleMode(IdleMode.kBrake);

        topBottomSupplyCurrent = List.of(topMotor.getSupplyCurrent(), bottomMotor.getSupplyCurrent());
        topBottomSupplyVoltage = List.of(topMotor.getSupplyVoltage(), bottomMotor.getSupplyVoltage());
        topBottomPosition = List.of(topMotor.getPosition(), bottomMotor.getPosition());
        topBottomVelocityRPM = List.of(topMotor.getVelocity(), bottomMotor.getVelocity());

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                topBottomSupplyCurrent.get(0),
                topBottomSupplyCurrent.get(1),
                topBottomSupplyVoltage.get(0),
                topBottomSupplyVoltage.get(1),
                topBottomPosition.get(0),
                topBottomPosition.get(1),
                topBottomVelocityRPM.get(0),
                topBottomVelocityRPM.get(1));

        topMotor.optimizeBusUtilization(1.0);
        bottomMotor.optimizeBusUtilization(1.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorConnected = BaseStatusSignal.refreshAll(
                topBottomSupplyCurrent.get(0),
                topBottomSupplyVoltage.get(0),
                topBottomPosition.get(0),
                topBottomVelocityRPM.get(0))
                .isOK();

        inputs.bottomMotorConnected = BaseStatusSignal.refreshAll(
                topBottomSupplyCurrent.get(1),
                topBottomSupplyVoltage.get(1),
                topBottomPosition.get(1),
                topBottomVelocityRPM.get(1))
                .isOK();

        inputs.topCurrent = topBottomSupplyCurrent.get(0).getValueAsDouble();
        inputs.bottomCurrent = topBottomSupplyCurrent.get(1).getValueAsDouble();
        inputs.sideCurrent = sideMotor.getOutputCurrent();

        inputs.topVoltage = topBottomSupplyVoltage.get(0).getValueAsDouble();
        inputs.bottomVoltage = topBottomSupplyVoltage.get(1).getValueAsDouble();
        inputs.sideVoltage = sideMotor.getBusVoltage() * sideMotor.getAppliedOutput();

        inputs.topPosition = topBottomPosition.get(0).getValueAsDouble();
        inputs.bottomPosition = topBottomPosition.get(1).getValueAsDouble();
        inputs.sidePosition = sideMotor.getEncoder().getPosition();

        inputs.topVelocity = topBottomVelocityRPM.get(0).getValueAsDouble();
        inputs.bottomVelocity = topBottomVelocityRPM.get(1).getValueAsDouble();
        inputs.sideVelocity = sideMotor.getEncoder().getVelocity();

        inputs.topBottomSetpointRPM = topBottomSetpoint;
        inputs.sideSetpointRPM = sideSetpoint;
    }

    @Override
    public void setTopBottomRPM(double RPM, double differential) {
        topMotor.setControl(voltageOut.withVelocity((RPM - differential) / 60.0));
        bottomMotor.setControl(voltageOut.withVelocity((RPM + differential) / 60.0));

        topBottomSetpoint = RPM / 60.0;
    }

    @Override
    public void setSideRPM(double RPM) {
        sideMotor.getPIDController().setReference(RPM, ControlType.kVelocity, 0,
                ShooterConstants.kSideS * Math.signum(RPM));

        sideSetpoint = RPM;
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
