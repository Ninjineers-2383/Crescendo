package com.team2383.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public PivotSubsystem(PivotIO io) {
        this.io = io;
    }

    public void setVoltage(Measure<Voltage> voltage) {
        io.setVoltage(voltage.in(Volts));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    public void addPosition(double velocity) {
        io.setAngle(Units.rotationsToRadians(inputs.desiredAngle) + velocity);
    }

    public void setPosition(double angleRads) {
        io.setAngle(angleRads);
    }

    public boolean isFinished() {
        return Math.abs(inputs.pivotAngle - inputs.desiredAngle) < 0.005;
        // return false;
    }

    public double getAngle() {
        return Units.rotationsToRadians(inputs.pivotAngle);
    }

    public double getVelocity() {
        return Units.rotationsToRadians(inputs.currentVelocity);
    }

    public void setPIDController(PIDController controller) {
        io.setPIDController(controller);
    }

    public void setFeedforward(ArmFeedforward feedforward) {
        io.setFeedforward(feedforward);
    }

    public void disable() {
        io.disable();
    }

    public double getVoltage() {
        return inputs.appliedVolts;
    }
}
