package com.team2383.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public PivotSubsystem(PivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    public void setVelocity(double velocity) {
        io.setAngle(Units.rotationsToRadians(inputs.desiredAngle) + velocity * 0.02);
    }

    public void setPosition(double angleRads) {
        io.setAngle(angleRads);
    }

    public boolean isFinished() {
        return Math.abs(inputs.pivotAngle - inputs.desiredAngle) < 0.01;
        // return false;
    }

    public double getAngle() {
        return Units.rotationsToRadians(inputs.pivotAngle);
    }

    public void setPIDController(PIDController controller) {
        io.setPIDController(controller);
    }

    public void setFeedforward(ArmFeedforward feedforward) {
        io.setFeedforward(feedforward);
    }
}
