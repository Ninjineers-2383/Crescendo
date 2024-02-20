package com.team2383.robot.subsystems.trap_arm;

import org.littletonrobotics.junction.Logger;

import com.team2383.lib.controller.TunableArmFeedforward;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapArmSubsystem extends SubsystemBase {
    private final TrapArmIO io;
    private final TrapArmIOInputsAutoLogged inputs = new TrapArmIOInputsAutoLogged();

    public TrapArmSubsystem(TrapArmIO trapArmIO) {
        io = trapArmIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TrapArm", inputs);
    }

    public void setAngle(double angle) {
        io.setAngle(angle);
    }

    public void setVelocity(double velocity) {
        io.setAngle(inputs.desiredAngle + 0.02 * velocity);
    }

    public boolean isDone() {
        return Math.abs(inputs.pivotAngle - inputs.desiredAngle) < 0.01;
    }

    public void setFeedforward(TunableArmFeedforward feedforward) {
        io.setFeedforward(feedforward);
    }

    public void setPIDController(PIDController controller) {
        io.setPIDController(controller);
    }
}
