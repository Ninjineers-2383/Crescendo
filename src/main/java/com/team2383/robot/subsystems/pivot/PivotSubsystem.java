package com.team2383.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private final ArmFeedforward feedforward = new ArmFeedforward(PivotConstants.kS,
            PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);
    private final PIDController controller = new PIDController(PivotConstants.kP, PivotConstants.kI,
            PivotConstants.kD);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            PivotConstants.kMaxVelo, PivotConstants.kMaxAccel);

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    public PivotSubsystem(PivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        var setpoint_next = profile.calculate(0.02, setpoint, goal);

        Logger.recordOutput("Pivot/Set Position", setpoint_next.position);
        Logger.recordOutput("Pivot/Set Velocity", setpoint_next.velocity);
        Logger.recordOutput("Pivot/Goal Position", goal.position);

        double voltage = calculateVoltage(setpoint_next, setpoint, inputs.pivotAngle, inputs.velocityRadPerS);

        setpoint = setpoint_next;

        io.setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setVelocity(double velocity) {
        goal = new TrapezoidProfile.State(goal.position + velocity * 0.02, 0);
    }

    public double calculateVoltage(TrapezoidProfile.State setpoint, TrapezoidProfile.State prev, double angleRad,
            double angleRadPerSec) {
        double accel = (setpoint.velocity - angleRadPerSec) / 0.02;
        return feedforward.calculate(inputs.pivotAngle, setpoint.velocity, accel)
                + controller.calculate(angleRad,
                        setpoint.position);
    }

    public void setPosition(double angleRads) {
        goal = new TrapezoidProfile.State(angleRads, 0.0);
    }

    public boolean isFinished() {
        return Math.abs(inputs.pivotAngle - goal.position) < 0.01;
    }

    public double getAngle() {
        return inputs.pivotAngle;
    }
}
