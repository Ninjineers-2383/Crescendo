package com.team2383.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(WristConstants.kS,
            WristConstants.kV, WristConstants.kA);
    private final PIDController controller = new PIDController(WristConstants.kP, WristConstants.kI,
            WristConstants.kD);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration);

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, goal);

    public WristSubsystem(WristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Wrist", inputs);

        profile = new TrapezoidProfile(constraints, goal, setpoint);

        var setpoint_next = profile.calculate(0.02);

        Logger.getInstance().recordOutput("Wrist/Set Position", setpoint_next.position);
        Logger.getInstance().recordOutput("Wrist/Set Velocity", setpoint_next.velocity);

        double voltage = calculateVoltage(setpoint_next, setpoint, inputs.wristAngle, inputs.velocityRadPerSec);

        setpoint = setpoint_next;

        io.setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public double calculateVoltage(TrapezoidProfile.State setpoint, TrapezoidProfile.State prev, double angleRad,
            double angleRadPerSec) {
        double accel = (setpoint.velocity - angleRadPerSec) / 0.02;
        return feedforward.calculate(setpoint.velocity, accel) + controller.calculate(angleRad,
                setpoint.position);
    }

    public void setPosition(double positionMeters) {
        goal = new TrapezoidProfile.State(positionMeters, 0.0);
    }

    public boolean isFinished() {
        return Math.abs(inputs.wristAngle - goal.position) < 0.01;
    }

    public double getAngle() {
        return inputs.wristAngle;
    }
}
