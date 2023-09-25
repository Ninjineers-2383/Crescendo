package com.team2383.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ElevatorConstants.kS,
            ElevatorConstants.kV, ElevatorConstants.kA);
    private final PIDController controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI,
            ElevatorConstants.kD);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration);

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, goal);

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);

        profile = new TrapezoidProfile(constraints, goal, setpoint);

        var setpoint_next = profile.calculate(0.02);

        Logger.getInstance().recordOutput("Elevator Set Position", setpoint_next.position);
        Logger.getInstance().recordOutput("Elevator Set Velocity", setpoint_next.velocity);

        double voltage = calculateVoltage(setpoint_next, setpoint, inputs.positionM, inputs.velocityMPS);

        setpoint = setpoint_next;

        io.setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public double calculateVoltage(TrapezoidProfile.State setpoint, TrapezoidProfile.State prev, double positionMeters,
            double velocityMetersPerSecond) {
        double accel = (setpoint.velocity - prev.velocity) / 0.02;
        return feedforward.calculate(setpoint.velocity, accel) + controller.calculate(positionMeters,
                setpoint.position);
    }

    public void forcePosition(double positionMeters) {
        io.forcePosition(positionMeters);
    }

    public void setPosition(double positionMeters) {
        goal = new TrapezoidProfile.State(positionMeters, 0.0);
    }

    public boolean isFinished() {
        return Math.abs(inputs.positionM - goal.position) < 0.01;
    }

    public double getPosition() {
        return inputs.positionM;
    }
}
