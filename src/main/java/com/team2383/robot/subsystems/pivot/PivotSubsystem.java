package com.team2383.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class PivotSubsystem extends SubsystemBase {

    private final ArmFeedforward feedforward = new ArmFeedforward(pivotConstants.kS, pivotConstants.kG,
            pivotConstants.kV);
    private final PIDController controller = new PIDController(pivotConstants.kP, pivotConstants.kI, pivotConstants.kD);

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            pivotConstants.kMaxVelo, pivotConstants.kMaxAccel);

    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    public PivotSubsystem(PivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        io.setVoltage(0);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public double calculateVoltage(TrapezoidProfile.State setpoint, TrapezoidProfile.State prev, double angleRad,
            double angleRadiansPerSec) {

        return feedforward.calculate(inputs.pivotAngle, setpoint.velocity)
                + controller.calculate(angleRad, setpoint.position);
    }

    public double getAngle() {
        return inputs.pivotAngle;
    }
}
