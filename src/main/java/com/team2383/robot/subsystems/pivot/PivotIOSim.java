package com.team2383.robot.subsystems.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class PivotIOSim implements PivotIO {
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            5, 7);

    private TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public PivotIOSim() {
        goal = new TrapezoidProfile.State(0, 0);
        setpoint = goal;
    }

    public void updateInputs(PivotIOInputs inputs) {
        setpoint = profile.calculate(0.02, setpoint, goal);

        inputs.current = 0;
        inputs.appliedVolts = 0;
        inputs.velocityRadPerS = setpoint.velocity;

        inputs.desiredVelocity = setpoint.velocity;

        inputs.pivotAngle = setpoint.position;
        inputs.currentDesiredAngle = setpoint.position;

        inputs.desiredAngle = goal.position;
    }

    @Override
    public void setAngle(double angle) {
        goal = new TrapezoidProfile.State(Units.radiansToRotations(angle), 0);
    }
}
