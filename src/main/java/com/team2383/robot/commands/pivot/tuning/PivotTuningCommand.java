package com.team2383.robot.commands.pivot.tuning;

import com.team2383.lib.controller.TunableArmFeedforward;
import com.team2383.robot.subsystems.pivot.PivotConstants;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class PivotTuningCommand extends Command {
    private final PivotSubsystem pivot;
    private final DoubleSupplier speed;
    private final BooleanSupplier resetSetpoint;

    private final PIDController pidController = new PIDController(PivotConstants.kP, PivotConstants.kI,
            PivotConstants.kD);
    private final TunableArmFeedforward feedforward = new TunableArmFeedforward(PivotConstants.kS, PivotConstants.kG,
            PivotConstants.kV, PivotConstants.kA);

    public PivotTuningCommand(PivotSubsystem pivot, DoubleSupplier speed, BooleanSupplier resetSetpoint) {
        this.pivot = pivot;
        this.speed = speed;
        this.resetSetpoint = resetSetpoint;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        SmartDashboard.putData("Pivot PID", pidController);
        SmartDashboard.putData("Pivot Feedforward", feedforward);
    }

    @Override
    public void execute() {
        pivot.setPIDController(PivotConstants.kPIDController);
        pivot.setFeedforward(PivotConstants.kFeedforwardController);

        pivot.setVelocity(speed.getAsDouble() * 2 * Math.PI);

        if (resetSetpoint.getAsBoolean()) {
            pivot.setPosition(pivot.getAngle());
        }
    }

}