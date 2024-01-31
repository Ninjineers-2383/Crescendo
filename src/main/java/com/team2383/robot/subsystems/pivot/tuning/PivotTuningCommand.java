package com.team2383.robot.subsystems.pivot.tuning;

import com.team2383.lib.controller.TunableArmFeedforward;
import com.team2383.robot.subsystems.pivot.PivotConstants;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class PivotTuningCommand extends Command {
    private final PivotSubsystem pivot;
    private final DoubleSupplier speed;

    private final PIDController pidController = new PIDController(PivotConstants.kP, PivotConstants.kI,
            PivotConstants.kD);
    private final TunableArmFeedforward feedforward = new TunableArmFeedforward(PivotConstants.kS, PivotConstants.kG,
            PivotConstants.kV, PivotConstants.kA);

    public PivotTuningCommand(PivotSubsystem pivot, DoubleSupplier speed) {
        this.pivot = pivot;
        this.speed = speed;
    }

    @Override
    public void execute() {
        pivot.setPIDController(pidController);
        pivot.setFeedforward(feedforward.toArmFeedforward());

        pivot.setVelocity(speed.getAsDouble());
    }

}
