package com.team2383.robot.commands.pivot;

import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.pivot.PivotConstants;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotPositionCommand extends Command {
    private final PivotSubsystem pivot;
    private final DoubleSupplier angle;

    public PivotPositionCommand(PivotSubsystem pivot, DoubleSupplier angle) {
        this.pivot = pivot;
        this.angle = angle;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        // pivot.setFeedforward(PivotConstants.kFeedforwardController);
        // pivot.setPIDController(PivotConstants.kPIDController);
        pivot.setPosition(angle.getAsDouble());
    }

    @Override
    public void execute() {
        pivot.setPosition(angle.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // return pivot.isFinished();
        return false;
    }

}
