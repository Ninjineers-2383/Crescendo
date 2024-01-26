package com.team2383.robot.commands.pivot;

import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotVelocityCommand extends Command {
    private final PivotSubsystem pivot;
    private final DoubleSupplier velocity;

    public PivotVelocityCommand(PivotSubsystem pivot, DoubleSupplier velocity) {
        this.pivot = pivot;
        this.velocity = velocity;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setVelocity(velocity.getAsDouble());
    }

}
