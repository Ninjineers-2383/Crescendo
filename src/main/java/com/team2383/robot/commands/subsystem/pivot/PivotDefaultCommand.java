package com.team2383.robot.commands.subsystem.pivot;

import java.util.function.DoubleSupplier;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotDefaultCommand extends Command {
    private final PivotSubsystem pivot;
    private final DoubleSupplier angle;
    private double previousAngle = 0;

    public PivotDefaultCommand(PivotSubsystem pivot, DoubleSupplier angle) {
        this.pivot = pivot;
        this.angle = angle;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        if (angle.getAsDouble() != previousAngle) {
            pivot.setPosition(angle.getAsDouble());
            previousAngle = angle.getAsDouble();
        }
    }

}
