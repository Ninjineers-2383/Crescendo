package com.team2383.robot.commands.subsystem.pivot;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotPositionCommand extends Command {
    private final PivotSubsystem pivot;
    private final double angle;
    private final double toleranceDegrees;

    public PivotPositionCommand(PivotSubsystem pivot, double angle, double toleranceDegrees) {
        this.pivot = pivot;
        this.angle = angle;
        this.toleranceDegrees = toleranceDegrees;

        addRequirements(pivot);
    }

    public PivotPositionCommand(PivotSubsystem pivot, double angle) {
        this.pivot = pivot;
        this.angle = angle;
        this.toleranceDegrees = -1;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setPosition(angle);
    }

    @Override
    public boolean isFinished() {
        if (toleranceDegrees <= 0) {
            return pivot.isFinished();
        } else {
            return pivot.isFinished(toleranceDegrees);
        }
    }

}
