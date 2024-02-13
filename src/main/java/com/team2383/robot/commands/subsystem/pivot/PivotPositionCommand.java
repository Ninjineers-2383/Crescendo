package com.team2383.robot.commands.subsystem.pivot;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotPositionCommand extends Command {
    private final PivotSubsystem pivot;
    private final double angle;

    public PivotPositionCommand(PivotSubsystem pivot, double angle) {
        this.pivot = pivot;
        this.angle = angle;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setPosition(angle);
    }

    @Override
    public boolean isFinished() {
        return pivot.isFinished();
    }

}
