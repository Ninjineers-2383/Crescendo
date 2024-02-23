package com.team2383.robot.commands.subsystem.pivot;

import com.team2383.robot.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotClimbCommand extends Command {
    private final PivotSubsystem pivot;

    public PivotClimbCommand(PivotSubsystem pivot) {
        this.pivot = pivot;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setPosition(Math.toRadians(-50));
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setPosition(pivot.getAngle());
    }

    @Override
    public boolean isFinished() {
        // return pivot.getAngle() <= -0.1;
        return false;
    }
}
