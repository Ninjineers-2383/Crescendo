package com.team2383.robot.commands.feeding;

import com.team2383.robot.subsystems.feeder.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PartialFeedCommand extends Command {
    private final FeederSubsystem feederSubsystem;

    public PartialFeedCommand(FeederSubsystem feederSubsystem) {
        this.feederSubsystem = feederSubsystem;
    }

    @Override
    public void execute() {
        feederSubsystem.setPower(-0.8);
    }

    @Override
    public boolean isFinished() {
        return feederSubsystem.isBeamBreakTripped();
    }
}
