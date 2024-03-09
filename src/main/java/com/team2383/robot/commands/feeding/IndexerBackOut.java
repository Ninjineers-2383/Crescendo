package com.team2383.robot.commands.feeding;

import com.team2383.robot.subsystems.indexer.IndexerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IndexerBackOut extends Command {
    private final IndexerSubsystem indexer;

    public IndexerBackOut(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setPower(0.5);
    }

    @Override
    public boolean isFinished() {
        return indexer.isBeamBreakTripped();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }
}
