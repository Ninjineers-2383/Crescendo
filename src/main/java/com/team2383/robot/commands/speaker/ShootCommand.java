package com.team2383.robot.commands.speaker;

import com.team2383.robot.commands.subsystem.indexer.IndexerCommand;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(IndexerSubsystem indexer) {
        addCommands(
                new IndexerCommand(indexer, () -> -1.0).withTimeout(1));
    }

}
