package com.team2383.robot.commands.speaker;

import com.team2383.robot.commands.subsystem.indexer.IndexerCommand;
import com.team2383.robot.commands.subsystem.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        addCommands(
                new ShooterRPMCommand(shooter, () -> -5000, () -> 400, () -> 0).withTimeout(1),
                new IndexerCommand(indexer, () -> -1.0).withTimeout(1));
    }

}
