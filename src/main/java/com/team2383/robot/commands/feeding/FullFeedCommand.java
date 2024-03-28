package com.team2383.robot.commands.feeding;

import com.team2383.robot.commands.subsystem.feeder.FeederPowerCommand;
import com.team2383.robot.commands.subsystem.indexer.IndexerCommand;
import com.team2383.robot.commands.subsystem.pivot.PivotPositionCommand;
import com.team2383.robot.commands.subsystem.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FullFeedCommand extends ParallelCommandGroup {

    public FullFeedCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, PivotSubsystem pivot,
            FeederSubsystem feeder, double angle) {

        addCommands(
                new FeederPowerCommand(feeder, () -> -0.8),
                new IndexerCommand(indexer, () -> -0.5),
                new ShooterRPMCommand(shooter, () -> 0, () -> -1000, () -> 0),
                new PivotPositionCommand(pivot, angle));
    }
}
