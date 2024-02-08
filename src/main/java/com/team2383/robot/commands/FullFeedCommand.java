package com.team2383.robot.commands;

import com.team2383.robot.commands.feeder.FeederPowerCommand;
import com.team2383.robot.commands.indexer.IndexerCommand;
import com.team2383.robot.commands.pivot.PivotPositionCommand;
import com.team2383.robot.commands.pivot.PivotPresets;
import com.team2383.robot.commands.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FullFeedCommand extends ParallelCommandGroup {

    public FullFeedCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, PivotSubsystem pivot,
            FeederSubsystem feeder) {

        addCommands(
                new FeederPowerCommand(feeder, () -> -1.0),
                new IndexerCommand(indexer, () -> -0.5),
                new ShooterRPMCommand(shooter, () -> 0, () -> -200, () -> 0),
                new PivotPositionCommand(pivot, PivotPresets.FEED_FRONT));
    }

}
