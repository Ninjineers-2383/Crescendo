package com.team2383.robot.commands.speaker;

import java.util.Set;

import com.team2383.robot.commands.subsystem.indexer.IndexerCommand;
import com.team2383.robot.commands.subsystem.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        Set<Subsystem> req = Set.of();
        addCommands(
                new ParallelRaceGroup(
                        new WaitUntilCommand(() -> !indexer.isBeamBreakTripped()),
                        new IndexerCommand(indexer, () -> -1.0).withTimeout(0.5)),
                new WaitCommand(0.1),
                new DeferredCommand(() -> new ShooterRPMCommand(shooter, () -> 0, () -> 0, () -> 0, true), req)
                        .withTimeout(0.02),
                new DeferredCommand(() -> new IndexerCommand(indexer, () -> 0), req).withTimeout(0.02));
    }

}
