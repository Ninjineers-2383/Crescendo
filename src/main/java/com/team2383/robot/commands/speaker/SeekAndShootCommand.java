package com.team2383.robot.commands.speaker;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SeekAndShootCommand extends SequentialCommandGroup {

    public SeekAndShootCommand(DrivetrainSubsystem drivetrain, PivotSubsystem pivot,
            ShooterSubsystem shooter, IndexerSubsystem indexer) {

        addCommands(new ParallelCommandGroup(new ShootCommand(shooter, indexer), new SeekCommand(drivetrain, pivot)));

    }

}
