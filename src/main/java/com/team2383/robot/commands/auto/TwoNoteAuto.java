package com.team2383.robot.commands.auto;

import com.team2383.robot.commands.feeding.FullFeedCommand;
import com.team2383.robot.commands.speaker.SeekAndShootCommand;
import com.team2383.robot.commands.subsystem.drivetrain.DrivetrainHeadingCommand;
import com.team2383.robot.commands.subsystem.drivetrain.auto.FollowPathCommandAlliance;
import com.team2383.robot.commands.subsystem.feeder.FeederPowerCommand;
import com.team2383.robot.commands.subsystem.indexer.IndexerCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoNoteAuto extends SequentialCommandGroup {

    public TwoNoteAuto(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, FeederSubsystem feeder,
            ShooterSubsystem shooter, IndexerSubsystem indexer) {
        addCommands(
                new WaitCommand(0.02),
                new InstantCommand(() -> drivetrain.forceHeading(drivetrain.getPose().getRotation())),
                new SeekAndShootCommand(drivetrain, pivot, shooter, indexer),
                new FullFeedCommand(shooter, indexer, pivot, feeder).withTimeout(0.01),
                new DrivetrainHeadingCommand(drivetrain, new Rotation2d(Math.PI)),
                new FollowPathCommandAlliance(drivetrain, "DriveTo1"),
                new IndexerCommand(indexer, () -> 0.1).withTimeout(0.01),
                new FeederPowerCommand(feeder, () -> 0).withTimeout(0.1),
                new SeekAndShootCommand(drivetrain, pivot, shooter, indexer));
    }

}
