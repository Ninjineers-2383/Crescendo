package com.team2383.robot.commands.auto;

import com.team2383.robot.commands.speaker.SeekAndShootCommand;
import com.team2383.robot.commands.speaker.SeekCommand;
import com.team2383.robot.commands.speaker.ShootCommand;
import com.team2383.robot.commands.subsystem.drivetrain.auto.StartPathCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OneRingAuto extends SequentialCommandGroup {

    public OneRingAuto(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, FeederSubsystem feeder,
            ShooterSubsystem shooter, IndexerSubsystem indexer) {
        addCommands(
                new WaitCommand(0.02),
                new InstantCommand(() -> drivetrain.forceHeading(drivetrain.getPose().getRotation())),
                new SeekAndShootCommand(drivetrain, pivot, shooter, indexer),
                new StartPathCommand(drivetrain, "DriveTo1"),
                new SeekCommand(drivetrain, pivot).withTimeout(1),
                new ShootCommand(shooter, indexer));
    }

}
