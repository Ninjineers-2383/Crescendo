package com.team2383.robot.commands.auto;

import com.team2383.robot.commands.speaker.SeekCommand;
import com.team2383.robot.commands.speaker.ShootCommand;
import com.team2383.robot.commands.subsystem.drivetrain.StartPathCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneRingAuto extends SequentialCommandGroup {

    public OneRingAuto(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, FeederSubsystem feeder,
            ShooterSubsystem shooter, IndexerSubsystem indexer) {
        addCommands(
                new StartPathCommand(drivetrain, "DriveTo1"),
                new SeekCommand(drivetrain, pivot),
                new ShootCommand(shooter, indexer));
    }

}
