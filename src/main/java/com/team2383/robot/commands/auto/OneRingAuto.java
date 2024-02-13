package com.team2383.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team2383.robot.commands.speaker.SeekCommand;
import com.team2383.robot.commands.speaker.ShootCommand;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
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
                AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("DriveTo1"),
                        DriveConstants.AUTO_CONSTRAINTS),
                new SeekCommand(drivetrain, pivot),
                new ShootCommand(shooter, indexer));
    }

}
