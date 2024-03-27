package com.team2383.robot.commands.amp;

import com.team2383.lib.util.AllianceUtil;
import com.team2383.robot.commands.subsystem.drivetrain.auto.PathfindCommandAlliance;
import com.team2383.robot.commands.subsystem.indexer.IndexerCommand;
import com.team2383.robot.commands.subsystem.pivot.PivotPositionCommand;
import com.team2383.robot.commands.subsystem.pivot.PivotPresets;
import com.team2383.robot.commands.subsystem.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpCommand extends SequentialCommandGroup {

    public ScoreAmpCommand(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, ShooterSubsystem shooter,
            IndexerSubsystem indexer) {

        addCommands(
                new PivotPositionCommand(pivot, PivotPresets.SCORE_AMP).alongWith(
                        new PathfindCommandAlliance(drivetrain,
                                () -> AllianceUtil
                                        .flipPose2d(new Pose2d(1.86, 7.75, new Rotation2d(Math.toRadians(90)))))),
                new ShooterRPMCommand(shooter, () -> -850, () -> 750, () -> 250)
                        .alongWith(new IndexerCommand(indexer, () -> -0.5)));
    }

}
