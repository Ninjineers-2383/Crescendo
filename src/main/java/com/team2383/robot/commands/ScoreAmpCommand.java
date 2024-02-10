package com.team2383.robot.commands;

import com.team2383.lib.util.AllianceUtil;
import com.team2383.robot.commands.drivetrain.PathfindCommand;
import com.team2383.robot.commands.indexer.IndexerCommand;
import com.team2383.robot.commands.pivot.PivotPositionCommand;
import com.team2383.robot.commands.pivot.PivotPresets;
import com.team2383.robot.commands.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpCommand extends SequentialCommandGroup {

    public ScoreAmpCommand(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, ShooterSubsystem shooter,
            IndexerSubsystem indexer) {

        addCommands(
                new PivotPositionCommand(pivot, PivotPresets.SCORE_AMP).alongWith(
                        new PathfindCommand(drivetrain, () -> AllianceUtil.flipPose2d(DriveConstants.SPEAKER_POSE))),
                new IndexerCommand(indexer, () -> -0.5).withTimeout(0.4),
                new ShooterRPMCommand(shooter, () -> 0, () -> -500, () -> 0)
                        .alongWith(new IndexerCommand(indexer, () -> 0.45).withTimeout(1)));
    }

}
