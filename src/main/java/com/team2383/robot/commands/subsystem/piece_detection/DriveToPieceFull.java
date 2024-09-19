package com.team2383.robot.commands.subsystem.piece_detection;

import com.team2383.robot.commands.feeding.FullFeedCommand;
import com.team2383.robot.commands.feeding.IndexerBackOut;
import com.team2383.robot.commands.subsystem.drivetrain.DrivetrainHeadingCommand;
import com.team2383.robot.commands.subsystem.pivot.PivotPositionCommand;
import com.team2383.robot.commands.subsystem.pivot.PivotPresets;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class DriveToPieceFull extends SequentialCommandGroup {
    public DriveToPieceFull(PieceDetectionSubsystem m_pieceDetectionSubsystem,
            DrivetrainSubsystem m_drivetrainSubsystem,
            FeederSubsystem m_backFeederSubsystem, IndexerSubsystem m_indexerSubsystem,
            ShooterSubsystem m_shooterSubsystem, PivotSubsystem m_pivotSubsystem) {

        addCommands(
                new ParallelDeadlineGroup(
                        new DriveToPieceAuto(m_pieceDetectionSubsystem, m_drivetrainSubsystem,
                                m_backFeederSubsystem),
                        new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem,
                                m_backFeederSubsystem, PivotPresets.FEED_BACK)),
                new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(), false, false),
                        m_drivetrainSubsystem),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(m_indexerSubsystem::isBeamBreakTripped),
                                                new WaitUntilCommand(() -> !m_indexerSubsystem.isBeamBreakTripped())),
                                        new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem,
                                                m_pivotSubsystem,
                                                m_backFeederSubsystem, PivotPresets.FEED_BACK)),
                                new IndexerBackOut(m_indexerSubsystem),
                                new PivotPositionCommand(m_pivotSubsystem, PivotPresets.ZERO)).withTimeout(1),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DrivetrainHeadingCommand(m_drivetrainSubsystem,
                                                Rotation2d.fromDegrees(-90)),
                                        new ParallelDeadlineGroup(
                                                new DriveToPieceCommand(m_pieceDetectionSubsystem,
                                                        m_drivetrainSubsystem,
                                                        m_backFeederSubsystem),
                                                new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem,
                                                        m_pivotSubsystem,
                                                        m_backFeederSubsystem,
                                                        PivotPresets.FEED_BACK))),
                                new SequentialCommandGroup(
                                        new DrivetrainHeadingCommand(m_drivetrainSubsystem,
                                                Rotation2d.fromDegrees(90)),
                                        new ParallelDeadlineGroup(
                                                new DriveToPieceCommand(m_pieceDetectionSubsystem,
                                                        m_drivetrainSubsystem,
                                                        m_backFeederSubsystem),
                                                new FullFeedCommand(m_shooterSubsystem, m_indexerSubsystem,
                                                        m_pivotSubsystem,
                                                        m_backFeederSubsystem,
                                                        PivotPresets.FEED_BACK))),
                                m_drivetrainSubsystem::isOnLeftSide),
                        m_backFeederSubsystem::isBeamBreakTripped));
    }
}
