package com.team2383.robot.commands.speaker;

import org.littletonrobotics.junction.Logger;

import com.team2383.robot.commands.subsystem.drivetrain.DrivetrainHeadingCommand;
import com.team2383.robot.commands.subsystem.drivetrain.FaceToSpeakerCommand;
import com.team2383.robot.commands.subsystem.pivot.PivotSeekCommand;
import com.team2383.robot.commands.subsystem.shooter.ShooterRPMCommand;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.pivot.PivotSubsystem;
import com.team2383.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SeekAndShootCommand extends ParallelDeadlineGroup {

    public SeekAndShootCommand(DrivetrainSubsystem drivetrain, PivotSubsystem pivot, ShooterSubsystem shooter,
            IndexerSubsystem indexer, boolean finish) {

        super(
                new SequentialCommandGroup(
                        new WaitCommand(0.04),
                        new InstantCommand(() -> {
                            Logger.recordOutput("SeekAndShoot/waiting", true);
                        }),
                        new WaitUntilCommand(() -> {
                            ChassisSpeeds speeds = drivetrain.getFieldRelativeSpeeds();
                            boolean isStopped = Math.abs(speeds.omegaRadiansPerSecond) < 0.001
                                    && Math.abs(speeds.vxMetersPerSecond) < 0.001
                                    && Math.abs(speeds.vyMetersPerSecond) < 0.001;
                            Logger.recordOutput("SeekAndShoot/isStopped", isStopped);
                            Logger.recordOutput("SeekAndShoot/pivot.isFinished", pivot.isFinished());
                            Logger.recordOutput("SeekAndShoot/drivetrain.headingIsFinished",
                                    drivetrain.headingIsFinished());
                            Logger.recordOutput("SeekAndShoot/shooter.isFinished", shooter.isFinished());
                            return pivot.isFinished() && isStopped && drivetrain.headingIsFinished()
                                    && shooter.isFinished();
                        }),
                        new InstantCommand(() -> {
                            Logger.recordOutput("SeekAndShoot/waiting", false);
                        }),
                        new ShootCommand(indexer, shooter)),
                new FaceToSpeakerCommand(drivetrain, finish),
                new PivotSeekCommand(pivot, drivetrain::getEstimatorPose3d, finish),
                new ShooterRPMCommand(shooter, () -> -4000, () -> 2000, () -> 0)
        // new RunCommand(() -> drivetrain
        // .forceHeading(DriverStation.getAlliance().get() ==
        // DriverStation.Alliance.Blue
        // ? drivetrain.getPose().getRotation()
        // : drivetrain.getPose().getRotation()
        // .plus(new Rotation2d(Math.PI))),
        // new Subsystem[0])
        );

    }

}
