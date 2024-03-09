package com.team2383.robot.commands.subsystem.piece_detection;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.indexer.IndexerSubsystem;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionSubsystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPieceCommand extends Command {
    private PieceDetectionSubsystem pieceDetectionSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private IndexerSubsystem indexerSubsystem;

    private SlewRateLimiter m_driveRateLimiter;

    public DriveToPieceCommand(PieceDetectionSubsystem pieceDetectionSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, IndexerSubsystem indexerSubsystem) {
        this.pieceDetectionSubsystem = pieceDetectionSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(drivetrainSubsystem, pieceDetectionSubsystem);
    }

    @Override
    public void initialize() {
        m_driveRateLimiter = new SlewRateLimiter(3.0, -3.0,
                drivetrainSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond);
    }

    @Override
    public void execute() {
        if (pieceDetectionSubsystem.inputs.frontSeesTarget) {
            drivetrainSubsystem.setHeading(
                    drivetrainSubsystem.getHeading()
                            .minus(Rotation2d.fromDegrees(pieceDetectionSubsystem.inputs.frontYaw)));
            drivetrainSubsystem.drive(new ChassisSpeeds(m_driveRateLimiter.calculate(-4), 0, 0), false, true);

        }
    }

    @Override
    public boolean isFinished() {
        return indexerSubsystem.isBeamBreakTripped();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.endManualHeadingControl();
    }
}
