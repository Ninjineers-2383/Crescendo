package com.team2383.robot.commands.subsystem.piece_detection;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionSubsystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPieceAuto extends Command {
    private PieceDetectionSubsystem pieceDetectionSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private FeederSubsystem feederSubsystem;

    private SlewRateLimiter m_driveRateLimiter;

    public DriveToPieceAuto(PieceDetectionSubsystem pieceDetectionSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, FeederSubsystem feederSubsystem) {
        this.pieceDetectionSubsystem = pieceDetectionSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.feederSubsystem = feederSubsystem;

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
            drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                            m_driveRateLimiter.calculate(
                                    -4 * getRobotSpeedMultiplier(pieceDetectionSubsystem.inputs.frontPitch)),
                            0, 0),
                    false, true);
        }
    }

    public double getRobotSpeedMultiplier(double pitch) {
        if (pitch > 0) {
            return 1;
        } else {
            return (pitch + 30.0) * (1.0 / 30.0);
        }
    }

    @Override
    public boolean isFinished() {
        return feederSubsystem.isBeamBreakTripped() || drivetrainSubsystem.hasCrossedCenterLine();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.endManualHeadingControl();
    }
}
