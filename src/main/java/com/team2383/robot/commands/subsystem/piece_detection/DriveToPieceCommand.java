package com.team2383.robot.commands.subsystem.piece_detection;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.feeder.FeederSubsystem;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionSubsystem;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPieceCommand extends Command {
    private PieceDetectionSubsystem pieceDetectionSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private FeederSubsystem feederSubsystem;

    private SlewRateLimiter m_driveRateLimiter;

    private MedianFilter medianFilter = new MedianFilter(5);

    public DriveToPieceCommand(PieceDetectionSubsystem pieceDetectionSubsystem,
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

        medianFilter.reset();
    }

    @Override
    public void execute() {
        if (pieceDetectionSubsystem.inputs.frontSeesTarget) {
            Rotation2d desireRotation2d = Rotation2d.fromDegrees(
                    medianFilter.calculate(drivetrainSubsystem.getHeading()
                            .minus(Rotation2d.fromDegrees(pieceDetectionSubsystem.inputs.frontYaw)).getDegrees()));
            drivetrainSubsystem.setHeading(desireRotation2d);
            drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                            m_driveRateLimiter.calculate(
                                    -4 * getRobotSpeedMultiplier(pieceDetectionSubsystem.inputs.frontPitch)),
                            0, 0),
                    false, true, new Translation2d(-Units.inchesToMeters(27 / 2.0), 0));
        }
    }

    public double getRobotSpeedMultiplier(double pitch) {
        if (pitch > 0) {
            return 1;
        } else {
            return (pitch + 40.0) * (1.0 / 40.0);
        }
    }

    @Override
    public boolean isFinished() {
        return feederSubsystem.isBeamBreakTripped();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.endManualHeadingControl();
    }
}
