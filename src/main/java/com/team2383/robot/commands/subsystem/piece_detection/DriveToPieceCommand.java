package com.team2383.robot.commands.subsystem.piece_detection;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveToPieceCommand {
    private PieceDetectionSubsystem pieceDetectionSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;

    public DriveToPieceCommand(PieceDetectionSubsystem pieceDetectionSubsystem,
            DrivetrainSubsystem drivetrainSubsystem) {
        this.pieceDetectionSubsystem = pieceDetectionSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    public void execute() {
        if (pieceDetectionSubsystem.inputs.seesTarget) {
            drivetrainSubsystem.setHeading(
                    drivetrainSubsystem.getHeading().plus(new Rotation2d(pieceDetectionSubsystem.inputs.yaw)));
        }

        if (drivetrainSubsystem.headingIsFinished()) {
            drivetrainSubsystem.drive(new ChassisSpeeds(1, 0, 0), true, true);
        }
    }
}
