package com.team2383.robot.commands.subsystem.piece_detection;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.robot.subsystems.piece_detection.PieceDetectionSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPieceCommand extends Command {
    private PieceDetectionSubsystem pieceDetectionSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private boolean isFront;

    public DriveToPieceCommand(PieceDetectionSubsystem pieceDetectionSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, boolean isFront) {
        this.pieceDetectionSubsystem = pieceDetectionSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.isFront = isFront;

        addRequirements(drivetrainSubsystem, pieceDetectionSubsystem);
    }

    @Override
    public void execute() {
        if (isFront) {
            if (pieceDetectionSubsystem.inputs.frontSeesTarget) {
                drivetrainSubsystem.setHeading(
                        drivetrainSubsystem.getHeading()
                                .minus(Rotation2d.fromDegrees(pieceDetectionSubsystem.inputs.frontYaw)));
            }

            if (drivetrainSubsystem.headingIsFinished()) {
                drivetrainSubsystem.drive(new ChassisSpeeds(2, 0, 0), false, true);
            }
        } else {
            if (pieceDetectionSubsystem.inputs.rearSeesTarget) {
                drivetrainSubsystem.setHeading(
                        drivetrainSubsystem.getHeading()
                                .minus(Rotation2d.fromDegrees(pieceDetectionSubsystem.inputs.rearYaw)));
            }

            if (drivetrainSubsystem.headingIsFinished()) {
                drivetrainSubsystem.drive(new ChassisSpeeds(-2, 0, 0), false, true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.endManualHeadingControl();
    }
}
