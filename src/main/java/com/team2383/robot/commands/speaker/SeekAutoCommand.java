package com.team2383.robot.commands.speaker;

import org.littletonrobotics.junction.Logger;

import com.team2383.robot.FieldConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class SeekAutoCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;

    public SeekAutoCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        Translation2d drivetrainPose = m_drivetrain.getPose().getTranslation();
        Rotation2d heading = m_drivetrain.getHeading();

        Translation2d seekingTranslation = FieldConstants.getSpeakerLocation();

        Logger.recordOutput("Swerve/SeekingTranslation2d", seekingTranslation);

        Logger.recordOutput("Swerve/SeekingTranslation3d", new Translation3d(seekingTranslation.getX(),
                seekingTranslation.getY(), Units.inchesToMeters(78.324)));

        Rotation2d angle = new Rotation2d(Math.atan2(seekingTranslation.getY() - drivetrainPose.getY(),
                seekingTranslation.getX() - drivetrainPose.getX()));

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angle = angle.plus(new Rotation2d(Math.PI));
        }

        if (heading.minus(angle).getDegrees() > 90) {
            angle = angle.plus(new Rotation2d(Math.PI));
        } else if (heading.minus(angle).getDegrees() < -90) {
            angle = angle.minus(new Rotation2d(Math.PI));
        }

        m_drivetrain.setRotationTargetOverride(heading);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.disableRotationTargetOverride();
    }
}
