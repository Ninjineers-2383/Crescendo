package com.team2383.robot.commands.drivetrain;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class FaceToTranslationCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final Translation2d m_translation;

    public FaceToTranslationCommand(DrivetrainSubsystem drivetrain, Translation2d translation) {
        m_drivetrain = drivetrain;
        m_translation = translation;
    }

    @Override
    public void execute() {
        Translation2d drivetrainTransform = m_drivetrain.getDeadReckoningPose3d().getTranslation().toTranslation2d();

        Rotation2d angle = new Rotation2d(Math.atan2(m_translation.getY() - drivetrainTransform.getY(),
                m_translation.getX() - drivetrainTransform.getX()));

        m_drivetrain.setHeading(angle);
    }
}
