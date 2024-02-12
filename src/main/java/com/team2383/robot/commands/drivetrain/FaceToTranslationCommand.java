package com.team2383.robot.commands.drivetrain;

import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;;

public class FaceToTranslationCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final Supplier<Translation2d> m_translation;

    public FaceToTranslationCommand(DrivetrainSubsystem drivetrain, Supplier<Translation2d> translation) {
        m_drivetrain = drivetrain;
        m_translation = translation;
    }

    @Override
    public void execute() {
        Translation2d drivetrainTransform = m_drivetrain.getPose().getTranslation();

        Logger.recordOutput("Swerve/SeekingTranslation2d", m_translation.get());

        Logger.recordOutput("Swerve/SeekingTranslation3d", new Translation3d(m_translation.get().getX(),
                m_translation.get().getY(), Units.inchesToMeters(78.324)));

        Rotation2d angle = new Rotation2d(Math.atan2(m_translation.get().getY() - drivetrainTransform.getY(),
                m_translation.get().getX() - drivetrainTransform.getX()));

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angle = angle.plus(new Rotation2d(Math.PI));
        }

        // Rotation2d angleDiff =
        // m_drivetrain.getPose().getRotation().minus(m_drivetrain.getHeading());

        m_drivetrain.setHeading(angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setHeading(m_drivetrain.getHeading());
    }
}
