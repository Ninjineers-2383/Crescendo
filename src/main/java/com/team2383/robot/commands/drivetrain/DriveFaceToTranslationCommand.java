package com.team2383.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.team2383.lib.math.ThrottleSoftener;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveFaceToTranslationCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final Translation2d m_translation;
    private final Supplier<Translation2d> m_moveSupplier;
    private final BooleanSupplier m_fieldRelative;

    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(1000);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(1000);

    public DriveFaceToTranslationCommand(DrivetrainSubsystem drivetrain, Translation2d translation,
            Supplier<Translation2d> moveSupplier, BooleanSupplier fieldRelative) {
        m_drivetrain = drivetrain;
        m_translation = translation;
        m_moveSupplier = moveSupplier;
        m_fieldRelative = fieldRelative;
    }

    @Override
    public void execute() {
        Translation2d drivetrainTransform = m_drivetrain.getDeadReckoningPose3d().getTranslation().toTranslation2d();

        Rotation2d angle = new Rotation2d(Math.atan2(m_translation.getY() - drivetrainTransform.getY(),
                m_translation.getX() - drivetrainTransform.getX()));

        m_drivetrain.setHeading(angle);

        Translation2d move = m_moveSupplier.get();
        double x = -ThrottleSoftener.soften(move.getX())
                * DriveConstants.kMaxSpeed;
        double y = -ThrottleSoftener.soften(move.getY())
                * DriveConstants.kMaxSpeed;

        m_drivetrain.drive(
                new Translation2d(
                        m_xRateLimiter.calculate(x),
                        m_yRateLimiter.calculate(y)),
                new Rotation2d(),
                m_fieldRelative.getAsBoolean(),
                new Translation2d(),
                false);
    }
}
