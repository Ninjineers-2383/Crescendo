package com.team2383.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.lib.math.ThrottleSoftener;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class JoystickDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrain;

    private final Supplier<Rotation2d> m_rotSupply;
    private final Supplier<Translation2d> m_moveSupply;
    private final BooleanSupplier m_fieldRelative;
    private final IntSupplier m_hatSupplier;

    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(1000);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(1000);
    private final SlewRateLimiter m_oRateLimiter = new SlewRateLimiter(1500);

    public JoystickDriveCommand(DrivetrainSubsystem drivetrain, Supplier<Translation2d> moveSupplier,
            Supplier<Rotation2d> rotation, BooleanSupplier fieldRelative, IntSupplier hatSupplier) {
        m_drivetrain = drivetrain;

        m_moveSupply = moveSupplier;
        m_rotSupply = rotation;
        m_fieldRelative = fieldRelative;
        m_hatSupplier = hatSupplier;

        addRequirements(m_drivetrain);

    }

    @Override
    public void execute() {
        Translation2d move = m_moveSupply.get();
        double x = -ThrottleSoftener.soften(move.getX())
                * DriveConstants.kMaxSpeed;
        double y = -ThrottleSoftener.soften(move.getY())
                * DriveConstants.kMaxSpeed;
        double omega = -ThrottleSoftener.soften(m_rotSupply.get().getRadians()) * 0.75;
        int hatPosition = m_hatSupplier.getAsInt();

        Rotation2d rotVelocity = new Rotation2d(m_oRateLimiter.calculate(omega));

        m_drivetrain.drive(
                new Translation2d(
                        m_xRateLimiter.calculate(x),
                        m_yRateLimiter.calculate(y)),
                rotVelocity,
                m_fieldRelative.getAsBoolean(),
                getCenterOfRotation(hatPosition));
    }

    private Translation2d getCenterOfRotation(int hatPosition) {
        if (!(hatPosition >= 0 && hatPosition <= 360)) {
            return new Translation2d(0, 0);
        }
        // Allows rotating around the swerve modules
        Translation2d pos = new Translation2d((Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 3, 0);
        pos = pos.rotateBy(Rotation2d.fromDegrees(-hatPosition));
        return new Translation2d((Math.sqrt(3) * DriveConstants.kTrackWidthMeters) / 12, 0).plus(pos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
