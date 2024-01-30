package com.team2383.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import com.team2383.lib.math.ThrottleSoftener;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class JoystickDriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;

    private final Supplier<ChassisSpeeds> m_moveSupply;
    private final BooleanSupplier m_fieldRelative;
    private final IntSupplier m_hatSupplier;

    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(1000);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(1000);
    private final SlewRateLimiter m_oRateLimiter = new SlewRateLimiter(1500);

    public JoystickDriveCommand(DrivetrainSubsystem drivetrain, Supplier<ChassisSpeeds> moveSupplier,
            BooleanSupplier fieldRelative, IntSupplier hatSupplier) {
        m_drivetrain = drivetrain;

        m_moveSupply = moveSupplier;
        m_fieldRelative = fieldRelative;
        m_hatSupplier = hatSupplier;

        addRequirements(m_drivetrain);

    }

    @Override
    public void execute() {
        ChassisSpeeds move = m_moveSupply.get();
        move.vxMetersPerSecond = m_xRateLimiter.calculate(-ThrottleSoftener.soften(move.vxMetersPerSecond)
                * DriveConstants.kMaxSpeed);
        move.vyMetersPerSecond = m_yRateLimiter.calculate(-ThrottleSoftener.soften(move.vyMetersPerSecond)
                * DriveConstants.kMaxSpeed);
        move.omegaRadiansPerSecond = m_oRateLimiter
                .calculate(-ThrottleSoftener.soften(move.omegaRadiansPerSecond) * 0.75);
        int hatPosition = m_hatSupplier.getAsInt();

        m_drivetrain.drive(
                move,
                m_fieldRelative.getAsBoolean(),
                getCenterOfRotation(hatPosition),
                true);
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
