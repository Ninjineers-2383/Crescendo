package com.team2383.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.team2383.lib.math.ThrottleSoftener;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveConstantHeading extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final Supplier<ChassisSpeeds> m_moveSupplier;
    private final BooleanSupplier m_fieldRelative;
    private final Rotation2d m_heading;

    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(1000);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(1000);

    public DriveConstantHeading(DrivetrainSubsystem drivetrain,
            Supplier<ChassisSpeeds> moveSupplier, BooleanSupplier fieldRelative, Rotation2d heading) {
        m_drivetrain = drivetrain;
        m_moveSupplier = moveSupplier;
        m_fieldRelative = fieldRelative;
        m_heading = heading;
    }

    @Override
    public void execute() {
        m_drivetrain.setHeading(m_heading);

        ChassisSpeeds move = m_moveSupplier.get();
        move.vxMetersPerSecond = m_xRateLimiter.calculate(-ThrottleSoftener.soften(move.vxMetersPerSecond)
                * DriveConstants.kMaxSpeed);

        move.vyMetersPerSecond = m_yRateLimiter.calculate(-ThrottleSoftener.soften(move.vyMetersPerSecond)
                * DriveConstants.kMaxSpeed);

        move.omegaRadiansPerSecond = 0.0;

        m_drivetrain.drive(
                move,
                m_fieldRelative.getAsBoolean());
    }
}
