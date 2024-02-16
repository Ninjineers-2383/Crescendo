package com.team2383.robot.commands.subsystem.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import com.team2383.lib.math.ThrottleSoftener;
import com.team2383.robot.subsystems.drivetrain.DriveConstants;
import com.team2383.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class JoystickDriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;

    private final Supplier<ChassisSpeeds> m_moveSupply;
    private final BooleanSupplier m_fieldRelative;

    private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(1000);
    private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(1000);
    private final SlewRateLimiter m_oRateLimiter = new SlewRateLimiter(1500);

    public JoystickDriveCommand(DrivetrainSubsystem drivetrain, Supplier<ChassisSpeeds> moveSupplier,
            BooleanSupplier fieldRelative) {
        m_drivetrain = drivetrain;

        m_moveSupply = moveSupplier;
        m_fieldRelative = fieldRelative;

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

        m_drivetrain.drive(
                move,
                m_fieldRelative.getAsBoolean(),
                false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
